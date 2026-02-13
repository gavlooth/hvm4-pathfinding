// Hybrid Bellman-Ford benchmark driver
// Graph in C memory (CSR), HVM4 for reduction only (radix-4 trie).
// Compile: clang -O2 -o bench/hybrid_bf bench/hybrid_bf.c -lpthread
// Usage:   ./bench/hybrid_bf [V] [edges_per_node]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>

#include "../c3lib/csrc/hvm4_bridge.c"

// ---------------------------------------------------------------------------
// LCG random graph generator (deterministic, matches bench/gen.py)
// ---------------------------------------------------------------------------

typedef struct { uint32_t s; } LCG;

static uint32_t lcg_next(LCG *r) {
  r->s = (r->s * 1103515245u + 12345u) & 0x7fffffffu;
  return r->s;
}

typedef struct { uint32_t u, v, w; } RawEdge;

static void gen_graph(uint32_t n, uint32_t epn, uint32_t seed,
                      uint32_t **row_ptr_out, uint32_t **col_idx_out,
                      uint32_t **weight_out, uint32_t *edge_count_out) {
  LCG rng = { .s = seed };
  uint32_t target_e = n * epn;
  uint32_t max_e = target_e + n;
  RawEdge *edges = malloc(max_e * sizeof(RawEdge));
  uint32_t ne = 0;

  // Chain for connectivity
  for (uint32_t i = 0; i + 1 < n; i++) {
    uint32_t w = lcg_next(&rng) % 10 + 1;
    edges[ne++] = (RawEdge){i, i + 1, w};
  }

  // Random extra edges (simple dedup via rejection)
  uint32_t attempts = n * (epn - 1) * 2;
  for (uint32_t a = 0; a < attempts && ne < target_e; a++) {
    uint32_t u = lcg_next(&rng) % n;
    uint32_t v = lcg_next(&rng) % n;
    if (u == v) continue;
    uint32_t w = lcg_next(&rng) % 20 + 1;
    edges[ne++] = (RawEdge){u, v, w};
  }

  // Counting sort into CSR
  uint32_t *rp = calloc(n + 2, sizeof(uint32_t));
  for (uint32_t i = 0; i < ne; i++) rp[edges[i].u + 1]++;
  for (uint32_t i = 1; i <= n; i++) rp[i] += rp[i - 1];

  uint32_t *ci = malloc(ne * sizeof(uint32_t));
  uint32_t *wt = malloc(ne * sizeof(uint32_t));
  uint32_t *pos = malloc((n + 1) * sizeof(uint32_t));
  memcpy(pos, rp, (n + 1) * sizeof(uint32_t));

  for (uint32_t i = 0; i < ne; i++) {
    uint32_t u = edges[i].u;
    uint32_t p = pos[u]++;
    ci[p] = edges[i].v;
    wt[p] = edges[i].w;
  }

  free(edges);
  free(pos);
  *row_ptr_out = rp;
  *col_idx_out = ci;
  *weight_out  = wt;
  *edge_count_out = ne;
}

// ---------------------------------------------------------------------------
// Reference Bellman-Ford in C (for validation)
// ---------------------------------------------------------------------------

static void bf_reference(uint32_t n, uint32_t *rp, uint32_t *ci, uint32_t *wt,
                         uint32_t src, uint32_t *dist) {
  for (uint32_t i = 0; i < n; i++) dist[i] = 999999;
  dist[src] = 0;
  for (uint32_t round = 0; round + 1 < n; round++) {
    int changed = 0;
    for (uint32_t u = 0; u < n; u++) {
      if (dist[u] == 999999) continue;
      for (uint32_t e = rp[u]; e < rp[u + 1]; e++) {
        uint32_t nd = dist[u] + wt[e];
        if (nd < dist[ci[e]]) {
          dist[ci[e]] = nd;
          changed = 1;
        }
      }
    }
    if (!changed) break;
  }
}

// ---------------------------------------------------------------------------
// HVM4 source generation (same template as C3 code generator)
// ---------------------------------------------------------------------------

static uint32_t ceil_log4(uint32_t n) {
  if (n <= 4) return 1;
  uint32_t d = 1, cap = 4;
  while (cap < n) { d++; cap *= 4; }
  return d;
}

static long peak_rss_kb(void) {
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return ru.ru_maxrss; // KB on Linux
}

#define APPEND(fmt, ...) pos += snprintf(buf + pos, cap - pos, fmt, ##__VA_ARGS__)
#define APPENDS(s) do { size_t _l = strlen(s); if (pos + _l < cap) { memcpy(buf + pos, s, _l); pos += _l; } } while(0)
// UTF-8 lambda: split to avoid C hex escape merging (\xcebb would be one escape)
#define L "\xce" "\xbb"

static char *gen_hvm4_source(uint32_t n, uint32_t source) {
  uint32_t depth = ceil_log4(n);
  uint32_t rounds = n > 1 ? n - 1 : 1;

  size_t cap = 16384;
  char *buf = malloc(cap);
  size_t pos = 0;

  APPEND("@INF = 999999\n@DEPTH = %u\n@V = %u\n", depth, n);

  // Q4 trie defs
  APPENDS(
    "@q4_get_lin = " L "&key. " L "&depth. " L "{"
    "#QE: #P{@INF, #QE{}}; "
    "#QL: " L "&val. #P{val, #QL{val}}; "
    "#Q: " L "c0. " L "c1. " L "c2. " L "c3. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4_get_lin_Q(slot, next, nd, c0, c1, c2, c3)}\n"

    "@q4_get_lin_Q = " L "{"
    "0: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "val. " L "new_c0. #P{val, #Q{new_c0, c1, c2, c3}}}(@q4_get_lin(next, nd, c0)); "
    "1: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "val. " L "new_c1. #P{val, #Q{c0, new_c1, c2, c3}}}(@q4_get_lin(next, nd, c1)); "
    "2: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "val. " L "new_c2. #P{val, #Q{c0, c1, new_c2, c3}}}(@q4_get_lin(next, nd, c2)); "
    "" L "n. " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "val. " L "new_c3. #P{val, #Q{c0, c1, c2, new_c3}}}(@q4_get_lin(next, nd, c3))}\n"

    "@q4_get = " L "&key. " L "&depth. " L "{"
    "#QE: @INF; #QL: " L "val. val; "
    "#Q: " L "c0. " L "c1. " L "c2. " L "c3. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4_get_Q(slot, next, nd, c0, c1, c2, c3)}\n"

    "@q4_get_Q = " L "{"
    "0: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. @q4_get(next, nd, c0); "
    "1: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. @q4_get(next, nd, c1); "
    "2: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. @q4_get(next, nd, c2); "
    "" L "n. " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. @q4_get(next, nd, c3)}\n"

    "@q4_set = " L "&key. " L "&val. " L "&depth. " L "{"
    "#QL: " L "old. #QL{val}; "
    "#QE: " L "{0: #QL{val}; " L "n. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4_set_QE(slot, next, val, nd)}(depth); "
    "#Q: " L "c0. " L "c1. " L "c2. " L "c3. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4_set_Q(slot, next, val, nd, c0, c1, c2, c3)}\n"

    "@q4_set_QE = " L "{"
    "0: " L "next. " L "val. " L "nd. #Q{@q4_set(next, val, nd, #QE{}), #QE{}, #QE{}, #QE{}}; "
    "1: " L "next. " L "val. " L "nd. #Q{#QE{}, @q4_set(next, val, nd, #QE{}), #QE{}, #QE{}}; "
    "2: " L "next. " L "val. " L "nd. #Q{#QE{}, #QE{}, @q4_set(next, val, nd, #QE{}), #QE{}}; "
    "" L "n. " L "next. " L "val. " L "nd. #Q{#QE{}, #QE{}, #QE{}, @q4_set(next, val, nd, #QE{})}}\n"

    "@q4_set_Q = " L "{"
    "0: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. #Q{@q4_set(next, val, nd, c0), c1, c2, c3}; "
    "1: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. #Q{c0, @q4_set(next, val, nd, c1), c2, c3}; "
    "2: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. #Q{c0, c1, @q4_set(next, val, nd, c2), c3}; "
    "" L "n. " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. #Q{c0, c1, c2, @q4_set(next, val, nd, c3)}}\n"

    "@q4_min_update_f = " L "&key. " L "&val. " L "&depth. " L "{"
    "#QL: " L "&old. " L "{0: #P{#QL{old}, 0}; " L "n. #P{#QL{val}, 1}}(val < old); "
    "#QE: " L "{0: #P{#QL{val}, 1}; " L "n. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4_muf_QE(slot, next, val, nd)}(depth); "
    "#Q: " L "c0. " L "c1. " L "c2. " L "c3. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4_muf_Q(slot, next, val, nd, c0, c1, c2, c3)}\n"

    "@q4_muf_QE = " L "{"
    "0: " L "next. " L "val. " L "nd. " L "{#P: " L "child. " L "c. #P{#Q{child, #QE{}, #QE{}, #QE{}}, c}}(@q4_min_update_f(next, val, nd, #QE{})); "
    "1: " L "next. " L "val. " L "nd. " L "{#P: " L "child. " L "c. #P{#Q{#QE{}, child, #QE{}, #QE{}}, c}}(@q4_min_update_f(next, val, nd, #QE{})); "
    "2: " L "next. " L "val. " L "nd. " L "{#P: " L "child. " L "c. #P{#Q{#QE{}, #QE{}, child, #QE{}}, c}}(@q4_min_update_f(next, val, nd, #QE{})); "
    "" L "n. " L "next. " L "val. " L "nd. " L "{#P: " L "child. " L "c. #P{#Q{#QE{}, #QE{}, #QE{}, child}, c}}(@q4_min_update_f(next, val, nd, #QE{}))}\n"

    "@q4_muf_Q = " L "{"
    "0: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "new_c0. " L "c. #P{#Q{new_c0, c1, c2, c3}, c}}(@q4_min_update_f(next, val, nd, c0)); "
    "1: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "new_c1. " L "c. #P{#Q{c0, new_c1, c2, c3}, c}}(@q4_min_update_f(next, val, nd, c1)); "
    "2: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "new_c2. " L "c. #P{#Q{c0, c1, new_c2, c3}, c}}(@q4_min_update_f(next, val, nd, c2)); "
    "" L "n. " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "new_c3. " L "c. #P{#Q{c0, c1, c2, new_c3}, c}}(@q4_min_update_f(next, val, nd, c3))}\n"
  );

  // Relaxation logic via FFI
  APPENDS(
    "@relax_edges = " L "&u. " L "&i. " L "&deg. " L "&du. " L "{"
    "#S: " L "&dist. " L "&changed. "
    "" L "{0: #S{dist, changed}; "
    "" L "n. "
    "! &v = %graph_target(u, i); "
    "! new_d = du + %graph_weight(u, i); "
    "" L "{#P: " L "new_dist. " L "c. "
    "@relax_edges(u, i + 1, deg, du, #S{new_dist, changed + c})"
    "}(@q4_min_update_f(v, new_d, @DEPTH, dist))"
    "}(i < deg)}\n"

    "@relax_node_et = " L "&u. " L "{"
    "#S: " L "dist. " L "&changed. "
    "" L "{#P: " L "&du. " L "dist2. "
    "@relax_node_go(du < @INF, u, du, dist2, changed)"
    "}(@q4_get_lin(u, @DEPTH, dist))}\n"

    "@relax_node_go = " L "{"
    "0: " L "u. " L "du. " L "dist. " L "changed. #S{dist, changed}; "
    "" L "n. " L "&u. " L "&du. " L "dist. " L "changed. "
    "@relax_edges(u, 0, %graph_deg(u), du, #S{dist, changed})}\n"

    "@node_loop = " L "&i. " L "&state. "
    "" L "{0: state; "
    "" L "n. @node_loop(i + 1, @relax_node_et(i, state))"
    "}(i < @V)\n"

    "@relax_round_et = " L "{"
    "#S: " L "dist. " L "old_changed. "
    "@node_loop(0, #S{dist, 0})}\n"

    // bf_loop references @relax_round_et by name (avoids DUP-REF bug)
    "@bf_loop = " L "{"
    "0: " L "state. state; "
    "" L "n. " L "state. @bf_check(n, @relax_round_et(state))}\n"
    "@bf_check = " L "&n. " L "{"
    "#S: " L "dist. " L "changed. @bf_check_go(changed, n, dist)}\n"
    "@bf_check_go = " L "{"
    "0: " L "n. " L "dist. #S{dist, 0}; "
    "" L "m. " L "n. " L "dist. @bf_loop(n - 1, %compact(#S{dist, 1}))}\n"
  );

  // Init + run + extract
  APPEND("@init_dist = @q4_set(%u, 0, @DEPTH, #QE{})\n", source);
  APPEND("@bf = @bf_loop(%u, #S{@init_dist, 1})\n", rounds);

  APPENDS(
    "@extract_go = " L "&i. " L "{"
    "#P: " L "&val. " L "dist. "
    "" L "{0: [val]; "
    "" L "n. val <> @extract_go(i + 1, @q4_get_lin(i + 1, @DEPTH, dist))"
    "}(i + 1 < @V)}\n"

    "@main = " L "{#S: " L "dist. " L "c. "
    "" L "{0: []; "
    "" L "n. @extract_go(0, @q4_get_lin(0, @DEPTH, dist))"
    "}(@V)}(@bf)\n"
  );

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V   = argc > 1 ? (uint32_t)atoi(argv[1]) : 100;
  uint32_t epn = argc > 2 ? (uint32_t)atoi(argv[2]) : 4;

  printf("=== Hybrid BF benchmark: V=%u, ~%u edges/node ===\n", V, epn);

  // Generate graph
  uint32_t *rp, *ci, *wt, ne;
  gen_graph(V, epn, 42 + V, &rp, &ci, &wt, &ne);
  printf("Graph: V=%u  E=%u  CSR=%lu KB\n", V, ne,
         (unsigned long)((V + 1 + ne * 2) * 4 / 1024));

  // Reference solution
  uint32_t *ref = malloc(V * sizeof(uint32_t));
  bf_reference(V, rp, ci, wt, 0, ref);
  printf("Reference: dist[%u]=%u\n", V - 1, ref[V - 1]);

  // Init HVM4 runtime
  hvm4_lib_init();

  // Generate HVM4 source
  char *src = gen_hvm4_source(V, 0);
  printf("HVM4 source: %lu bytes\n", (unsigned long)strlen(src));
  if (V <= 10) {
    printf("--- HVM4 SOURCE ---\n%s--- END SOURCE ---\n", src);
    printf("CSR row_ptr: ");
    for (uint32_t i = 0; i <= V; i++) printf("%u ", rp[i]);
    printf("\nCSR col_idx: ");
    for (uint32_t i = 0; i < ne; i++) printf("%u ", ci[i]);
    printf("\nCSR weight: ");
    for (uint32_t i = 0; i < ne; i++) printf("%u ", wt[i]);
    printf("\n");
  }

  // Setup
  hvm4_lib_reset();
  hvm4_graph_setup(rp, ci, wt, V);

  // Run
  uint32_t *out = malloc(V * sizeof(uint32_t));
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  int count = hvm4_run(src, 0, out, (int)V);

  clock_gettime(CLOCK_MONOTONIC, &t1);
  double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;

  printf("Time: %.3f s\n", elapsed);
  printf("Peak RSS: %ld MB\n", peak_rss_kb() / 1024);

  if (count < 0) {
    printf("FAIL: hvm4_run returned %d\n", count);
    hvm4_lib_cleanup();
    return 1;
  }

  printf("Extracted %d values\n", count);

  // Validate
  int ok = 1;
  if ((uint32_t)count != V) {
    printf("FAIL: expected %u values, got %d\n", V, count);
    ok = 0;
  } else {
    for (uint32_t i = 0; i < V; i++) {
      if (out[i] != ref[i]) {
        printf("FAIL: dist[%u] = %u, expected %u\n", i, out[i], ref[i]);
        ok = 0;
        if (i > 5) { printf("  ... (more mismatches)\n"); break; }
      }
    }
  }

  if (ok) printf("PASS: all %u distances match reference\n", V);

  free(src);
  free(out);
  free(ref);
  free(rp);
  free(ci);
  free(wt);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
