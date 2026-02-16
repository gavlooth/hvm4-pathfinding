// Bidirectional BFS benchmark driver
// Undirected graph in C memory (CSR), HVM4 for reduction (Q4 trie visited sets).
// Compile: clang -O2 -o bench/bidir_bfs bench/bidir_bfs.c -lpthread
// Usage:   ./bench/bidir_bfs [V] [edges_per_node]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>

#include "../c3lib/csrc/hvm4_bridge.c"

// ---------------------------------------------------------------------------
// LCG random undirected graph generator
// ---------------------------------------------------------------------------

typedef struct { uint32_t s; } LCG;

static uint32_t lcg_next(LCG *r) {
  r->s = (r->s * 1103515245u + 12345u) & 0x7fffffffu;
  return r->s;
}

typedef struct { uint32_t u, v; } RawEdge;

// Generate undirected graph: chain + random edges, both directions stored.
// All weights = 1 (BFS is unweighted). CSR row_ptr/col_idx only.
static void gen_graph(uint32_t n, uint32_t epn, uint32_t seed,
                      uint32_t **row_ptr_out, uint32_t **col_idx_out,
                      uint32_t *edge_count_out) {
  LCG rng = { .s = seed };
  uint32_t target_e = n * epn; // directed edges target
  uint32_t max_e = target_e * 2 + 2 * n;
  RawEdge *edges = malloc(max_e * sizeof(RawEdge));
  uint32_t ne = 0;

  // Chain for connectivity (both directions)
  for (uint32_t i = 0; i + 1 < n; i++) {
    edges[ne++] = (RawEdge){i, i + 1};
    edges[ne++] = (RawEdge){i + 1, i};
  }

  // Random extra edges (both directions)
  uint32_t attempts = n * (epn - 1) * 2;
  for (uint32_t a = 0; a < attempts && ne < target_e * 2; a++) {
    uint32_t u = lcg_next(&rng) % n;
    uint32_t v = lcg_next(&rng) % n;
    if (u == v) continue;
    edges[ne++] = (RawEdge){u, v};
    edges[ne++] = (RawEdge){v, u};
  }

  // Counting sort into CSR
  uint32_t *rp = calloc(n + 2, sizeof(uint32_t));
  for (uint32_t i = 0; i < ne; i++) rp[edges[i].u + 1]++;
  for (uint32_t i = 1; i <= n; i++) rp[i] += rp[i - 1];

  uint32_t *ci = malloc(ne * sizeof(uint32_t));
  uint32_t *pos = malloc((n + 1) * sizeof(uint32_t));
  memcpy(pos, rp, (n + 1) * sizeof(uint32_t));

  for (uint32_t i = 0; i < ne; i++) {
    uint32_t u = edges[i].u;
    uint32_t p = pos[u]++;
    ci[p] = edges[i].v;
  }

  free(edges);
  free(pos);
  *row_ptr_out = rp;
  *col_idx_out = ci;
  *edge_count_out = ne;
}

// ---------------------------------------------------------------------------
// Reference BFS in C
// ---------------------------------------------------------------------------

static uint32_t bfs_reference(uint32_t n, uint32_t *rp, uint32_t *ci,
                              uint32_t src, uint32_t dest) {
  uint32_t *dist = malloc(n * sizeof(uint32_t));
  for (uint32_t i = 0; i < n; i++) dist[i] = 999999;
  dist[src] = 0;

  uint32_t *queue = malloc(n * sizeof(uint32_t));
  uint32_t qh = 0, qt = 0;
  queue[qt++] = src;

  while (qh < qt) {
    uint32_t u = queue[qh++];
    if (u == dest) break;
    for (uint32_t e = rp[u]; e < rp[u + 1]; e++) {
      uint32_t v = ci[e];
      if (dist[v] == 999999) {
        dist[v] = dist[u] + 1;
        queue[qt++] = v;
      }
    }
  }

  uint32_t result = dist[dest];
  free(dist);
  free(queue);
  return result;
}

// ---------------------------------------------------------------------------
// HVM4 source generation
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
  return ru.ru_maxrss;
}

#define APPEND(fmt, ...) pos += snprintf(buf + pos, cap - pos, fmt, ##__VA_ARGS__)
#define APPENDS(s) do { size_t _l = strlen(s); if (pos + _l < cap) { memcpy(buf + pos, s, _l); pos += _l; } } while(0)
#define L "\xce" "\xbb"

static char *gen_hvm4_source(uint32_t n, uint32_t src, uint32_t dest) {
  uint32_t depth = ceil_log4(n);

  size_t cap = 16384;
  char *buf = malloc(cap);
  size_t pos = 0;

  APPEND("@INF = 999999\n@DEPTH = %u\n", depth);

  // Q4 trie: get_lin + set (reuse from hybrid_bf.c)
  APPENDS(
    "@q4_get_lin = " L "&key. " L "&depth. " L "{"
    "#QE: #P{@INF, #QE{}}; "
    "#QL: " L "&val. #P{val, #QL{val}}; "
    "#Q: " L "c0. " L "c1. " L "c2. " L "c3. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4gl_Q(slot, next, nd, c0, c1, c2, c3)}\n"

    "@q4gl_Q = " L "{"
    "0: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "val. " L "nc0. #P{val, #Q{nc0, c1, c2, c3}}}(@q4_get_lin(next, nd, c0)); "
    "1: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "val. " L "nc1. #P{val, #Q{c0, nc1, c2, c3}}}(@q4_get_lin(next, nd, c1)); "
    "2: " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "val. " L "nc2. #P{val, #Q{c0, c1, nc2, c3}}}(@q4_get_lin(next, nd, c2)); "
    "" L "n. " L "next. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. "
    "" L "{#P: " L "val. " L "nc3. #P{val, #Q{c0, c1, c2, nc3}}}(@q4_get_lin(next, nd, c3))}\n"

    "@q4_set = " L "&key. " L "&val. " L "&depth. " L "{"
    "#QL: " L "old. #QL{val}; "
    "#QE: " L "{0: #QL{val}; " L "n. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4s_QE(slot, next, val, nd)}(depth); "
    "#Q: " L "c0. " L "c1. " L "c2. " L "c3. "
    "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
    "@q4s_Q(slot, next, val, nd, c0, c1, c2, c3)}\n"

    "@q4s_QE = " L "{"
    "0: " L "next. " L "val. " L "nd. #Q{@q4_set(next, val, nd, #QE{}), #QE{}, #QE{}, #QE{}}; "
    "1: " L "next. " L "val. " L "nd. #Q{#QE{}, @q4_set(next, val, nd, #QE{}), #QE{}, #QE{}}; "
    "2: " L "next. " L "val. " L "nd. #Q{#QE{}, #QE{}, @q4_set(next, val, nd, #QE{}), #QE{}}; "
    "" L "n. " L "next. " L "val. " L "nd. #Q{#QE{}, #QE{}, #QE{}, @q4_set(next, val, nd, #QE{})}}\n"

    "@q4s_Q = " L "{"
    "0: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. #Q{@q4_set(next, val, nd, c0), c1, c2, c3}; "
    "1: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. #Q{c0, @q4_set(next, val, nd, c1), c2, c3}; "
    "2: " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. #Q{c0, c1, @q4_set(next, val, nd, c2), c3}; "
    "" L "n. " L "next. " L "val. " L "nd. " L "c0. " L "c1. " L "c2. " L "c3. #Q{c0, c1, c2, @q4_set(next, val, nd, c3)}}\n"
  );

  // min helper
  APPENDS("@min = " L "&a. " L "&b. " L "{0: b; " L "n. a}(a < b)\n");

  // expand_nbrs: expand neighbors of node u (i..deg)
  APPENDS(
    "@expand_nbrs = " L "&u. " L "&i. " L "&deg. " L "front. " L "&d. " L "vis. "
    "@enb_ck(i < deg, u, i, deg, front, d, vis)\n"

    "@enb_ck = " L "{"
    "0: " L "u. " L "i. " L "deg. " L "front. " L "d. " L "vis. #P{front, vis}; "
    "" L "n. " L "&u. " L "&i. " L "deg. " L "front. " L "&d. " L "vis. "
    "! &v = %graph_target(u, i); "
    "" L "{#P: " L "&vd. " L "vis2. "
    "@enb_go(vd < @INF, u, i, deg, front, d, v, vis2)"
    "}(@q4_get_lin(v, @DEPTH, vis))}\n"

    "@enb_go = " L "{"
    "0: " L "u. " L "i. " L "deg. " L "front. " L "&d. " L "&v. " L "vis. "
    "@expand_nbrs(u, i + 1, deg, v <> front, d, @q4_set(v, d, @DEPTH, vis)); "
    "" L "m. " L "u. " L "i. " L "deg. " L "front. " L "d. " L "v. " L "vis. "
    "@expand_nbrs(u, i + 1, deg, front, d, vis)}\n"
  );

  // expand_all: expand all nodes in frontier
  APPENDS(
    "@exp_all = " L "{"
    "[]: " L "nf. " L "d. " L "vis. #P{nf, vis}; "
    "<>: " L "&u. " L "rest. " L "nf. " L "&d. " L "vis. "
    "" L "{#P: " L "f2. " L "v2. "
    "@exp_all(rest, f2, d, v2)"
    "}(@expand_nbrs(u, 0, %graph_deg(u), nf, d, vis))}\n"
  );

  // check_meet: check frontier nodes against opposite visited
  APPENDS(
    "@chk_mt = " L "{"
    "[]: " L "d. " L "best. " L "opp. #P{best, opp}; "
    "<>: " L "&v. " L "rest. " L "&d. " L "&best. " L "opp. "
    "" L "{#P: " L "&od. " L "opp2. "
    "@cm_go(od < @INF, rest, d, best, od, opp2)"
    "}(@q4_get_lin(v, @DEPTH, opp))}\n"

    "@cm_go = " L "{"
    "0: " L "rest. " L "d. " L "best. " L "od. " L "opp. @chk_mt(rest, d, best, opp); "
    "" L "m. " L "rest. " L "&d. " L "best. " L "od. " L "opp. @chk_mt(rest, d, @min(best, d + od), opp)}\n"
  );

  // bfs_loop: main loop
  APPENDS(
    "@bfs = " L "&fd. " L "&bd. " L "&best. " L "ff. " L "bf. " L "fv. " L "bv. "
    "! &nfd = fd + 1; ! &nbd = bd + 1; "
    "" L "{#P: " L "&nff. " L "fv2. "
    "" L "{#P: " L "&bst2. " L "bv2. "
    "" L "{#P: " L "&nbf. " L "bv3. "
    "" L "{#P: " L "&bst3. " L "fv3. "
    "@bdn(bst3 < @INF, nff, nbf, fv3, bv3, nfd, nbd, bst3)"
    "}(@chk_mt(nbf, nbd, bst2, fv2))"
    "}(@exp_all(bf, [], nbd, bv2))"
    "}(@chk_mt(nff, nfd, best, bv))"
    "}(@exp_all(ff, [], nfd, fv))\n"
  );

  // bfs_done + empty checks
  APPENDS(
    "@bdn = " L "{"
    "0: " L "ff. " L "bf. " L "fv. " L "bv. " L "fd. " L "bd. " L "bst. "
    "@bmt(ff, bf, fv, bv, fd, bd); "
    "" L "m. " L "ff. " L "bf. " L "fv. " L "bv. " L "fd. " L "bd. " L "bst. bst}\n"

    "@bmt = " L "{"
    "[]: " L "bf. " L "fv. " L "bv. " L "fd. " L "bd. @INF; "
    "<>: " L "h. " L "t. " L "bf. " L "fv. " L "bv. " L "fd. " L "bd. "
    "@bm2(bf, h <> t, fv, bv, fd, bd)}\n"

    "@bm2 = " L "{"
    "[]: " L "ff. " L "fv. " L "bv. " L "fd. " L "bd. @INF; "
    "<>: " L "h2. " L "t2. " L "ff. " L "fv. " L "bv. " L "fd. " L "bd. "
    "@bfs(fd, bd, @INF, ff, h2 <> t2, fv, bv)}\n"
  );

  // main: init visited tries and start BFS
  APPEND(
    "@main = "
    "! fv = @q4_set(%u, 0, @DEPTH, #QE{}); "
    "! bv = @q4_set(%u, 0, @DEPTH, #QE{}); "
    "@bfs(0, 0, @INF, [%u], [%u], fv, bv)\n",
    src, dest, src, dest
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

  printf("=== Bidir BFS benchmark: V=%u, ~%u edges/node ===\n", V, epn);

  // Generate undirected graph
  uint32_t *rp, *ci, ne;
  gen_graph(V, epn, 42 + V, &rp, &ci, &ne);
  printf("Graph: V=%u  E=%u (directed)\n", V, ne);

  // Reference BFS
  uint32_t src = 0, dest = V - 1;
  uint32_t ref = bfs_reference(V, rp, ci, src, dest);
  printf("Reference: dist[%u->%u] = %u\n", src, dest, ref);

  // graph_weight not used for BFS but the bridge requires it
  uint32_t *wt = malloc(ne * sizeof(uint32_t));
  for (uint32_t i = 0; i < ne; i++) wt[i] = 1;

  // Init HVM4 runtime
  hvm4_lib_init();

  // Generate HVM4 source
  char *hvm_src = gen_hvm4_source(V, src, dest);
  printf("HVM4 source: %lu bytes\n", (unsigned long)strlen(hvm_src));
  if (V <= 10) {
    printf("--- HVM4 SOURCE ---\n%s--- END SOURCE ---\n", hvm_src);
  }

  // Setup
  hvm4_lib_reset();
  hvm4_graph_setup(rp, ci, wt, V);

  // Run
  uint32_t result;
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  int count = hvm4_run(hvm_src, 0, &result, 1);

  clock_gettime(CLOCK_MONOTONIC, &t1);
  double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;

  printf("Time: %.3f s\n", elapsed);
  printf("Peak RSS: %ld MB\n", peak_rss_kb() / 1024);

  if (count < 0) {
    printf("FAIL: hvm4_run returned %d\n", count);
    hvm4_lib_cleanup();
    free(hvm_src);
    free(rp);
    free(ci);
    free(wt);
    return 1;
  }

  printf("HVM4 result: %u\n", result);

  int ok = (count >= 1 && result == ref);
  if (ok) {
    printf("PASS: dist[%u->%u] = %u matches reference\n", src, dest, result);
  } else {
    printf("FAIL: got %u (count=%d), expected %u\n", result, count, ref);
  }

  free(hvm_src);
  free(rp);
  free(ci);
  free(wt);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
