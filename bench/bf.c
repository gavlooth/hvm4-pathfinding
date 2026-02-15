// Bellman-Ford benchmark driver (C-side distance array, HVM4 control flow)
// Graph in C memory (CSR), distances in C array accessed via FFI.
// HVM4 manages BF loop with early termination — no trie, no OOM wall.
// Compile: clang -O2 -o bench/bf bench/bf.c -lpthread
// Usage:   ./bench/bf [V] [edges_per_node]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>

#include "../c3lib/csrc/hvm4_bridge.c"

// ---------------------------------------------------------------------------
// LCG random graph generator
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

  // Random extra edges
  uint32_t attempts = n * (epn - 1) * 2;
  for (uint32_t a = 0; a < attempts && ne < target_e; a++) {
    uint32_t u = lcg_next(&rng) % n;
    uint32_t v = lcg_next(&rng) % n;
    if (u == v) continue;
    uint32_t w = lcg_next(&rng) % 20 + 1;
    edges[ne++] = (RawEdge){u, v, w};
  }

  // Convert to CSR
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
// Reference: Bellman-Ford in C with early termination
// ---------------------------------------------------------------------------

#define INF 999999999u

static void bf_reference(uint32_t n, uint32_t *rp, uint32_t *ci, uint32_t *wt,
                         uint32_t src, uint32_t *dist) {
  for (uint32_t i = 0; i < n; i++) dist[i] = INF;
  dist[src] = 0;
  for (uint32_t round = 0; round + 1 < n; round++) {
    int changed = 0;
    for (uint32_t u = 0; u < n; u++) {
      if (dist[u] >= INF) continue;
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
// Distance FFI: C-side distance array
// ---------------------------------------------------------------------------

static uint32_t *g_dist;
static uint32_t  g_dist_v;

// %dist_get(v) -> distance of node v
static Term prim_dist_get(Term *args) {
  Term v = wnf(args[0]);
  uint32_t node = term_val(v);
  return term_new_num(node < g_dist_v ? g_dist[node] : INF);
}

// %dist_upd(v, d) -> min-update: if d < dist[v], set and return 1, else 0
static Term prim_dist_upd(Term *args) {
  Term v = wnf(args[0]);
  Term d = wnf(args[1]);
  uint32_t node = term_val(v);
  uint32_t val  = term_val(d);
  if (node < g_dist_v && val < g_dist[node]) {
    g_dist[node] = val;
    return term_new_num(1);
  }
  return term_new_num(0);
}

static void dist_setup(uint32_t *dist, uint32_t v) {
  g_dist   = dist;
  g_dist_v = v;
  prim_register("dist_get", 8, 1, prim_dist_get);
  prim_register("dist_upd", 8, 2, prim_dist_upd);
}

// ---------------------------------------------------------------------------
// HVM4 source generation
// ---------------------------------------------------------------------------

static long peak_rss_kb(void) {
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return ru.ru_maxrss;
}

#define APPEND(fmt, ...) pos += snprintf(buf + pos, cap - pos, fmt, ##__VA_ARGS__)
#define APPENDS(s) do { size_t _l = strlen(s); if (pos + _l < cap) { memcpy(buf + pos, s, _l); pos += _l; } } while(0)
#define L "\xce" "\xbb"

static char *gen_hvm4_source(uint32_t V, uint32_t dest) {
  uint32_t rounds = V > 1 ? V - 1 : 1;

  size_t cap = 4096;
  char *buf = malloc(cap);
  size_t pos = 0;

  APPEND("@INF = %u\n@V = %u\n@DEST = %u\n@ROUNDS = %u\n",
         INF, V, dest, rounds);

  // @rel: relax edges from node u (du = dist[u] already fetched)
  // Returns sum of changes (0 or 1 per edge)
  APPENDS(
    "@rel = " L "u. " L "du. " L "&i. " L "&deg. "
    "@rgo(i < deg, u, du, i, deg)\n"

    "@rgo = " L "{"
    "0: " L "u. " L "du. " L "i. " L "deg. 0; "
    "" L "n. " L "&u. " L "&du. " L "&i. " L "deg. "
    "! v = %graph_target(u, i); "
    "! w = %graph_weight(u, i); "
    "! ndv = du + w; "
    "%dist_upd(v, ndv) + @rel(u, du, i + 1, deg)}\n"
  );

  // @rall: relax all nodes 0..V-1, return total change count
  APPENDS(
    "@rall = " L "&u. @rago(u < @V, u)\n"

    "@rago = " L "{"
    "0: " L "u. 0; "
    "" L "n. " L "&u. ! &du = %dist_get(u); @rask(du < @INF, u, du)}\n"

    "@rask = " L "{"
    "0: " L "u. " L "du. @rall(u + 1); "
    "" L "n. " L "&u. " L "du. "
    "@rel(u, du, 0, %graph_deg(u)) + @rall(u + 1)}\n"
  );

  // @bf: BF main loop with early termination
  APPENDS(
    "@bf = " L "round. @bfgo(round)\n"

    "@bfgo = " L "{"
    "0: %dist_get(@DEST); "
    "" L "k. ! changed = @rall(0); @bfet(changed, k)}\n"

    "@bfet = " L "{"
    "0: " L "k. %dist_get(@DEST); "
    "" L "n. " L "k. @bf(k - 1)}\n"
  );

  APPENDS("@main = @bf(@ROUNDS)\n");

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V   = argc > 1 ? (uint32_t)atoi(argv[1]) : 1000;
  uint32_t epn = argc > 2 ? (uint32_t)atoi(argv[2]) : 4;

  printf("=== Bellman-Ford benchmark (C-side dist): V=%u, ~%u edges/node ===\n", V, epn);

  // Generate graph
  uint32_t *rp, *ci, *wt, ne;
  gen_graph(V, epn, 42 + V, &rp, &ci, &wt, &ne);
  printf("Graph: V=%u  E=%u\n", V, ne);

  // C reference
  uint32_t *ref = malloc(V * sizeof(uint32_t));
  bf_reference(V, rp, ci, wt, 0, ref);
  printf("Reference: dist[%u] = %u\n", V - 1, ref[V - 1]);

  // HVM4 setup
  hvm4_lib_init();
  char *src = gen_hvm4_source(V, V - 1);
  printf("HVM4 source: %lu bytes\n", (unsigned long)strlen(src));
  if (V <= 20) {
    printf("--- HVM4 SOURCE ---\n%s--- END SOURCE ---\n", src);
  }

  hvm4_lib_reset();
  hvm4_graph_setup(rp, ci, wt, V);

  // Initialize C-side distance array
  uint32_t *dist = malloc(V * sizeof(uint32_t));
  for (uint32_t i = 0; i < V; i++) dist[i] = INF;
  dist[0] = 0;
  dist_setup(dist, V);

  // Run HVM4
  uint32_t result;
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  int count = hvm4_run(src, 0, &result, 1);

  clock_gettime(CLOCK_MONOTONIC, &t1);
  double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;

  printf("Time: %.3f s\n", elapsed);
  printf("Peak RSS: %ld MB\n", peak_rss_kb() / 1024);

  if (count < 0) {
    printf("FAIL: hvm4_run returned %d\n", count);
    hvm4_lib_cleanup();
    free(src); free(rp); free(ci); free(wt); free(ref); free(dist);
    return 1;
  }

  printf("HVM4 result: dist[%u] = %u\n", V - 1, result);

  // Validate all distances against reference
  int ok = 1;
  int mismatches = 0;
  for (uint32_t i = 0; i < V; i++) {
    if (dist[i] != ref[i]) {
      if (mismatches < 5)
        printf("MISMATCH: dist[%u] = %u, expected %u\n", i, dist[i], ref[i]);
      mismatches++;
      ok = 0;
    }
  }

  if (ok) {
    printf("PASS: all %u distances match reference\n", V);
  } else {
    printf("FAIL: %d/%u mismatches\n", mismatches, V);
  }

  free(src); free(rp); free(ci); free(wt); free(ref); free(dist);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
