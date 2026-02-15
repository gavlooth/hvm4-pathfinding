// Contraction Hierarchies benchmark driver (C-side preprocessing, HVM4 query)
// Rank = node ID. DAG (all edges forward u<v) avoids shortcut need.
// HVM4 runs bidirectional Dijkstra: forward from src, backward from dest.
// Compile: clang -O2 -o bench/ch bench/ch.c -lpthread
// Usage:   ./bench/ch [V] [edges_per_node]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>

#include "../c3lib/csrc/hvm4_bridge.c"

// ---------------------------------------------------------------------------
// LCG random DAG generator (all edges u -> v where u < v)
// ---------------------------------------------------------------------------

typedef struct { uint32_t s; } LCG;

static uint32_t lcg_next(LCG *r) {
  r->s = (r->s * 1103515245u + 12345u) & 0x7fffffffu;
  return r->s;
}

typedef struct { uint32_t u, v, w; } RawEdge;

static void gen_dag(uint32_t n, uint32_t epn, uint32_t seed,
                    uint32_t **row_ptr_out, uint32_t **col_idx_out,
                    uint32_t **weight_out, uint32_t *edge_count_out) {
  LCG rng = { .s = seed };
  uint32_t target_e = n * epn;
  uint32_t max_e = target_e + n;
  RawEdge *edges = malloc(max_e * sizeof(RawEdge));
  uint32_t ne = 0;

  // Chain for connectivity (always forward: i -> i+1)
  for (uint32_t i = 0; i + 1 < n; i++) {
    uint32_t w = lcg_next(&rng) % 10 + 1;
    edges[ne++] = (RawEdge){i, i + 1, w};
  }

  // Random forward edges (u < v)
  uint32_t attempts = n * (epn - 1) * 2;
  for (uint32_t a = 0; a < attempts && ne < target_e; a++) {
    uint32_t u = lcg_next(&rng) % n;
    uint32_t v = lcg_next(&rng) % n;
    if (u >= v) continue;
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

// Build reverse CSR from a forward CSR
static void build_reverse_csr(uint32_t n, uint32_t ne,
                               uint32_t *fwd_rp, uint32_t *fwd_ci, uint32_t *fwd_wt,
                               uint32_t **rev_rp_out, uint32_t **rev_ci_out,
                               uint32_t **rev_wt_out) {
  uint32_t *rrp = calloc(n + 2, sizeof(uint32_t));
  for (uint32_t u = 0; u < n; u++)
    for (uint32_t e = fwd_rp[u]; e < fwd_rp[u + 1]; e++)
      rrp[fwd_ci[e] + 1]++;
  for (uint32_t i = 1; i <= n; i++) rrp[i] += rrp[i - 1];

  uint32_t *rci = malloc(ne * sizeof(uint32_t));
  uint32_t *rwt = malloc(ne * sizeof(uint32_t));
  uint32_t *pos = malloc((n + 1) * sizeof(uint32_t));
  memcpy(pos, rrp, (n + 1) * sizeof(uint32_t));

  for (uint32_t u = 0; u < n; u++)
    for (uint32_t e = fwd_rp[u]; e < fwd_rp[u + 1]; e++) {
      uint32_t v = fwd_ci[e];
      uint32_t p = pos[v]++;
      rci[p] = u;
      rwt[p] = fwd_wt[e];
    }

  free(pos);
  *rev_rp_out = rrp;
  *rev_ci_out = rci;
  *rev_wt_out = rwt;
}

// ---------------------------------------------------------------------------
// Reference: BF on DAG with early termination
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
// C-side: two distance arrays, two PQs, two graphs
// ---------------------------------------------------------------------------

// Mode 0 = forward (DAG), mode 1 = backward (reversed DAG)
static uint32_t *g_ch_rp[2], *g_ch_ci[2], *g_ch_wt[2];
static uint32_t  g_ch_v;

static uint32_t *g_ch_dist[2];

typedef struct { uint32_t dist, node; } PQEntry;
static PQEntry  *g_ch_pq[2];
static uint32_t  g_ch_pq_size[2];
static uint32_t  g_ch_pq_cap[2];

static void ch_pq_swap(int m, uint32_t i, uint32_t j) {
  PQEntry tmp = g_ch_pq[m][i];
  g_ch_pq[m][i] = g_ch_pq[m][j];
  g_ch_pq[m][j] = tmp;
}

static void ch_pq_siftup(int m, uint32_t i) {
  while (i > 0) {
    uint32_t p = (i - 1) / 2;
    if (g_ch_pq[m][i].dist >= g_ch_pq[m][p].dist) break;
    ch_pq_swap(m, i, p);
    i = p;
  }
}

static void ch_pq_siftdown(int m, uint32_t i) {
  uint32_t n = g_ch_pq_size[m];
  for (;;) {
    uint32_t s = i, l = 2*i+1, r = 2*i+2;
    if (l < n && g_ch_pq[m][l].dist < g_ch_pq[m][s].dist) s = l;
    if (r < n && g_ch_pq[m][r].dist < g_ch_pq[m][s].dist) s = r;
    if (s == i) break;
    ch_pq_swap(m, i, s);
    i = s;
  }
}

static void ch_pq_push(int m, uint32_t dist, uint32_t node) {
  if (g_ch_pq_size[m] >= g_ch_pq_cap[m]) {
    g_ch_pq_cap[m] = g_ch_pq_cap[m] ? g_ch_pq_cap[m] * 2 : 256;
    g_ch_pq[m] = realloc(g_ch_pq[m], g_ch_pq_cap[m] * sizeof(PQEntry));
  }
  g_ch_pq[m][g_ch_pq_size[m]] = (PQEntry){dist, node};
  ch_pq_siftup(m, g_ch_pq_size[m]);
  g_ch_pq_size[m]++;
}

static int ch_pq_pop(int m, PQEntry *out) {
  if (g_ch_pq_size[m] == 0) return 0;
  *out = g_ch_pq[m][0];
  g_ch_pq_size[m]--;
  if (g_ch_pq_size[m] > 0) {
    g_ch_pq[m][0] = g_ch_pq[m][g_ch_pq_size[m]];
    ch_pq_siftdown(m, 0);
  }
  return 1;
}

// ---------------------------------------------------------------------------
// FFI primitives (mode-parameterized)
// ---------------------------------------------------------------------------

// %ch_deg(mode, v) -> degree in upward or reverse graph
static Term prim_ch_deg(Term *args) {
  uint32_t m = term_val(wnf(args[0]));
  uint32_t v = term_val(wnf(args[1]));
  if (v >= g_ch_v || m > 1) return term_new_num(0);
  return term_new_num(g_ch_rp[m][v + 1] - g_ch_rp[m][v]);
}

// %ch_target(mode, v, i) -> i-th neighbor
static Term prim_ch_target(Term *args) {
  uint32_t m = term_val(wnf(args[0]));
  uint32_t v = term_val(wnf(args[1]));
  uint32_t i = term_val(wnf(args[2]));
  uint32_t edge = g_ch_rp[m][v] + i;
  return term_new_num(g_ch_ci[m][edge]);
}

// %ch_weight(mode, v, i) -> i-th edge weight
static Term prim_ch_weight(Term *args) {
  uint32_t m = term_val(wnf(args[0]));
  uint32_t v = term_val(wnf(args[1]));
  uint32_t i = term_val(wnf(args[2]));
  uint32_t edge = g_ch_rp[m][v] + i;
  return term_new_num(g_ch_wt[m][edge]);
}

// %ch_get(mode, v) -> distance
static Term prim_ch_get(Term *args) {
  uint32_t m = term_val(wnf(args[0]));
  uint32_t v = term_val(wnf(args[1]));
  if (v >= g_ch_v || m > 1) return term_new_num(INF);
  return term_new_num(g_ch_dist[m][v]);
}

// %ch_next(mode, dummy) -> pop min from PQ[mode], skip stale, return V if empty
static Term prim_ch_next(Term *args) {
  uint32_t m = term_val(wnf(args[0]));
  wnf(args[1]);
  PQEntry e;
  while (ch_pq_pop(m, &e)) {
    if (e.dist == g_ch_dist[m][e.node])
      return term_new_num(e.node);
  }
  return term_new_num(g_ch_v);
}

// %ch_relax(mode, u, nd) -> min-update dist[mode][u], push to PQ, return 1/0
static Term prim_ch_relax(Term *args) {
  uint32_t m  = term_val(wnf(args[0]));
  uint32_t u  = term_val(wnf(args[1]));
  uint32_t nd = term_val(wnf(args[2]));
  if (u < g_ch_v && m <= 1 && nd < g_ch_dist[m][u]) {
    g_ch_dist[m][u] = nd;
    ch_pq_push(m, nd, u);
    return term_new_num(1);
  }
  return term_new_num(0);
}

// %ch_meet(dummy) -> scan for min(fwd[v] + bwd[v])
static Term prim_ch_meet(Term *args) {
  wnf(args[0]);
  uint32_t best = INF;
  for (uint32_t v = 0; v < g_ch_v; v++) {
    uint32_t sum = g_ch_dist[0][v] + g_ch_dist[1][v];
    if (sum < best) best = sum;
  }
  return term_new_num(best);
}

static void ch_setup(uint32_t V, uint32_t src, uint32_t dest,
                     uint32_t *fwd_rp, uint32_t *fwd_ci, uint32_t *fwd_wt,
                     uint32_t *bwd_rp, uint32_t *bwd_ci, uint32_t *bwd_wt) {
  g_ch_v = V;
  g_ch_rp[0] = fwd_rp; g_ch_ci[0] = fwd_ci; g_ch_wt[0] = fwd_wt;
  g_ch_rp[1] = bwd_rp; g_ch_ci[1] = bwd_ci; g_ch_wt[1] = bwd_wt;

  for (int m = 0; m < 2; m++) {
    g_ch_dist[m] = malloc(V * sizeof(uint32_t));
    for (uint32_t i = 0; i < V; i++) g_ch_dist[m][i] = INF;
    g_ch_pq[m]      = NULL;
    g_ch_pq_size[m]  = 0;
    g_ch_pq_cap[m]   = 0;
  }

  // Seed: forward from src, backward from dest
  g_ch_dist[0][src] = 0;
  ch_pq_push(0, 0, src);
  g_ch_dist[1][dest] = 0;
  ch_pq_push(1, 0, dest);

  prim_register("ch_deg",    6, 2, prim_ch_deg);
  prim_register("ch_target", 9, 3, prim_ch_target);
  prim_register("ch_weight", 9, 3, prim_ch_weight);
  prim_register("ch_get",    6, 2, prim_ch_get);
  prim_register("ch_next",   7, 2, prim_ch_next);
  prim_register("ch_relax",  8, 3, prim_ch_relax);
  prim_register("ch_meet",   7, 1, prim_ch_meet);
}

static void ch_cleanup(void) {
  for (int m = 0; m < 2; m++) {
    free(g_ch_dist[m]);
    free(g_ch_pq[m]);
  }
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

static char *gen_hvm4_source(uint32_t V) {
  size_t cap = 4096;
  char *buf = malloc(cap);
  size_t pos = 0;

  APPEND("@V = %u\n", V);

  // @dj_rel: relax all edges of node v in direction mode
  APPENDS(
    "@dj_rel = " L "mode. " L "v. " L "dv. " L "&i. " L "&deg. "
    "@dj_rgo(i < deg, mode, v, dv, i, deg)\n"

    "@dj_rgo = " L "{"
    "0: " L "mode. " L "v. " L "dv. " L "i. " L "deg. 0; "
    "" L "n. " L "&mode. " L "&v. " L "&dv. " L "&i. " L "deg. "
    "! u = %ch_target(mode, v, i); "
    "! w = %ch_weight(mode, v, i); "
    "! nd = dv + w; "
    "%ch_relax(mode, u, nd) + @dj_rel(mode, v, dv, i + 1, deg)}\n"
  );

  // @dj_loop: Dijkstra loop for one direction
  APPENDS(
    "@dj_loop = " L "{"
    "0: " L "mode. " L "v. 0; "
    "" L "n. " L "&mode. " L "&v. "
    "! dv = %ch_get(mode, v); "
    "! &v2 = %ch_next(mode, @dj_rel(mode, v, dv, 0, %ch_deg(mode, v))); "
    "@dj_loop(v2 < @V, mode, v2)}\n"
  );

  // @dijkstra: run one direction to completion
  APPENDS(
    "@dijkstra = " L "&mode. "
    "! &v = %ch_next(mode, 0); "
    "@dj_loop(v < @V, mode, v)\n"
  );

  // Main: forward + backward + meet
  APPENDS("@main = @dijkstra(0) + @dijkstra(1) + %ch_meet(0)\n");

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V   = argc > 1 ? (uint32_t)atoi(argv[1]) : 1000;
  uint32_t epn = argc > 2 ? (uint32_t)atoi(argv[2]) : 4;

  printf("=== CH benchmark (bidir Dijkstra on DAG): V=%u, ~%u edges/node ===\n", V, epn);

  // Generate DAG
  uint32_t *fwd_rp, *fwd_ci, *fwd_wt, ne;
  gen_dag(V, epn, 42 + V, &fwd_rp, &fwd_ci, &fwd_wt, &ne);
  printf("Graph: V=%u  E=%u (DAG, all edges u<v)\n", V, ne);

  // Build reverse graph
  uint32_t *bwd_rp, *bwd_ci, *bwd_wt;
  build_reverse_csr(V, ne, fwd_rp, fwd_ci, fwd_wt, &bwd_rp, &bwd_ci, &bwd_wt);

  // BF reference on the DAG
  uint32_t *ref = malloc(V * sizeof(uint32_t));
  bf_reference(V, fwd_rp, fwd_ci, fwd_wt, 0, ref);
  printf("Reference: dist[%u] = %u\n", V - 1, ref[V - 1]);

  // HVM4 setup
  hvm4_lib_init();
  char *src = gen_hvm4_source(V);
  printf("HVM4 source: %lu bytes\n", (unsigned long)strlen(src));
  if (V <= 20) {
    printf("--- HVM4 SOURCE ---\n%s--- END SOURCE ---\n", src);
  }

  hvm4_lib_reset();
  ch_setup(V, 0, V - 1, fwd_rp, fwd_ci, fwd_wt, bwd_rp, bwd_ci, bwd_wt);

  // Run
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
    ch_cleanup();
    hvm4_lib_cleanup();
    free(src); free(fwd_rp); free(fwd_ci); free(fwd_wt);
    free(bwd_rp); free(bwd_ci); free(bwd_wt); free(ref);
    return 1;
  }

  printf("HVM4 result: dist[%u] = %u\n", V - 1, result);

  int ok = (count >= 1 && result == ref[V - 1]);
  if (ok) {
    printf("PASS: CH dist = %u matches reference\n", result);
  } else {
    printf("FAIL: CH dist = %u (count=%d), expected %u\n", result, count, ref[V - 1]);
  }

  ch_cleanup();
  free(src); free(fwd_rp); free(fwd_ci); free(fwd_wt);
  free(bwd_rp); free(bwd_ci); free(bwd_wt); free(ref);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
