// Delta-Stepping benchmark driver (C-side dist + priority queue, HVM4 control)
// Graph in C memory (CSR), distances and PQ in C, HVM4 drives relaxation loop.
// Compile: clang -O2 -o bench/delta_step bench/delta_step.c -lpthread
// Usage:   ./bench/delta_step [V] [edges_per_node]

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

  for (uint32_t i = 0; i + 1 < n; i++) {
    uint32_t w = lcg_next(&rng) % 10 + 1;
    edges[ne++] = (RawEdge){i, i + 1, w};
  }

  uint32_t attempts = n * (epn - 1) * 2;
  for (uint32_t a = 0; a < attempts && ne < target_e; a++) {
    uint32_t u = lcg_next(&rng) % n;
    uint32_t v = lcg_next(&rng) % n;
    if (u == v) continue;
    uint32_t w = lcg_next(&rng) % 20 + 1;
    edges[ne++] = (RawEdge){u, v, w};
  }

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
// C-side distance array + min-heap priority queue
// ---------------------------------------------------------------------------

static uint32_t *g_dist;
static uint32_t  g_dist_v;

typedef struct { uint32_t dist, node; } PQEntry;
static PQEntry  *g_pq;
static uint32_t  g_pq_size;
static uint32_t  g_pq_cap;

static void pq_swap(uint32_t i, uint32_t j) {
  PQEntry tmp = g_pq[i]; g_pq[i] = g_pq[j]; g_pq[j] = tmp;
}

static void pq_siftup(uint32_t i) {
  while (i > 0) {
    uint32_t p = (i - 1) / 2;
    if (g_pq[i].dist >= g_pq[p].dist) break;
    pq_swap(i, p);
    i = p;
  }
}

static void pq_siftdown(uint32_t i) {
  for (;;) {
    uint32_t s = i, l = 2*i+1, r = 2*i+2;
    if (l < g_pq_size && g_pq[l].dist < g_pq[s].dist) s = l;
    if (r < g_pq_size && g_pq[r].dist < g_pq[s].dist) s = r;
    if (s == i) break;
    pq_swap(i, s);
    i = s;
  }
}

static void pq_push(uint32_t dist, uint32_t node) {
  if (g_pq_size >= g_pq_cap) {
    g_pq_cap = g_pq_cap ? g_pq_cap * 2 : 256;
    g_pq = realloc(g_pq, g_pq_cap * sizeof(PQEntry));
  }
  g_pq[g_pq_size] = (PQEntry){dist, node};
  pq_siftup(g_pq_size);
  g_pq_size++;
}

static int pq_pop(PQEntry *out) {
  if (g_pq_size == 0) return 0;
  *out = g_pq[0];
  g_pq_size--;
  if (g_pq_size > 0) { g_pq[0] = g_pq[g_pq_size]; pq_siftdown(0); }
  return 1;
}

// ---------------------------------------------------------------------------
// FFI primitives
// ---------------------------------------------------------------------------

// %dist_get(v) -> distance of node v
static Term prim_dist_get(Term *args) {
  Term v = wnf(args[0]);
  uint32_t node = term_val(v);
  return term_new_num(node < g_dist_v ? g_dist[node] : INF);
}

// %ds_next(dummy) -> pop min from PQ (skip stale), return V if empty
static Term prim_ds_next(Term *args) {
  wnf(args[0]);
  PQEntry e;
  while (pq_pop(&e)) {
    if (e.dist == g_dist[e.node])
      return term_new_num(e.node);
  }
  return term_new_num(g_dist_v);  // sentinel: all done
}

// %ds_relax(u, nd) -> if nd < dist[u], update + push to PQ, return 1 else 0
static Term prim_ds_relax(Term *args) {
  Term u_term  = wnf(args[0]);
  Term nd_term = wnf(args[1]);
  uint32_t u  = term_val(u_term);
  uint32_t nd = term_val(nd_term);
  if (u < g_dist_v && nd < g_dist[u]) {
    g_dist[u] = nd;
    pq_push(nd, u);
    return term_new_num(1);
  }
  return term_new_num(0);
}

static void ds_setup(uint32_t *dist, uint32_t V) {
  g_dist   = dist;
  g_dist_v = V;
  g_pq      = NULL;
  g_pq_size = 0;
  g_pq_cap  = 0;

  // Seed source node
  dist[0] = 0;
  pq_push(0, 0);

  prim_register("dist_get",  8, 1, prim_dist_get);
  prim_register("ds_next",   7, 1, prim_ds_next);
  prim_register("ds_relax",  8, 2, prim_ds_relax);
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
  size_t cap = 4096;
  char *buf = malloc(cap);
  size_t pos = 0;

  APPEND("@V = %u\n@DEST = %u\n", V, dest);

  // @ds_rel: relax all edges of node v (dv = dist[v])
  APPENDS(
    "@ds_rel = " L "v. " L "dv. " L "&i. " L "&deg. "
    "@ds_rgo(i < deg, v, dv, i, deg)\n"

    "@ds_rgo = " L "{"
    "0: " L "v. " L "dv. " L "i. " L "deg. 0; "
    "" L "n. " L "&v. " L "&dv. " L "&i. " L "deg. "
    "! u = %graph_target(v, i); "
    "! w = %graph_weight(v, i); "
    "! nd = dv + w; "
    "%ds_relax(u, nd) + @ds_rel(v, dv, i + 1, deg)}\n"
  );

  // @ds_loop: pop nodes from PQ, relax edges, repeat
  // @ds_rel result passed as dummy arg to %ds_next (forces eval order)
  APPENDS(
    "@ds_loop = " L "{"
    "0: " L "v. %dist_get(@DEST); "
    "" L "n. " L "&v. "
    "! dv = %dist_get(v); "
    "! &v2 = %ds_next(@ds_rel(v, dv, 0, %graph_deg(v))); "
    "@ds_loop(v2 < @V, v2)}\n"
  );

  // Main: pop first node and start loop
  APPENDS("@main = ! &v = %ds_next(0); @ds_loop(v < @V, v)\n");

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V   = argc > 1 ? (uint32_t)atoi(argv[1]) : 1000;
  uint32_t epn = argc > 2 ? (uint32_t)atoi(argv[2]) : 4;

  printf("=== Delta-Stepping benchmark (C-side PQ): V=%u, ~%u edges/node ===\n", V, epn);

  uint32_t *rp, *ci, *wt, ne;
  gen_graph(V, epn, 42 + V, &rp, &ci, &wt, &ne);
  printf("Graph: V=%u  E=%u\n", V, ne);

  // Reference
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

  uint32_t *dist = malloc(V * sizeof(uint32_t));
  for (uint32_t i = 0; i < V; i++) dist[i] = INF;
  ds_setup(dist, V);

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
    hvm4_lib_cleanup();
    free(src); free(rp); free(ci); free(wt); free(ref); free(dist); free(g_pq);
    return 1;
  }

  printf("HVM4 result: dist[%u] = %u\n", V - 1, result);

  // Validate
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

  free(src); free(rp); free(ci); free(wt); free(ref); free(dist); free(g_pq);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
