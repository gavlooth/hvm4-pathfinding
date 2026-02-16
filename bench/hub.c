// Hub Labeling with Parallel Dijkstra via HVM4
//
// Phase 1: HVM4 tree reduction dispatches V independent Dijkstra runs in parallel.
//          Each %hub_dijk(i) runs full SSSP from node i, writes dist_matrix row.
//          Per-thread workspaces (indexed by WNF_TID) enable safe parallel execution.
// Phase 2: Sequential PLL (Pruned Landmark Labeling) extracts hub labels from the
//          distance matrix. Processes nodes in degree-descending order with pruning.
// Phase 3: Validate hub queries against distance matrix (all-pairs or sampled).
//
// Unlike CCH (batch size ~1.7), this has V independent work units — ideal for HVM4
// thread parallelism. Expected speedup: near-linear up to thread count.
//
// Compile: cc -O2 -o bench/hub bench/hub.c -lpthread -lm
// Usage:   ./bench/hub [V] [edges_per_node]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <sys/resource.h>
#include "../c3lib/csrc/hvm4_bridge.c"

#define HUB_INF 999999u
#define MAX_HUB_THREADS 64

// ---------------------------------------------------------------------------
// LCG random
// ---------------------------------------------------------------------------

typedef struct { uint32_t s; } LCG;
static uint32_t lcg_next(LCG *r) { r->s = r->s * 1103515245u + 12345u; return r->s >> 16; }

// ---------------------------------------------------------------------------
// CSR Graph (undirected, both directions stored)
// ---------------------------------------------------------------------------

typedef struct {
  uint32_t *row_ptr;  // V+1
  uint32_t *col_idx;  // E
  uint32_t *weight;   // E
  uint32_t V, E;
} CSRGraph;

static CSRGraph g_csr;

static void gen_graph(uint32_t V, uint32_t epn, uint32_t seed) {
  LCG rng = { seed };
  uint32_t max_directed = V * epn;

  // Temporary edge list
  typedef struct { uint32_t u, v, w; } Edge;
  Edge *edges = malloc(max_directed * sizeof(Edge));
  uint32_t ne = 0;

  for (uint32_t u = 0; u < V; u++) {
    for (uint32_t j = 0; j < epn; j++) {
      uint32_t v = lcg_next(&rng) % V;
      if (v == u) v = (v + 1) % V;
      uint32_t w = 1 + lcg_next(&rng) % 100;
      edges[ne++] = (Edge){u, v, w};
    }
  }

  // Count edges per node (both directions for undirected)
  uint32_t *counts = calloc(V, sizeof(uint32_t));
  for (uint32_t i = 0; i < ne; i++) {
    counts[edges[i].u]++;
    counts[edges[i].v]++;
  }

  g_csr.V = V;
  g_csr.row_ptr = malloc((V + 1) * sizeof(uint32_t));
  g_csr.row_ptr[0] = 0;
  for (uint32_t i = 0; i < V; i++)
    g_csr.row_ptr[i + 1] = g_csr.row_ptr[i] + counts[i];
  g_csr.E = g_csr.row_ptr[V];
  g_csr.col_idx = malloc(g_csr.E * sizeof(uint32_t));
  g_csr.weight  = malloc(g_csr.E * sizeof(uint32_t));

  memset(counts, 0, V * sizeof(uint32_t));
  for (uint32_t i = 0; i < ne; i++) {
    uint32_t u = edges[i].u, v = edges[i].v, w = edges[i].w;
    uint32_t pu = g_csr.row_ptr[u] + counts[u]++;
    g_csr.col_idx[pu] = v; g_csr.weight[pu] = w;
    uint32_t pv = g_csr.row_ptr[v] + counts[v]++;
    g_csr.col_idx[pv] = u; g_csr.weight[pv] = w;
  }

  free(edges);
  free(counts);
}

// ---------------------------------------------------------------------------
// Per-thread Dijkstra workspace
// ---------------------------------------------------------------------------

typedef struct {
  uint32_t *dist;
  uint8_t  *visited;
  uint32_t *h_node;   // binary min-heap: node ids
  uint32_t *h_dist;   // binary min-heap: distances
  uint32_t  h_size;
} DijkWS;

static DijkWS g_ws[MAX_HUB_THREADS];
static uint32_t *g_dist_matrix;  // V*V, row-major
static uint32_t g_V;

static void ws_init(uint32_t V) {
  g_V = V;
  g_dist_matrix = malloc((size_t)V * V * sizeof(uint32_t));
  uint32_t heap_cap = g_csr.E + V;  // lazy heap can grow up to E entries
  for (int t = 0; t < MAX_HUB_THREADS; t++) {
    g_ws[t].dist    = malloc(V * sizeof(uint32_t));
    g_ws[t].visited = malloc(V);
    g_ws[t].h_node  = malloc(heap_cap * sizeof(uint32_t));
    g_ws[t].h_dist  = malloc(heap_cap * sizeof(uint32_t));
  }
}

static void ws_destroy(void) {
  free(g_dist_matrix);
  for (int t = 0; t < MAX_HUB_THREADS; t++) {
    free(g_ws[t].dist); free(g_ws[t].visited);
    free(g_ws[t].h_node); free(g_ws[t].h_dist);
  }
}

// Binary min-heap

static void h_swap(DijkWS *ws, uint32_t a, uint32_t b) {
  uint32_t tn = ws->h_node[a]; ws->h_node[a] = ws->h_node[b]; ws->h_node[b] = tn;
  uint32_t td = ws->h_dist[a]; ws->h_dist[a] = ws->h_dist[b]; ws->h_dist[b] = td;
}

static void h_push(DijkWS *ws, uint32_t node, uint32_t dist) {
  uint32_t i = ws->h_size++;
  ws->h_node[i] = node;
  ws->h_dist[i] = dist;
  while (i > 0) {
    uint32_t p = (i - 1) / 2;
    if (ws->h_dist[p] <= ws->h_dist[i]) break;
    h_swap(ws, i, p);
    i = p;
  }
}

static uint32_t h_pop(DijkWS *ws) {
  uint32_t node = ws->h_node[0];
  ws->h_size--;
  if (ws->h_size > 0) {
    ws->h_node[0] = ws->h_node[ws->h_size];
    ws->h_dist[0] = ws->h_dist[ws->h_size];
    uint32_t i = 0;
    while (1) {
      uint32_t l = 2*i+1, r = 2*i+2, m = i;
      if (l < ws->h_size && ws->h_dist[l] < ws->h_dist[m]) m = l;
      if (r < ws->h_size && ws->h_dist[r] < ws->h_dist[m]) m = r;
      if (m == i) break;
      h_swap(ws, i, m);
      i = m;
    }
  }
  return node;
}

// Run Dijkstra from source using thread tid's workspace
static void run_dijkstra(uint32_t src, uint32_t tid) {
  DijkWS *ws = &g_ws[tid];
  uint32_t V = g_V;

  for (uint32_t i = 0; i < V; i++) { ws->dist[i] = HUB_INF; ws->visited[i] = 0; }
  ws->dist[src] = 0;
  ws->h_size = 0;
  h_push(ws, src, 0);

  while (ws->h_size > 0) {
    uint32_t u = h_pop(ws);
    if (ws->visited[u]) continue;
    ws->visited[u] = 1;
    uint32_t du = ws->dist[u];
    if (du >= HUB_INF) break;

    for (uint32_t i = g_csr.row_ptr[u]; i < g_csr.row_ptr[u+1]; i++) {
      uint32_t v = g_csr.col_idx[i];
      uint32_t nd = du + g_csr.weight[i];
      if (nd < ws->dist[v]) {
        ws->dist[v] = nd;
        h_push(ws, v, nd);
      }
    }
  }

  memcpy(&g_dist_matrix[(size_t)src * V], ws->dist, V * sizeof(uint32_t));
}

// ---------------------------------------------------------------------------
// FFI: %hub_dijk(source) → runs Dijkstra, returns source id
// ---------------------------------------------------------------------------

static Term prim_hub_dijk(Term *args) {
  uint32_t src = term_val(wnf(args[0]));
  if (src < g_V) run_dijkstra(src, WNF_TID);
  return term_new_num(src);
}

// ---------------------------------------------------------------------------
// Hub label structures
// ---------------------------------------------------------------------------

typedef struct { uint32_t hub; uint32_t dist; } HubLabel;

typedef struct {
  HubLabel *data;
  uint32_t count;
  uint32_t cap;
} LabelSet;

static LabelSet *g_fwd;  // V sets
static LabelSet *g_bwd;  // V sets

static void label_init(uint32_t V) {
  g_fwd = calloc(V, sizeof(LabelSet));
  g_bwd = calloc(V, sizeof(LabelSet));
  for (uint32_t i = 0; i < V; i++) {
    g_fwd[i].cap = 4; g_fwd[i].data = malloc(4 * sizeof(HubLabel));
    g_bwd[i].cap = 4; g_bwd[i].data = malloc(4 * sizeof(HubLabel));
  }
}

static void label_add(LabelSet *ls, uint32_t hub, uint32_t dist) {
  if (ls->count >= ls->cap) {
    ls->cap *= 2;
    ls->data = realloc(ls->data, ls->cap * sizeof(HubLabel));
  }
  ls->data[ls->count++] = (HubLabel){hub, dist};
}

static void label_destroy(uint32_t V) {
  for (uint32_t i = 0; i < V; i++) { free(g_fwd[i].data); free(g_bwd[i].data); }
  free(g_fwd); free(g_bwd);
}

// Unsorted hub query (used during PLL building, labels not yet sorted)
static uint32_t hub_query_build(uint32_t s, uint32_t t) {
  if (s == t) return 0;
  uint32_t best = HUB_INF;
  for (uint32_t i = 0; i < g_fwd[s].count; i++) {
    uint32_t h = g_fwd[s].data[i].hub;
    uint32_t d1 = g_fwd[s].data[i].dist;
    for (uint32_t j = 0; j < g_bwd[t].count; j++) {
      if (g_bwd[t].data[j].hub == h) {
        uint32_t d = d1 + g_bwd[t].data[j].dist;
        if (d < best) best = d;
      }
    }
  }
  return best;
}

// Sorted hub query (merge-join, used after PLL building)
static int cmp_label(const void *a, const void *b) {
  return (int)((HubLabel*)a)->hub - (int)((HubLabel*)b)->hub;
}

static uint32_t hub_query(uint32_t s, uint32_t t) {
  if (s == t) return 0;
  LabelSet *fs = &g_fwd[s], *bt = &g_bwd[t];
  uint32_t fi = 0, bi = 0, best = HUB_INF;
  while (fi < fs->count && bi < bt->count) {
    uint32_t fh = fs->data[fi].hub, bh = bt->data[bi].hub;
    if (fh == bh) {
      uint32_t d = fs->data[fi].dist + bt->data[bi].dist;
      if (d < best) best = d;
      fi++; bi++;
    } else if (fh < bh) fi++;
    else bi++;
  }
  return best;
}

// PLL: extract hub labels from distance matrix
static void extract_labels_pll(uint32_t V) {
  // Node order: degree descending
  uint32_t *order = malloc(V * sizeof(uint32_t));
  for (uint32_t i = 0; i < V; i++) order[i] = i;
  for (uint32_t i = 0; i < V; i++) {
    uint32_t best = i;
    uint32_t best_deg = g_csr.row_ptr[order[i]+1] - g_csr.row_ptr[order[i]];
    for (uint32_t j = i + 1; j < V; j++) {
      uint32_t deg = g_csr.row_ptr[order[j]+1] - g_csr.row_ptr[order[j]];
      if (deg > best_deg) { best = j; best_deg = deg; }
    }
    if (best != i) { uint32_t t = order[i]; order[i] = order[best]; order[best] = t; }
  }

  for (uint32_t idx = 0; idx < V; idx++) {
    uint32_t v = order[idx];

    // Self-labels: "from v, reach hub v at cost 0" and "hub v reaches v at cost 0"
    label_add(&g_fwd[v], v, 0);
    label_add(&g_bwd[v], v, 0);

    // Forward Dijkstra from v: d(v→u) → add backward labels for each u
    for (uint32_t u = 0; u < V; u++) {
      if (v == u) continue;
      uint32_t d = g_dist_matrix[(size_t)v * V + u];
      if (d >= HUB_INF) continue;
      if (hub_query_build(v, u) <= d) continue;
      label_add(&g_bwd[u], v, d);
    }

    // Backward: d(u→v) → add forward labels (undirected: same as d(v→u))
    for (uint32_t u = 0; u < V; u++) {
      if (v == u) continue;
      uint32_t d = g_dist_matrix[(size_t)u * V + v];
      if (d >= HUB_INF) continue;
      if (hub_query_build(u, v) <= d) continue;
      label_add(&g_fwd[u], v, d);
    }
  }

  // Sort for merge-join queries
  for (uint32_t i = 0; i < V; i++) {
    qsort(g_fwd[i].data, g_fwd[i].count, sizeof(HubLabel), cmp_label);
    qsort(g_bwd[i].data, g_bwd[i].count, sizeof(HubLabel), cmp_label);
  }

  free(order);
}

// ---------------------------------------------------------------------------
// HVM4 source generation
// ---------------------------------------------------------------------------

static char *gen_hvm4_source(uint32_t V, uint32_t *out_depth) {
  size_t cap = 2048;
  char *buf = malloc(cap);
  size_t pos = 0;

  uint32_t depth = 0;
  { uint32_t n = V; while (n > 1) { n = (n + 1) / 2; depth++; } }
  *out_depth = depth;

  #define L "\xce\xbb"
  #define APPEND(fmt, ...) pos += snprintf(buf + pos, cap - pos, fmt, ##__VA_ARGS__)
  #define APPENDS(s) do { size_t _l = strlen(s); \
    if (pos + _l < cap) { memcpy(buf + pos, s, _l); pos += _l; } } while(0)

  APPEND("@DEPTH = %u\n\n", depth);
  APPEND("@V = %u\n\n", V);

  APPENDS(
    "// Parallel Dijkstra dispatch: pair tree for work-stealing\n"
    "// Pairs (not +) so normalize distributes children to thread queues\n"
    "@par = " L "{0: " L "&lo. " L "hi. @p1(lo < hi, lo);\n"
    "  " L "d. " L "&lo. " L "&hi.\n"
    "    ! &d1 = d - 1;\n"
    "    ! &mid = (lo + hi) / 2;\n"
    "    #P{@par(d1, lo, mid), @par(d1, mid, hi)}}\n\n"

    "@p1 = " L "{0: " L "lo. 0; " L "n. " L "lo. %hub_dijk(lo)}\n\n"

    "@main = @par(@DEPTH, 0, @V)\n"
  );

  #undef L
  #undef APPEND
  #undef APPENDS

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Reference: sequential Dijkstra for one source (for spot-checks)
// ---------------------------------------------------------------------------

static void dijkstra_ref(uint32_t src, uint32_t *dist) {
  uint32_t V = g_V;
  bool *visited = calloc(V, sizeof(bool));
  for (uint32_t i = 0; i < V; i++) dist[i] = HUB_INF;
  dist[src] = 0;

  for (uint32_t iter = 0; iter < V; iter++) {
    uint32_t u = V, du = HUB_INF;
    for (uint32_t i = 0; i < V; i++)
      if (!visited[i] && dist[i] < du) { du = dist[i]; u = i; }
    if (u == V) break;
    visited[u] = true;
    for (uint32_t i = g_csr.row_ptr[u]; i < g_csr.row_ptr[u+1]; i++) {
      uint32_t nd = du + g_csr.weight[i];
      if (nd < dist[g_csr.col_idx[i]]) dist[g_csr.col_idx[i]] = nd;
    }
  }
  free(visited);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V   = argc > 1 ? (uint32_t)atoi(argv[1]) : 200;
  uint32_t epn = argc > 2 ? (uint32_t)atoi(argv[2]) : 4;
  bool phase1_only = argc > 3 && argv[3][0] == '1';  // skip PLL for benchmarking

  printf("=== Hub Labeling (Parallel Dijkstra): V=%u, ~%u edges/node ===\n", V, epn);

  gen_graph(V, epn, 42 + V);
  printf("Graph: V=%u  E=%u (undirected)\n", V, g_csr.E);

  ws_init(V);
  label_init(V);

  // HVM4 setup
  hvm4_lib_init();
  hvm4_lib_reset();
  prim_register("hub_dijk", 8, 1, prim_hub_dijk);

  uint32_t depth;
  char *src = gen_hvm4_source(V, &depth);
  printf("HVM4 source: %zu bytes, tree depth: %u\n", strlen(src), depth);
  if (V <= 30) {
    printf("--- HVM4 SOURCE ---\n%s--- END SOURCE ---\n", src);
  }

  // Phase 1: Parallel Dijkstra via HVM4
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  uint32_t hvm_result = 0;
  int count = hvm4_run(src, 0, &hvm_result, 1);
  clock_gettime(CLOCK_MONOTONIC, &t1);
  double phase1 = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
  printf("Phase 1 (parallel SSSP × %u): %.3f s\n", V, phase1);
  if (count < 0) { printf("FAIL: hvm4_run returned %d\n", count); return 1; }

  // Verify distance matrix against reference Dijkstra (spot check node 0)
  uint32_t *ref_dist = malloc(V * sizeof(uint32_t));
  dijkstra_ref(0, ref_dist);
  bool matrix_ok = true;
  for (uint32_t i = 0; i < V; i++) {
    if (g_dist_matrix[i] != ref_dist[i]) {
      printf("  MATRIX MISMATCH: d(0,%u) hvm4=%u ref=%u\n", i, g_dist_matrix[i], ref_dist[i]);
      matrix_ok = false;
    }
  }
  free(ref_dist);
  if (!matrix_ok) { printf("FAIL: distance matrix incorrect\n"); return 1; }
  printf("Distance matrix spot-check: OK (row 0 matches reference)\n");

  uint32_t fail = 0;
  if (!phase1_only) {
    // Phase 2: PLL label extraction
    clock_gettime(CLOCK_MONOTONIC, &t0);
    extract_labels_pll(V);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double phase2 = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
    printf("Phase 2 (PLL labels): %.3f s\n", phase2);

    uint32_t total_labels = 0;
    for (uint32_t i = 0; i < V; i++)
      total_labels += g_fwd[i].count + g_bwd[i].count;
    printf("Labels: %u total (avg %.1f per node fwd+bwd)\n", total_labels, (double)total_labels / V);

    // Phase 3: Validate hub queries
    uint32_t pass = 0;
    uint32_t check_pairs = (V <= 500) ? V * V : 100000;
    LCG rng = { 12345 };

    for (uint32_t k = 0; k < check_pairs; k++) {
      uint32_t s, t;
      if (V <= 500) { s = k / V; t = k % V; }
      else { s = lcg_next(&rng) % V; t = lcg_next(&rng) % V; }

      uint32_t hd = hub_query(s, t);
      uint32_t rd = (s == t) ? 0 : g_dist_matrix[(size_t)s * V + t];
      if (hd == rd) pass++;
      else {
        if (fail < 5) printf("  MISMATCH: d(%u,%u) hub=%u ref=%u\n", s, t, hd, rd);
        fail++;
      }
    }

    printf("Validation: %u/%u queries correct\n", pass, pass + fail);
    if (fail == 0) printf("PASS\n");
    else printf("FAIL: %u mismatches\n", fail);
  } else {
    printf("Phase 1 only (skipped PLL/validation)\n");
    printf("PASS\n");
  }

  // Peak RSS
  FILE *f = fopen("/proc/self/status", "r");
  if (f) {
    char line[256];
    while (fgets(line, sizeof(line), f))
      if (strncmp(line, "VmHWM:", 6) == 0) { printf("Peak RSS: %s", line + 6); break; }
    fclose(f);
  }

  // Cleanup
  hvm4_lib_cleanup();
  label_destroy(V);
  ws_destroy();
  free(g_csr.row_ptr); free(g_csr.col_idx); free(g_csr.weight);
  free(src);
  return fail > 0 ? 1 : 0;
}
