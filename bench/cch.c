// CCH Batch-Parallel Benchmark: HVM4 tree reduction for parallel contraction
//
// C holds graph + CCH state. HVM4 drives batch-parallel computation:
//   1. C finds independent set (local priority minima)
//   2. HVM4 parallel tree: compute shortcuts per batch node
//   3. C applies shortcuts, marks contracted, finds affected neighbors
//   4. HVM4 parallel tree: update priorities for affected nodes
//
// Per-thread witness workspaces enable safe parallel execution.
//
// Compile: clang -O2 -o bench/cch bench/cch.c -lpthread -lm
// Usage:   ./bench/cch [V] [edges_per_node]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <sys/resource.h>
#include "../c3lib/csrc/hvm4_bridge.c"

#define CCH_INF 999999u

// ---------------------------------------------------------------------------
// LCG random number generator
// ---------------------------------------------------------------------------

typedef struct { uint32_t s; } LCG;

static uint32_t lcg_next(LCG *r) {
  r->s = (r->s * 1103515245u + 12345u) & 0x7fffffffu;
  return r->s;
}

// ---------------------------------------------------------------------------
// Dynamic adjacency list graph
// ---------------------------------------------------------------------------

typedef struct {
  uint32_t *to;
  uint32_t *wt;
  uint32_t count;
  uint32_t cap;
} AdjList;

static void adj_init(AdjList *a) {
  a->to = NULL; a->wt = NULL; a->count = 0; a->cap = 0;
}

static void adj_add(AdjList *a, uint32_t to, uint32_t wt) {
  if (a->count >= a->cap) {
    a->cap = a->cap ? a->cap * 2 : 4;
    a->to = realloc(a->to, a->cap * sizeof(uint32_t));
    a->wt = realloc(a->wt, a->cap * sizeof(uint32_t));
  }
  a->to[a->count] = to;
  a->wt[a->count] = wt;
  a->count++;
}

static void adj_free(AdjList *a) {
  free(a->to); free(a->wt);
}

typedef struct {
  AdjList *fwd;
  AdjList *rev;
  uint32_t n;
} CchGraph;

static void cgraph_init(CchGraph *g, uint32_t n) {
  g->n = n;
  g->fwd = malloc(n * sizeof(AdjList));
  g->rev = malloc(n * sizeof(AdjList));
  for (uint32_t i = 0; i < n; i++) {
    adj_init(&g->fwd[i]);
    adj_init(&g->rev[i]);
  }
}

static void cgraph_add(CchGraph *g, uint32_t from, uint32_t to, uint32_t wt) {
  adj_add(&g->fwd[from], to, wt);
  adj_add(&g->rev[to], from, wt);
}

static void cgraph_free(CchGraph *g) {
  for (uint32_t i = 0; i < g->n; i++) {
    adj_free(&g->fwd[i]);
    adj_free(&g->rev[i]);
  }
  free(g->fwd); free(g->rev);
}

// ---------------------------------------------------------------------------
// Random graph generator (undirected, connected)
// ---------------------------------------------------------------------------

static void gen_graph(uint32_t n, uint32_t epn, uint32_t seed, CchGraph *g) {
  LCG rng = { .s = seed };
  cgraph_init(g, n);
  // Chain for connectivity
  for (uint32_t i = 0; i + 1 < n; i++) {
    uint32_t w = lcg_next(&rng) % 10 + 1;
    cgraph_add(g, i, i + 1, w);
    cgraph_add(g, i + 1, i, w);
  }
  // Random edges
  uint32_t ne = n - 1;
  uint32_t target = n * epn;
  uint32_t attempts = n * (epn - 1) * 2;
  for (uint32_t a = 0; a < attempts && ne < target; a++) {
    uint32_t u = lcg_next(&rng) % n;
    uint32_t v = lcg_next(&rng) % n;
    if (u == v) continue;
    uint32_t w = lcg_next(&rng) % 20 + 1;
    cgraph_add(g, u, v, w);
    cgraph_add(g, v, u, w);
    ne++;
  }
}

// ---------------------------------------------------------------------------
// CCH global state
// ---------------------------------------------------------------------------

static CchGraph g_graph;
static uint32_t g_n;
static bool    *g_contracted;
static uint32_t *g_level;
static int32_t *g_priority;
static uint32_t *g_rank;
static uint32_t g_rank_pos;

// Batch state
static uint32_t *g_batch;
static uint32_t g_batch_size;
static bool     *g_in_batch;

// Affected neighbors (for priority update)
static uint32_t *g_affected;
static uint32_t g_affected_count;
static bool     *g_is_affected;

// Per-batch shortcut buffer
typedef struct { uint32_t from, to, wt; } Shortcut;

typedef struct {
  Shortcut *data;
  uint32_t count;
  uint32_t cap;
} ShortcutBuf;

static ShortcutBuf *g_sc_bufs;

// Witness workspace: one per HVM4 thread (indexed by WNF_TID)
typedef struct {
  uint32_t *dist;
  uint32_t *gen;
  uint32_t cur_gen;
} WitnessWS;

static WitnessWS g_ws[64]; // MAX_THREADS = 64

// ---------------------------------------------------------------------------
// Witness search (thread-safe via per-thread workspace)
// ---------------------------------------------------------------------------

static bool witness_search(uint32_t tid, uint32_t v, uint32_t w,
                           uint32_t limit, uint32_t skip) {
  WitnessWS *ws = &g_ws[tid];
  ws->cur_gen++;
  ws->dist[v] = 0;
  ws->gen[v] = ws->cur_gen;

  uint32_t queue[256];
  uint32_t qh = 0, qt = 0;
  queue[qt++] = v;

  for (uint32_t settled = 0; settled < 20 && qh < qt; settled++) {
    // Linear scan for min (queue is small, bounded by 20 settles)
    uint32_t best_qi = qh;
    uint32_t best_d = (ws->gen[queue[qh]] == ws->cur_gen)
                      ? ws->dist[queue[qh]] : CCH_INF;
    for (uint32_t qi = qh + 1; qi < qt && qi < qh + 256; qi++) {
      uint32_t nd = queue[qi & 255];
      uint32_t d = (ws->gen[nd] == ws->cur_gen) ? ws->dist[nd] : CCH_INF;
      if (d < best_d) { best_d = d; best_qi = qi; }
    }
    if (best_qi != qh) {
      uint32_t tmp = queue[qh & 255];
      queue[qh & 255] = queue[best_qi & 255];
      queue[best_qi & 255] = tmp;
    }

    uint32_t u = queue[qh++ & 255];
    uint32_t du = (ws->gen[u] == ws->cur_gen) ? ws->dist[u] : CCH_INF;
    if (du > limit) continue;
    if (u == w) return true;
    if (g_contracted[u] && u != v) continue;
    if (u == skip) continue;
    if (g_in_batch[u] && u != v) continue;

    AdjList *adj = &g_graph.fwd[u];
    for (uint32_t i = 0; i < adj->count; i++) {
      uint32_t nb = adj->to[i];
      if (g_contracted[nb] || nb == skip) continue;
      if (g_in_batch[nb]) continue;
      uint32_t nd = du + adj->wt[i];
      if (nd > limit) continue;
      uint32_t cur = (ws->gen[nb] == ws->cur_gen) ? ws->dist[nb] : CCH_INF;
      if (nd < cur) {
        ws->dist[nb] = nd;
        ws->gen[nb] = ws->cur_gen;
        if ((qt - qh) < 256)
          queue[qt++ & 255] = nb;
      }
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Priority computation (thread-safe: uses per-thread witness workspace)
// ---------------------------------------------------------------------------

static int32_t compute_priority_mt(uint32_t u, uint32_t tid) {
  uint32_t shortcuts = 0, edges_removed = 0;

  AdjList *fwd = &g_graph.fwd[u];
  AdjList *rev = &g_graph.rev[u];
  for (uint32_t i = 0; i < fwd->count; i++)
    if (!g_contracted[fwd->to[i]]) edges_removed++;
  for (uint32_t i = 0; i < rev->count; i++)
    if (!g_contracted[rev->to[i]]) edges_removed++;

  for (uint32_t i = 0; i < rev->count; i++) {
    uint32_t v = rev->to[i];
    if (g_contracted[v]) continue;
    uint32_t w_vu = rev->wt[i];
    for (uint32_t j = 0; j < fwd->count; j++) {
      uint32_t w = fwd->to[j];
      if (g_contracted[w] || w == v) continue;
      uint32_t sc_dist = w_vu + fwd->wt[j];
      if (!witness_search(tid, v, w, sc_dist, u))
        shortcuts++;
    }
  }
  return (int32_t)shortcuts - (int32_t)edges_removed + (int32_t)(2 * g_level[u]);
}

// ---------------------------------------------------------------------------
// Independent set finding (sequential, O(V) scan)
// ---------------------------------------------------------------------------

static uint32_t find_independent_set(void) {
  g_batch_size = 0;
  for (uint32_t i = 0; i < g_n; i++) g_in_batch[i] = false;

  for (uint32_t i = 0; i < g_n; i++) {
    if (g_contracted[i]) continue;
    bool is_min = true;

    // Check forward neighbors
    AdjList *fwd = &g_graph.fwd[i];
    for (uint32_t j = 0; j < fwd->count && is_min; j++) {
      uint32_t nb = fwd->to[j];
      if (!g_contracted[nb] && nb != i)
        if (g_priority[nb] < g_priority[i] ||
            (g_priority[nb] == g_priority[i] && nb < i))
          is_min = false;
    }
    if (!is_min) continue;

    // Check reverse neighbors
    AdjList *rev = &g_graph.rev[i];
    for (uint32_t j = 0; j < rev->count && is_min; j++) {
      uint32_t nb = rev->to[j];
      if (!g_contracted[nb] && nb != i)
        if (g_priority[nb] < g_priority[i] ||
            (g_priority[nb] == g_priority[i] && nb < i))
          is_min = false;
    }
    if (!is_min) continue;

    // Verify no neighbor already in batch (independence)
    bool conflict = false;
    for (uint32_t j = 0; j < fwd->count && !conflict; j++)
      if (g_in_batch[fwd->to[j]]) conflict = true;
    for (uint32_t j = 0; j < rev->count && !conflict; j++)
      if (g_in_batch[rev->to[j]]) conflict = true;
    if (conflict) continue;

    g_batch[g_batch_size++] = i;
    g_in_batch[i] = true;
  }

  // Fallback: force-contract lowest priority node
  if (g_batch_size == 0) {
    uint32_t best = g_n;
    int32_t best_p = 0x7FFFFFFF;
    for (uint32_t i = 0; i < g_n; i++)
      if (!g_contracted[i] && g_priority[i] < best_p) {
        best_p = g_priority[i]; best = i;
      }
    if (best < g_n) {
      g_batch[0] = best;
      g_batch_size = 1;
      g_in_batch[best] = true;
    }
  }
  return g_batch_size;
}

// ---------------------------------------------------------------------------
// Compute shortcuts for one batch node (thread-safe)
// ---------------------------------------------------------------------------

static uint32_t compute_shortcuts_for(uint32_t batch_idx, uint32_t tid) {
  uint32_t u = g_batch[batch_idx];
  ShortcutBuf *buf = &g_sc_bufs[batch_idx];
  buf->count = 0;

  AdjList *rev = &g_graph.rev[u];
  AdjList *fwd = &g_graph.fwd[u];

  for (uint32_t i = 0; i < rev->count; i++) {
    uint32_t v = rev->to[i];
    if (g_contracted[v] || g_in_batch[v]) continue;
    uint32_t w_vu = rev->wt[i];

    for (uint32_t j = 0; j < fwd->count; j++) {
      uint32_t w = fwd->to[j];
      if (g_contracted[w] || g_in_batch[w] || w == v) continue;
      uint32_t sc_dist = w_vu + fwd->wt[j];

      if (!witness_search(tid, v, w, sc_dist, u)) {
        if (buf->count >= buf->cap) {
          buf->cap = buf->cap ? buf->cap * 2 : 16;
          buf->data = realloc(buf->data, buf->cap * sizeof(Shortcut));
        }
        buf->data[buf->count++] = (Shortcut){ v, w, sc_dist };
      }
    }
  }
  return buf->count;
}

// ---------------------------------------------------------------------------
// Apply round: insert shortcuts, mark contracted, find affected
// ---------------------------------------------------------------------------

static uint32_t apply_round(void) {
  // Apply all buffered shortcuts to the graph
  for (uint32_t bi = 0; bi < g_batch_size; bi++) {
    ShortcutBuf *buf = &g_sc_bufs[bi];
    for (uint32_t i = 0; i < buf->count; i++)
      cgraph_add(&g_graph, buf->data[i].from, buf->data[i].to, buf->data[i].wt);
  }

  // Assign ranks + mark contracted + update levels
  for (uint32_t bi = 0; bi < g_batch_size; bi++) {
    uint32_t u = g_batch[bi];
    g_rank[u] = g_rank_pos++;
    g_contracted[u] = true;
    AdjList *fwd = &g_graph.fwd[u];
    for (uint32_t i = 0; i < fwd->count; i++) {
      uint32_t w = fwd->to[i];
      if (!g_contracted[w]) {
        uint32_t nl = g_level[u] + 1;
        if (nl > g_level[w]) g_level[w] = nl;
      }
    }
  }

  // Collect affected neighbors (need priority update)
  g_affected_count = 0;
  for (uint32_t i = 0; i < g_n; i++) g_is_affected[i] = false;

  for (uint32_t bi = 0; bi < g_batch_size; bi++) {
    uint32_t u = g_batch[bi];
    AdjList *fwd = &g_graph.fwd[u];
    AdjList *rev = &g_graph.rev[u];
    for (uint32_t i = 0; i < fwd->count; i++) {
      uint32_t w = fwd->to[i];
      if (!g_contracted[w] && !g_is_affected[w]) {
        g_is_affected[w] = true;
        g_affected[g_affected_count++] = w;
      }
    }
    for (uint32_t i = 0; i < rev->count; i++) {
      uint32_t w = rev->to[i];
      if (!g_contracted[w] && !g_is_affected[w]) {
        g_is_affected[w] = true;
        g_affected[g_affected_count++] = w;
      }
    }
  }

  // Clear batch markers
  for (uint32_t bi = 0; bi < g_batch_size; bi++)
    g_in_batch[g_batch[bi]] = false;

  return g_affected_count;
}

// ---------------------------------------------------------------------------
// FFI Primitives
// ---------------------------------------------------------------------------

static uint32_t g_round_num = 0;

// %cch_round(dummy) → batch_size (C finds independent set)
static Term prim_cch_round(Term *args) {
  wnf(args[0]);
  uint32_t bs = find_independent_set();
  if (bs > 0) g_round_num++;
  return term_new_num(bs);
}

// %cch_sc(batch_idx) → shortcut_count (parallel: per-thread workspace)
static Term prim_cch_sc(Term *args) {
  uint32_t idx = term_val(wnf(args[0]));
  if (idx >= g_batch_size) return term_new_num(0);
  return term_new_num(compute_shortcuts_for(idx, WNF_TID));
}

// %cch_apply(dummy) → affected_count (C applies shortcuts + marks contracted)
static Term prim_cch_apply(Term *args) {
  wnf(args[0]);
  return term_new_num(apply_round());
}

// %cch_up(affected_idx) → new_priority (parallel: per-thread workspace)
static Term prim_cch_up(Term *args) {
  uint32_t idx = term_val(wnf(args[0]));
  if (idx >= g_affected_count) return term_new_num(0);
  uint32_t node = g_affected[idx];
  g_priority[node] = compute_priority_mt(node, WNF_TID);
  return term_new_num((uint32_t)(g_priority[node] + 100000)); // offset to keep positive
}

// ---------------------------------------------------------------------------
// CCH setup + cleanup
// ---------------------------------------------------------------------------

static void cch_setup(uint32_t n) {
  g_n = n;
  g_contracted  = calloc(n, sizeof(bool));
  g_level       = calloc(n, sizeof(uint32_t));
  g_priority    = malloc(n * sizeof(int32_t));
  g_rank        = malloc(n * sizeof(uint32_t));
  g_rank_pos    = 0;
  g_batch       = malloc(n * sizeof(uint32_t));
  g_in_batch    = calloc(n, sizeof(bool));
  g_affected    = malloc(n * sizeof(uint32_t));
  g_is_affected = calloc(n, sizeof(bool));
  g_sc_bufs     = calloc(n, sizeof(ShortcutBuf));

  // Per-thread witness workspaces
  for (int t = 0; t < 64; t++) {
    g_ws[t].dist    = malloc(n * sizeof(uint32_t));
    g_ws[t].gen     = calloc(n, sizeof(uint32_t));
    g_ws[t].cur_gen = 0;
  }

  // Compute initial priorities (single-threaded, tid=0)
  for (uint32_t i = 0; i < n; i++)
    g_priority[i] = compute_priority_mt(i, 0);

  // Register FFI primitives
  prim_register("cch_round", 9,  1, prim_cch_round);
  prim_register("cch_sc",    6,  1, prim_cch_sc);
  prim_register("cch_apply", 9,  1, prim_cch_apply);
  prim_register("cch_up",    6,  1, prim_cch_up);
}

static void cch_cleanup(void) {
  free(g_contracted); free(g_level); free(g_priority);
  free(g_rank); free(g_batch); free(g_in_batch);
  free(g_affected); free(g_is_affected);
  for (uint32_t i = 0; i < g_n; i++)
    free(g_sc_bufs[i].data);
  free(g_sc_bufs);
  for (int t = 0; t < 64; t++) {
    free(g_ws[t].dist); free(g_ws[t].gen);
  }
}

// ---------------------------------------------------------------------------
// Reference Dijkstra (C, for validation)
// ---------------------------------------------------------------------------

static void dijkstra_ref(CchGraph *g, uint32_t src, uint32_t *dist) {
  uint32_t n = g->n;
  bool *visited = calloc(n, sizeof(bool));
  for (uint32_t i = 0; i < n; i++) dist[i] = CCH_INF;
  dist[src] = 0;

  for (uint32_t iter = 0; iter < n; iter++) {
    uint32_t u = n, du = CCH_INF;
    for (uint32_t i = 0; i < n; i++)
      if (!visited[i] && dist[i] < du) { du = dist[i]; u = i; }
    if (u == n) break;
    visited[u] = true;
    AdjList *adj = &g->fwd[u];
    for (uint32_t i = 0; i < adj->count; i++) {
      uint32_t nd = du + adj->wt[i];
      if (nd < dist[adj->to[i]]) dist[adj->to[i]] = nd;
    }
  }
  free(visited);
}

// ---------------------------------------------------------------------------
// CCH query: bidirectional Dijkstra on upward graph
// ---------------------------------------------------------------------------

static uint32_t cch_query(uint32_t src, uint32_t dst) {
  uint32_t n = g_n;
  uint32_t *df = malloc(n * sizeof(uint32_t));
  uint32_t *db = malloc(n * sizeof(uint32_t));
  bool *vf = calloc(n, sizeof(bool));
  bool *vb = calloc(n, sizeof(bool));

  for (uint32_t i = 0; i < n; i++) { df[i] = CCH_INF; db[i] = CCH_INF; }
  df[src] = 0; db[dst] = 0;

  uint32_t mu = CCH_INF;

  for (uint32_t iter = 0; iter < 2 * n; iter++) {
    // Forward step
    uint32_t uf = n, duf = CCH_INF;
    for (uint32_t i = 0; i < n; i++)
      if (!vf[i] && df[i] < duf) { duf = df[i]; uf = i; }

    if (uf < n && duf < mu) {
      vf[uf] = true;
      if (db[uf] < CCH_INF && duf + db[uf] < mu) mu = duf + db[uf];
      AdjList *adj = &g_graph.fwd[uf];
      for (uint32_t i = 0; i < adj->count; i++) {
        uint32_t v = adj->to[i];
        if (g_rank[v] > g_rank[uf]) { // upward only
          uint32_t nd = duf + adj->wt[i];
          if (nd < df[v]) {
            df[v] = nd;
            if (db[v] < CCH_INF && nd + db[v] < mu) mu = nd + db[v];
          }
        }
      }
    }

    // Backward step
    uint32_t ub = n, dub = CCH_INF;
    for (uint32_t i = 0; i < n; i++)
      if (!vb[i] && db[i] < dub) { dub = db[i]; ub = i; }

    if (ub < n && dub < mu) {
      vb[ub] = true;
      if (df[ub] < CCH_INF && df[ub] + dub < mu) mu = df[ub] + dub;
      AdjList *rev = &g_graph.rev[ub];
      for (uint32_t i = 0; i < rev->count; i++) {
        uint32_t v = rev->to[i];
        if (g_rank[v] > g_rank[ub]) { // upward only
          uint32_t nd = dub + rev->wt[i];
          if (nd < db[v]) {
            db[v] = nd;
            if (df[v] < CCH_INF && df[v] + nd < mu) mu = df[v] + nd;
          }
        }
      }
    }

    // Both sides exhausted or can't improve
    if ((uf == n || duf >= mu) && (ub == n || dub >= mu)) break;
  }

  free(df); free(db); free(vf); free(vb);
  return mu;
}

// ---------------------------------------------------------------------------
// HVM4 source generation
// ---------------------------------------------------------------------------

static uint32_t ceil_log2(uint32_t n) {
  if (n <= 1) return 1;
  uint32_t d = 0, cap = 1;
  while (cap < n) { d++; cap *= 2; }
  return d;
}

static long peak_rss_kb(void) {
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return ru.ru_maxrss;
}

#define APPEND(fmt, ...) pos += snprintf(buf + pos, cap - pos, fmt, ##__VA_ARGS__)
#define APPENDS(s) do { size_t _l = strlen(s); if (pos + _l < cap) { memcpy(buf + pos, s, _l); pos += _l; } } while(0)
#define L "\xce\xbb"

static char *gen_hvm4_source(uint32_t V) {
  size_t cap = 4096;
  char *buf = malloc(cap);
  size_t pos = 0;

  uint32_t depth = ceil_log2(V) + 1; // enough for any batch_size <= V

  APPEND("@DEPTH = %u\n\n", depth);

  // Parallel tree: sum %cch_sc(i) for i in [lo..hi)
  APPENDS(
    "// Parallel shortcut computation\n"
    "@par_c = " L "{0: " L "&lo. " L "hi. @pc1(lo < hi, lo);\n"
    "  " L "d. " L "&lo. " L "&hi.\n"
    "    ! &d1 = d - 1;\n"
    "    ! &mid = (lo + hi) / 2;\n"
    "    @par_c(d1, lo, mid) + @par_c(d1, mid, hi)}\n\n"

    "@pc1 = " L "{0: " L "lo. 0; " L "n. " L "lo. %cch_sc(lo)}\n\n"
  );

  // Parallel tree: sum %cch_up(i) for i in [lo..hi)
  APPENDS(
    "// Parallel priority update\n"
    "@par_u = " L "{0: " L "&lo. " L "hi. @pu1(lo < hi, lo);\n"
    "  " L "d. " L "&lo. " L "&hi.\n"
    "    ! &d1 = d - 1;\n"
    "    ! &mid = (lo + hi) / 2;\n"
    "    @par_u(d1, lo, mid) + @par_u(d1, mid, hi)}\n\n"

    "@pu1 = " L "{0: " L "lo. 0; " L "n. " L "lo. %cch_up(lo)}\n\n"
  );

  // Main contraction loop
  APPENDS(
    "// Contraction loop: find batch → parallel contract → apply → parallel update\n"
    "// dummy arg passed to %%cch_round to force evaluation of previous round\n"
    "@loop = " L "dummy.\n"
    "  ! &bs = %cch_round(dummy);\n"
    "  @go(bs > 0, bs)\n\n"

    "@go = " L "{\n"
    "  0: " L "bs. 0;\n"
    "  " L "n. " L "bs.\n"
    "    ! sc = @par_c(@DEPTH, 0, bs);\n"
    "    ! ac = %cch_apply(sc);\n"
    "    ! up = @par_u(@DEPTH, 0, ac);\n"
    "    @loop(up)}\n\n"

    "@main = @loop(0)\n"
  );

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V   = argc > 1 ? (uint32_t)atoi(argv[1]) : 200;
  uint32_t epn = argc > 2 ? (uint32_t)atoi(argv[2]) : 4;

  printf("=== CCH Batch-Parallel Benchmark: V=%u, ~%u edges/node ===\n", V, epn);

  // Generate graph
  gen_graph(V, epn, 42 + V, &g_graph);
  uint32_t total_edges = 0;
  for (uint32_t i = 0; i < V; i++) total_edges += g_graph.fwd[i].count;
  printf("Graph: V=%u  E=%u (undirected, both directions stored)\n", V, total_edges);

  // Reference Dijkstra (on original graph, before contraction modifies it)
  uint32_t *ref_dist = malloc(V * sizeof(uint32_t));
  dijkstra_ref(&g_graph, 0, ref_dist);
  printf("Reference: Dijkstra dist[0→%u] = %u\n", V - 1, ref_dist[V - 1]);

  // HVM4 setup
  hvm4_lib_init();
  hvm4_lib_reset();
  cch_setup(V);

  char *src = gen_hvm4_source(V);
  printf("HVM4 source: %lu bytes, tree depth: %u\n",
         (unsigned long)strlen(src), ceil_log2(V) + 1);
  if (V <= 20) {
    printf("--- HVM4 SOURCE ---\n%s--- END SOURCE ---\n", src);
  }

  // Run HVM4 contraction
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  uint32_t result;
  int count = hvm4_run(src, 0, &result, 1);

  clock_gettime(CLOCK_MONOTONIC, &t1);
  double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;

  printf("Contraction: %.3f s, %u nodes ranked in %u rounds\n",
         elapsed, g_rank_pos, g_round_num);
  printf("Peak RSS: %ld MB\n", peak_rss_kb() / 1024);

  if (count < 0) {
    printf("FAIL: hvm4_run returned %d\n", count);
    goto cleanup;
  }

  // Validate: CCH query vs reference Dijkstra
  {
    uint32_t cch_dist = cch_query(0, V - 1);
    uint32_t ref = ref_dist[V - 1];
    printf("CCH query:  dist[0→%u] = %u\n", V - 1, cch_dist);
    printf("Reference:  dist[0→%u] = %u\n", V - 1, ref);

    if (cch_dist == ref) {
      printf("PASS: CCH matches reference Dijkstra\n");
    } else {
      printf("MISMATCH: CCH=%u vs Dijkstra=%u\n", cch_dist, ref);
    }

    // Test a few more pairs
    int pass = 0, total = 0;
    uint32_t test_pairs[][2] = {
      {0, V/4}, {0, V/2}, {0, 3*V/4}, {V/4, 3*V/4}, {V/2, V-1}
    };
    for (int i = 0; i < 5 && V > 4; i++) {
      uint32_t s = test_pairs[i][0], d = test_pairs[i][1];
      if (s >= V || d >= V) continue;
      uint32_t *rdist = malloc(V * sizeof(uint32_t));
      dijkstra_ref(&g_graph, s, rdist);
      // Note: Dijkstra on augmented graph (with shortcuts) gives same results
      // because shortcuts preserve shortest-path distances
      uint32_t cq = cch_query(s, d);
      total++;
      if (cq == rdist[d]) { pass++; }
      else {
        printf("  query(%u,%u): CCH=%u vs Dijk=%u MISMATCH\n", s, d, cq, rdist[d]);
      }
      free(rdist);
    }
    if (total > 0)
      printf("Additional queries: %d/%d passed\n", pass, total);
  }

cleanup:
  cch_cleanup();
  cgraph_free(&g_graph);
  free(ref_dist);
  free(src);
  hvm4_lib_cleanup();
  return 0;
}
