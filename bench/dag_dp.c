// DAG-DP benchmark driver
// Graph structure embedded directly in HVM4 let-binding chain. No FFI needed.
// Each node becomes a `! &XXXX = ...` binding; @min reduces multiple edges.
// O(E) work, O(V+E) heap terms. Limit: V <= 131072 (PARSE_BINDS).
//
// Variable names use 4-char base-64 encoding to avoid nick hash collisions.
// (HVM4 nick encoding: 6 bits/char, EXT_MASK=0xFFFFFF → 4 chars = 24 bits exact.)
//
// Compile: clang -O2 -o bench/dag_dp bench/dag_dp.c -lpthread
// Usage:   ./bench/dag_dp [V] [edges_per_node]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>
#include <pthread.h>

#include "../c3lib/csrc/hvm4_bridge.c"

// ---------------------------------------------------------------------------
// 4-char collision-free variable names (base-64 in HVM4 nick alphabet)
// ---------------------------------------------------------------------------

// HVM4 nick b64 alphabet: _abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789$
static const char B64[64] =
  "_abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789$";

// Encode node number n as 4-char name: prefix 'e' + 3 base-64 digits.
// Supports n in [0, 262143]. All names are unique under 24-bit nick hash.
static inline void node_name(uint32_t n, char *buf) {
  buf[0] = 'e';
  buf[1] = B64[(n >> 12) & 63];
  buf[2] = B64[(n >> 6) & 63];
  buf[3] = B64[n & 63];
  buf[4] = '\0';
}

// ---------------------------------------------------------------------------
// LCG random DAG generator (deterministic, forward-only edges)
// ---------------------------------------------------------------------------

typedef struct { uint32_t s; } LCG;

static uint32_t lcg_next(LCG *r) {
  r->s = (r->s * 1103515245u + 12345u) & 0x7fffffffu;
  return r->s;
}

typedef struct { uint32_t u, v, w; } RawEdge;

// Generate a random DAG: chain 0->1->...->V-1 + forward-only random edges.
// All edges satisfy u < v, so natural node order is topological order.
static void gen_dag(uint32_t n, uint32_t epn, uint32_t seed,
                    uint32_t **row_ptr_out, uint32_t **col_idx_out,
                    uint32_t **weight_out, uint32_t *edge_count_out) {
  LCG rng = { .s = seed };
  uint32_t target_e = n * epn;
  uint32_t max_e = target_e + n;
  RawEdge *edges = malloc(max_e * sizeof(RawEdge));
  uint32_t ne = 0;

  // Chain for connectivity (forward: i -> i+1)
  for (uint32_t i = 0; i + 1 < n; i++) {
    uint32_t w = lcg_next(&rng) % 10 + 1;
    edges[ne++] = (RawEdge){i, i + 1, w};
  }

  // Random forward-only edges (u < v ensures DAG)
  uint32_t attempts = n * (epn - 1) * 3;
  for (uint32_t a = 0; a < attempts && ne < target_e; a++) {
    uint32_t u = lcg_next(&rng) % n;
    uint32_t v = lcg_next(&rng) % n;
    if (u >= v) continue; // forward-only
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
// Reference DAG shortest path (DP in reverse topological order)
// ---------------------------------------------------------------------------

#define INF 999999

static uint32_t dag_dp_reference(uint32_t n, uint32_t *rp, uint32_t *ci,
                                 uint32_t *wt, uint32_t src, uint32_t dest) {
  uint32_t *dist = malloc(n * sizeof(uint32_t));
  for (uint32_t i = 0; i < n; i++) dist[i] = INF;
  dist[dest] = 0;

  // Reverse topological order (n-1 down to 0, valid since all edges forward)
  for (int32_t u = (int32_t)n - 1; u >= 0; u--) {
    if ((uint32_t)u == dest) continue;
    for (uint32_t e = rp[u]; e < rp[u + 1]; e++) {
      uint32_t nd = wt[e] + dist[ci[e]];
      if (nd < dist[u]) dist[u] = nd;
    }
  }

  uint32_t result = dist[src];
  free(dist);
  return result;
}

// ---------------------------------------------------------------------------
// HVM4 source generation (DAG-DP: nested let-bindings)
// ---------------------------------------------------------------------------

static long peak_rss_kb(void) {
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return ru.ru_maxrss; // KB on Linux
}

// UTF-8 lambda: split to avoid C hex escape merging
#define L "\xce" "\xbb"

static char *gen_hvm4_source(uint32_t n, uint32_t *rp, uint32_t *ci,
                             uint32_t *wt, uint32_t src, uint32_t dest) {
  uint32_t ne = rp[n];
  // ~48 bytes per edge + 24 bytes per node overhead (4-char names are shorter)
  size_t cap = 512 + (size_t)ne * 48 + (size_t)n * 24;
  char *buf = malloc(cap);
  if (!buf) return NULL;
  size_t pos = 0;
  char name[8], tname[8];

#define APPEND(fmt, ...) pos += snprintf(buf + pos, cap - pos, fmt, ##__VA_ARGS__)

  // Definitions
  APPEND("@min = " L "&a. " L "&b. " L "{0: b; " L "n. a}(a < b)\n");
  APPEND("@INF = 999999\n");
  APPEND("@main =\n");

  // Destination binding
  node_name(dest, name);
  APPEND("  ! &%s = 0;\n", name);

  // Non-source, non-dest nodes in reverse order
  for (int32_t u = (int32_t)n - 2; u >= 0; u--) {
    if ((uint32_t)u == src) continue;
    if ((uint32_t)u == dest) continue;

    node_name((uint32_t)u, name);
    uint32_t deg = rp[u + 1] - rp[u];
    if (deg == 0) {
      APPEND("  ! &%s = @INF;\n", name);
    } else if (deg == 1) {
      uint32_t e = rp[u];
      node_name(ci[e], tname);
      APPEND("  ! &%s = %u + %s;\n", name, wt[e], tname);
    } else {
      // Multiple edges: @min(w1 + t1, @min(w2 + t2, ... wk + tk))
      APPEND("  ! &%s = ", name);
      for (uint32_t i = 0; i < deg - 1; i++) {
        uint32_t e = rp[u] + i;
        node_name(ci[e], tname);
        APPEND("@min(%u + %s, ", wt[e], tname);
      }
      uint32_t e = rp[u] + deg - 1;
      node_name(ci[e], tname);
      APPEND("%u + %s", wt[e], tname);
      for (uint32_t i = 0; i < deg - 1; i++) APPEND(")");
      APPEND(";\n");
    }
  }

  // Source node: return expression (not bound)
  uint32_t src_deg = rp[src + 1] - rp[src];
  if (src_deg == 0) {
    APPEND("  @INF\n");
  } else if (src_deg == 1) {
    uint32_t e = rp[src];
    node_name(ci[e], tname);
    APPEND("  %u + %s\n", wt[e], tname);
  } else {
    APPEND("  ");
    for (uint32_t i = 0; i < src_deg - 1; i++) {
      uint32_t e = rp[src] + i;
      node_name(ci[e], tname);
      APPEND("@min(%u + %s, ", wt[e], tname);
    }
    uint32_t e = rp[src] + src_deg - 1;
    node_name(ci[e], tname);
    APPEND("%u + %s", wt[e], tname);
    for (uint32_t i = 0; i < src_deg - 1; i++) APPEND(")");
    APPEND("\n");
  }

#undef APPEND

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V   = argc > 1 ? (uint32_t)atoi(argv[1]) : 100;
  uint32_t epn = argc > 2 ? (uint32_t)atoi(argv[2]) : 4;

  if (V > 131072) {
    printf("ERROR: V=%u exceeds PARSE_BINDS limit (131072).\n", V);
    return 1;
  }
  if (V > 262144) {
    printf("ERROR: V=%u exceeds 4-char nick encoding limit (262144).\n", V);
    return 1;
  }
  if (V < 2) {
    printf("V must be >= 2\n");
    return 1;
  }

  printf("=== DAG-DP benchmark: V=%u, ~%u edges/node ===\n", V, epn);

  // Generate DAG
  uint32_t *rp, *ci, *wt, ne;
  gen_dag(V, epn, 42 + V, &rp, &ci, &wt, &ne);
  printf("Graph: V=%u  E=%u\n", V, ne);

  // Reference solution
  uint32_t ref = dag_dp_reference(V, rp, ci, wt, 0, V - 1);
  printf("Reference: dist[0->%u]=%u\n", V - 1, ref);

  // Init HVM4 runtime
  hvm4_lib_init();

  // Generate HVM4 source
  char *src = gen_hvm4_source(V, rp, ci, wt, 0, V - 1);
  if (!src) {
    printf("FAIL: source generation OOM\n");
    hvm4_lib_cleanup();
    return 1;
  }
  printf("HVM4 source: %lu bytes\n", (unsigned long)strlen(src));
  if (V <= 10) {
    printf("--- HVM4 SOURCE ---\n%s--- END SOURCE ---\n", src);
  }

  // Run
  hvm4_lib_reset();

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
    free(src);
    free(rp);
    free(ci);
    free(wt);
    return 1;
  }

  printf("HVM4 result: %u\n", result);

  // Validate
  int ok = (count >= 1 && result == ref);
  if (ok) {
    printf("PASS: dist[0->%u] = %u matches reference\n", V - 1, result);
  } else {
    printf("FAIL: got %u (count=%d), expected %u\n", result, count, ref);
  }

  free(src);
  free(rp);
  free(ci);
  free(wt);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
