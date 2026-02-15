// Path enumeration benchmark (DFS on sparse DAG)
// Sparse DAG in C memory (CSR), HVM4 for recursive DFS with list accumulation.
// Compile: clang -O2 -o bench/path_enum bench/path_enum.c -lpthread
// Usage:   ./bench/path_enum [V] [K]
//   V = total nodes (default 100), K = branch points (default 15)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>

#include "../c3lib/csrc/hvm4_bridge.c"

// ---------------------------------------------------------------------------
// DAG generator: spine with skip edges
// ---------------------------------------------------------------------------
//
// Creates a sparse DAG with H "spine" nodes evenly spaced through V nodes.
// - Spine edges: spine[k] -> spine[k+1] with weight 1-10
// - K skip edges: spine[i] -> spine[i+2] with weight 1-10 (skip one spine node)
// - All non-spine nodes are dead-ends (degree 0)
// - Total paths from spine[0] to spine[H-1]: up to 2^K

typedef struct { uint32_t s; } LCG;

static uint32_t lcg_next(LCG *r) {
  r->s = (r->s * 1103515245u + 12345u) & 0x7fffffffu;
  return r->s;
}

static void gen_dag(uint32_t V, uint32_t K,
                    uint32_t **row_ptr_out, uint32_t **col_idx_out,
                    uint32_t **weight_out, uint32_t *edge_count_out,
                    uint32_t *spine_out, uint32_t *H_out) {
  LCG rng = { .s = 12345 + V + K };

  // Spine: H evenly spaced nodes including 0 and V-1
  uint32_t H = K + 5;  // spine length (enough for K skip points)
  if (H < 4) H = 4;
  if (H > V) H = V;
  *H_out = H;

  uint32_t *spine = malloc(H * sizeof(uint32_t));
  for (uint32_t i = 0; i < H; i++) {
    spine[i] = (uint32_t)((uint64_t)i * (V - 1) / (H - 1));
  }
  spine[0] = 0;
  spine[H - 1] = V - 1;

  // Count edges: H-1 spine edges + K skip edges
  uint32_t max_edges = (H - 1) + K;
  uint32_t *src_arr = malloc(max_edges * sizeof(uint32_t));
  uint32_t *dst_arr = malloc(max_edges * sizeof(uint32_t));
  uint32_t *wt_arr = malloc(max_edges * sizeof(uint32_t));
  uint32_t ne = 0;

  // Spine edges
  for (uint32_t i = 0; i + 1 < H; i++) {
    src_arr[ne] = spine[i];
    dst_arr[ne] = spine[i + 1];
    wt_arr[ne] = lcg_next(&rng) % 10 + 1;
    ne++;
  }

  // Skip edges: spine[i] -> spine[i+2] for first K skip points
  for (uint32_t i = 0; i < K && i + 2 < H; i++) {
    src_arr[ne] = spine[i];
    dst_arr[ne] = spine[i + 2];
    wt_arr[ne] = lcg_next(&rng) % 10 + 1;
    ne++;
  }

  // Convert to CSR
  uint32_t *rp = calloc(V + 2, sizeof(uint32_t));
  for (uint32_t e = 0; e < ne; e++) rp[src_arr[e] + 1]++;
  for (uint32_t i = 1; i <= V; i++) rp[i] += rp[i - 1];

  uint32_t *ci = malloc(ne * sizeof(uint32_t));
  uint32_t *wt = malloc(ne * sizeof(uint32_t));
  uint32_t *pos = malloc((V + 1) * sizeof(uint32_t));
  memcpy(pos, rp, (V + 1) * sizeof(uint32_t));

  for (uint32_t e = 0; e < ne; e++) {
    uint32_t p = pos[src_arr[e]]++;
    ci[p] = dst_arr[e];
    wt[p] = wt_arr[e];
  }

  free(pos);
  free(src_arr); free(dst_arr); free(wt_arr);
  memcpy(spine_out, spine, H * sizeof(uint32_t));
  free(spine);

  *row_ptr_out = rp;
  *col_idx_out = ci;
  *weight_out = wt;
  *edge_count_out = ne;
}

// ---------------------------------------------------------------------------
// Reference: C recursive DFS
// ---------------------------------------------------------------------------

static uint32_t *g_rp, *g_ci, *g_wt;
static uint32_t *g_paths;
static int g_path_count, g_path_max;

static void c_dfs(uint32_t node, uint32_t dest, uint32_t weight) {
  if (node == dest) {
    if (g_path_count < g_path_max) {
      g_paths[g_path_count] = weight;
    }
    g_path_count++;
    return;
  }
  uint32_t deg = g_rp[node + 1] - g_rp[node];
  for (uint32_t i = 0; i < deg; i++) {
    uint32_t v = g_ci[g_rp[node] + i];
    uint32_t w = g_wt[g_rp[node] + i];
    c_dfs(v, dest, weight + w);
  }
}

static int cmp_u32(const void *a, const void *b) {
  uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
  return (x > y) - (x < y);
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

static char *gen_hvm4_source(uint32_t dest) {
  size_t cap = 8192;
  char *buf = malloc(cap);
  size_t pos = 0;

  // apnd (list append: linear in first arg, DUP second via &ys)
  APPENDS(
    "@apnd = " L "xs. " L "&ys. @apgo(xs, ys)\n"
    "@apgo = " L "{"
    "[]: " L "ys. ys; "
    "<>: " L "h. " L "t. " L "&ys. h <> @apnd(t, ys)}\n"
  );

  // expl (explore: DFS from node, accumulating weight)
  APPEND(
    "@expl = " L "&node. " L "&wt. "
    "@exgo(node == %u, node, wt)\n",
    dest
  );

  // exgo: if node == dest, return [wt]; else branch over neighbors
  APPENDS(
    "@exgo = " L "{"
    "0: " L "&node. " L "wt. @br(node, wt, 0, %graph_deg(node)); "
    "" L "n. " L "node. " L "wt. wt <> []}\n"
  );

  // br (branch over neighbors i..deg of node)
  APPENDS(
    "@br = " L "&node. " L "&wt. " L "&i. " L "&deg. "
    "@brgo(i < deg, node, wt, i, deg)\n"

    "@brgo = " L "{"
    "0: " L "node. " L "wt. " L "i. " L "deg. []; "
    "" L "n. " L "&node. " L "&wt. " L "&i. " L "deg. "
    "! &v = %graph_target(node, i); "
    "! &w = %graph_weight(node, i); "
    "! &sub = @expl(v, wt + w); "
    "! &rest = @br(node, wt, i + 1, deg); "
    "@apnd(sub, rest)}\n"
  );

  // main
  APPENDS("@main = @expl(0, 0)\n");

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V = argc > 1 ? (uint32_t)atoi(argv[1]) : 100;
  uint32_t K = argc > 2 ? (uint32_t)atoi(argv[2]) : 15;

  printf("=== Path enumeration benchmark (DFS on sparse DAG): V=%u K=%u ===\n", V, K);

  // Generate sparse DAG
  uint32_t *rp, *ci, *wt, ne;
  uint32_t H;
  uint32_t *spine = malloc(1024 * sizeof(uint32_t));
  gen_dag(V, K, &rp, &ci, &wt, &ne, spine, &H);
  uint32_t dest = V - 1;
  printf("Graph: V=%u  E=%u  spine=%u nodes  K=%u branch points\n", V, ne, H, K);

  // C reference DFS
  g_rp = rp; g_ci = ci; g_wt = wt;
  int max_paths = 1 << (K < 20 ? K : 20);
  g_paths = malloc(max_paths * sizeof(uint32_t));
  g_path_max = max_paths;
  g_path_count = 0;
  c_dfs(0, dest, 0);
  int ref_count = g_path_count;
  qsort(g_paths, ref_count < max_paths ? ref_count : max_paths, sizeof(uint32_t), cmp_u32);
  printf("Reference: %d paths found\n", ref_count);
  if (ref_count <= 16) {
    printf("  Paths:");
    for (int i = 0; i < ref_count && i < max_paths; i++) printf(" %u", g_paths[i]);
    printf("\n");
  }

  // Init HVM4 runtime
  hvm4_lib_init();

  // Generate HVM4 source
  char *hvm_src = gen_hvm4_source(dest);
  printf("HVM4 source: %lu bytes\n", (unsigned long)strlen(hvm_src));
  if (V <= 20) {
    printf("--- HVM4 SOURCE ---\n%s--- END SOURCE ---\n", hvm_src);
  }

  // Setup
  hvm4_lib_reset();
  hvm4_graph_setup(rp, ci, wt, V);

  // Run
  uint32_t *hvm_results = calloc(max_paths, sizeof(uint32_t));
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  int hvm_count = hvm4_run(hvm_src, 0, hvm_results, max_paths);

  clock_gettime(CLOCK_MONOTONIC, &t1);
  double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;

  printf("Time: %.3f s\n", elapsed);
  printf("Peak RSS: %ld MB\n", peak_rss_kb() / 1024);

  if (hvm_count < 0) {
    printf("FAIL: hvm4_run returned %d\n", hvm_count);
    hvm4_lib_cleanup();
    free(hvm_src); free(rp); free(ci); free(wt); free(spine);
    free(g_paths); free(hvm_results);
    return 1;
  }

  printf("HVM4: %d paths found\n", hvm_count);

  // Sort and compare
  qsort(hvm_results, hvm_count, sizeof(uint32_t), cmp_u32);

  int ok = (hvm_count == ref_count);
  if (ok) {
    int cmp_count = hvm_count < max_paths ? hvm_count : max_paths;
    for (int i = 0; i < cmp_count; i++) {
      if (hvm_results[i] != g_paths[i]) {
        ok = 0;
        printf("MISMATCH at path[%d]: HVM4=%u ref=%u\n", i, hvm_results[i], g_paths[i]);
        break;
      }
    }
  }

  if (ok) {
    printf("PASS: %d paths match reference\n", hvm_count);
    if (hvm_count <= 16) {
      printf("  Paths:");
      for (int i = 0; i < hvm_count; i++) printf(" %u", hvm_results[i]);
      printf("\n");
    }
  } else {
    printf("FAIL: HVM4 found %d paths, reference found %d\n", hvm_count, ref_count);
    if (hvm_count <= 32) {
      printf("  HVM4:");
      for (int i = 0; i < hvm_count; i++) printf(" %u", hvm_results[i]);
      printf("\n");
    }
    if (ref_count <= 32) {
      printf("  Ref: ");
      for (int i = 0; i < ref_count && i < max_paths; i++) printf(" %u", g_paths[i]);
      printf("\n");
    }
  }

  free(hvm_src); free(rp); free(ci); free(wt); free(spine);
  free(g_paths); free(hvm_results);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
