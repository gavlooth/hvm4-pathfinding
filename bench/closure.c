// Boolean transitive closure benchmark (boolean semiring matrix squaring)
// Dense graph in C memory (CSR), HVM4 for matrix operations (flat-list DUP).
// Compile: clang -O2 -o bench/closure bench/closure.c -lpthread
// Usage:   ./bench/closure [V]

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

// Generate dense directed graph (~50% edge probability).
// Returns CSR arrays + dense boolean adjacency matrix (1 = edge, 0 = no edge).
static void gen_graph(uint32_t n, uint32_t seed,
                      uint32_t **row_ptr_out, uint32_t **col_idx_out,
                      uint32_t **weight_out, uint32_t *edge_count_out,
                      uint32_t **adj_out) {
  LCG rng = { .s = seed };

  // Dense boolean adjacency matrix (1 on diagonal = self-reachable)
  uint32_t *adj = malloc(n * n * sizeof(uint32_t));
  for (uint32_t i = 0; i < n; i++)
    for (uint32_t j = 0; j < n; j++)
      adj[i * n + j] = (i == j) ? 1 : 0;

  // Add edges with ~50% probability
  uint32_t ne = 0;
  for (uint32_t i = 0; i < n; i++) {
    for (uint32_t j = 0; j < n; j++) {
      if (i == j) continue;
      if (lcg_next(&rng) % 2 == 0) {
        adj[i * n + j] = 1;
        ne++;
      }
    }
  }

  // Convert to CSR (weight = 1 for all edges)
  uint32_t *rp = calloc(n + 2, sizeof(uint32_t));
  for (uint32_t i = 0; i < n; i++)
    for (uint32_t j = 0; j < n; j++)
      if (i != j && adj[i * n + j]) rp[i + 1]++;
  for (uint32_t i = 1; i <= n; i++) rp[i] += rp[i - 1];

  uint32_t *ci = malloc(ne * sizeof(uint32_t));
  uint32_t *wt = malloc(ne * sizeof(uint32_t));
  uint32_t *pos = malloc((n + 1) * sizeof(uint32_t));
  memcpy(pos, rp, (n + 1) * sizeof(uint32_t));

  for (uint32_t i = 0; i < n; i++)
    for (uint32_t j = 0; j < n; j++)
      if (i != j && adj[i * n + j]) {
        uint32_t p = pos[i]++;
        ci[p] = j;
        wt[p] = 1;
      }

  free(pos);
  *row_ptr_out = rp;
  *col_idx_out = ci;
  *weight_out = wt;
  *edge_count_out = ne;
  *adj_out = adj;
}

// ---------------------------------------------------------------------------
// Reference: Warshall's algorithm (boolean transitive closure)
// ---------------------------------------------------------------------------

static void warshall(uint32_t n, uint32_t *adj, uint32_t *reach) {
  memcpy(reach, adj, n * n * sizeof(uint32_t));
  for (uint32_t k = 0; k < n; k++)
    for (uint32_t i = 0; i < n; i++)
      for (uint32_t j = 0; j < n; j++) {
        if (reach[i * n + k] && reach[k * n + j])
          reach[i * n + j] = 1;
      }
}

// ---------------------------------------------------------------------------
// HVM4 source generation
// ---------------------------------------------------------------------------

static uint32_t ceil_log2(uint32_t n) {
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
#define L "\xce" "\xbb"

static char *gen_hvm4_source(uint32_t n, uint32_t src, uint32_t dest) {
  uint32_t iters = ceil_log2(n);

  size_t cap = 16384;
  char *buf = malloc(cap);
  size_t pos = 0;

  APPEND("@N = %u\n", n);

  // bor (boolean OR)
  APPENDS("@bor = " L "&a. " L "&b. " L "{0: b; " L "n. 1}(a)\n");

  // band (boolean AND) — multiply analog
  APPENDS("@band = " L "&a. " L "&b. " L "{0: 0; " L "n. b}(a)\n");

  // nth (list indexing, default = 0)
  APPENDS(
    "@nth = " L "&i. " L "{"
    "[]: 0; "
    "<>: " L "&h. " L "t. @ngo(i, h, t)}\n"
    "@ngo = " L "{"
    "0: " L "h. " L "t. h; "
    "" L "k. " L "h. " L "t. @nth(k - 1, t)}\n"
  );

  // bdot (boolean dot product: OR over k of (A[i,k] AND B[k,j]))
  APPENDS(
    "@bdot = " L "&A. " L "&B. " L "&N. " L "&i. " L "&j. " L "&k. " L "&rem. "
    "@bdgo(rem, A, B, N, i, j, k)\n"

    "@bdgo = " L "{"
    "0: " L "A. " L "B. " L "N. " L "i. " L "j. " L "k. 0; "
    "" L "r. " L "&A. " L "&B. " L "&N. " L "&i. " L "&j. " L "&k. "
    "! &aik = @nth(i * N + k, A); "
    "! &bkj = @nth(k * N + j, B); "
    "! &trm = @band(aik, bkj); "
    "! &rst = @bdot(A, B, N, i, j, k + 1, r - 1); "
    "@bor(trm, rst)}\n"
  );

  // bld (build result cells for mat_mul)
  APPENDS(
    "@bld = " L "&A. " L "&B. " L "&N. " L "&idx. " L "&rem. "
    "@blgo(rem, A, B, N, idx)\n"

    "@blgo = " L "{"
    "0: " L "A. " L "B. " L "N. " L "idx. []; "
    "" L "r. " L "&A. " L "&B. " L "&N. " L "&idx. "
    "! &i = idx / N; ! &j = idx % N; "
    "! &val = @bdot(A, B, N, i, j, 0, N); "
    "val <> @bld(A, B, N, idx + 1, r - 1)}\n"
  );

  // mmul (boolean matrix multiply wrapper)
  APPENDS("@mmul = " L "&A. " L "&B. " L "&N. @bld(A, B, N, 0, N * N)\n");

  // bini (build initial boolean matrix from FFI)
  APPENDS(
    "@bini = " L "&idx. " L "&rem. " L "&N. "
    "@bigo(rem, idx, N)\n"

    "@bigo = " L "{"
    "0: " L "idx. " L "N. []; "
    "" L "r. " L "&idx. " L "&N. "
    "! &i = idx / N; ! &j = idx % N; "
    "! &val = @cel(i, j); "
    "val <> @bini(idx + 1, r - 1, N)}\n"
  );

  // cel (single cell: 1 on diagonal, check edge otherwise)
  APPENDS(
    "@cel = " L "&i. " L "&j. "
    "@ceq(i == j, i, j)\n"

    "@ceq = " L "{"
    "0: " L "&i. " L "j. @fed(i, j, 0, %graph_deg(i)); "
    "" L "k. " L "i. " L "j. 1}\n"
  );

  // fed (find edge u->j: returns 1 if found, 0 if not)
  APPENDS(
    "@fed = " L "&u. " L "&j. " L "&k. " L "&deg. "
    "@fego(k < deg, u, j, k, deg)\n"

    "@fego = " L "{"
    "0: " L "u. " L "j. " L "k. " L "deg. 0; "
    "" L "n. " L "&u. " L "&j. " L "&k. " L "deg. "
    "! &t = %graph_target(u, k); "
    "@fck(t == j, u, j, k, deg)}\n"

    "@fck = " L "{"
    "0: " L "u. " L "j. " L "k. " L "deg. @fed(u, j, k + 1, deg); "
    "" L "n. " L "u. " L "j. " L "k. " L "deg. 1}\n"
  );

  // mor (element-wise OR of two flat boolean matrices)
  APPENDS(
    "@mor = " L "a. " L "b. @morga(a, b)\n"

    "@morga = " L "{"
    "[]: " L "b. []; "
    "<>: " L "ah. " L "at. " L "b. @mor2(b, ah, at)}\n"

    "@mor2 = " L "{"
    "[]: " L "ah. " L "at. ah <> at; "
    "<>: " L "bh. " L "bt. " L "ah. " L "at. "
    "! &val = @bor(ah, bh); "
    "val <> @mor(at, bt)}\n"
  );

  // clos (repeated squaring)
  APPENDS(
    "@clos = " L "&iter. " L "&D. " L "&N. "
    "@clgo(iter, D, N)\n"

    "@clgo = " L "{"
    "0: " L "D. " L "N. D; "
    "" L "k. " L "&D. " L "&N. "
    "! &D2 = @mmul(D, D, N); "
    "! &Dnw = @mor(D, D2); "
    "@clos(k - 1, Dnw, N)}\n"
  );

  // main
  APPEND(
    "@main = "
    "! &D = @bini(0, %u, @N); "
    "! &R = @clos(%u, D, @N); "
    "@nth(%u, R)\n",
    n * n, iters, src * n + dest
  );

  buf[pos] = '\0';
  return buf;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  uint32_t V = argc > 1 ? (uint32_t)atoi(argv[1]) : 20;
  if (V > 40) printf("WARNING: V=%u will be very slow (O(V^3 log V))\n", V);

  printf("=== Transitive closure benchmark (boolean semiring): V=%u ===\n", V);

  // Generate dense graph
  uint32_t *rp, *ci, *wt, ne;
  uint32_t *adj;
  gen_graph(V, 42 + V, &rp, &ci, &wt, &ne, &adj);
  printf("Graph: V=%u  E=%u (directed, ~50%% density)\n", V, ne);

  // Warshall reference
  uint32_t src = 0, dest = V - 1;
  uint32_t *reach = malloc(V * V * sizeof(uint32_t));
  warshall(V, adj, reach);
  uint32_t ref = reach[src * V + dest];
  printf("Reference: reach[%u->%u] = %u\n", src, dest, ref);

  // Init HVM4 runtime
  hvm4_lib_init();

  // Generate HVM4 source
  char *hvm_src = gen_hvm4_source(V, src, dest);
  printf("HVM4 source: %lu bytes\n", (unsigned long)strlen(hvm_src));
  if (V <= 5) {
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
    free(hvm_src); free(rp); free(ci); free(wt); free(adj); free(reach);
    return 1;
  }

  printf("HVM4 result: %u\n", result);

  int ok = (count >= 1 && result == ref);
  if (ok) {
    printf("PASS: reach[%u->%u] = %u matches Warshall\n", src, dest, result);
  } else {
    printf("FAIL: got %u (count=%d), expected %u\n", result, count, ref);
  }

  free(hvm_src); free(rp); free(ci); free(wt); free(adj); free(reach);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
