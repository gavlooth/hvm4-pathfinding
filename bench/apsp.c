// Algebraic APSP benchmark driver (tropical semiring matrix squaring)
// Dense graph in C memory (CSR), HVM4 for matrix operations (flat-list DUP).
// Compile: clang -O2 -o bench/apsp bench/apsp.c -lpthread
// Usage:   ./bench/apsp [V]

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

#define INF 999999

// Generate dense directed weighted graph (~50% edge probability).
// Returns CSR arrays + dense adjacency matrix.
static void gen_graph(uint32_t n, uint32_t seed,
                      uint32_t **row_ptr_out, uint32_t **col_idx_out,
                      uint32_t **weight_out, uint32_t *edge_count_out,
                      uint32_t **adj_out) {
  LCG rng = { .s = seed };

  // Dense adjacency matrix
  uint32_t *adj = malloc(n * n * sizeof(uint32_t));
  for (uint32_t i = 0; i < n; i++)
    for (uint32_t j = 0; j < n; j++)
      adj[i * n + j] = (i == j) ? 0 : INF;

  // Add edges with ~50% probability
  uint32_t ne = 0;
  for (uint32_t i = 0; i < n; i++) {
    for (uint32_t j = 0; j < n; j++) {
      if (i == j) continue;
      if (lcg_next(&rng) % 2 == 0) {
        uint32_t w = lcg_next(&rng) % 20 + 1;
        adj[i * n + j] = w;
        ne++;
      }
    }
  }

  // Convert to CSR
  uint32_t *rp = calloc(n + 2, sizeof(uint32_t));
  for (uint32_t i = 0; i < n; i++)
    for (uint32_t j = 0; j < n; j++)
      if (i != j && adj[i * n + j] < INF) rp[i + 1]++;
  for (uint32_t i = 1; i <= n; i++) rp[i] += rp[i - 1];

  uint32_t *ci = malloc(ne * sizeof(uint32_t));
  uint32_t *wt = malloc(ne * sizeof(uint32_t));
  uint32_t *pos = malloc((n + 1) * sizeof(uint32_t));
  memcpy(pos, rp, (n + 1) * sizeof(uint32_t));

  for (uint32_t i = 0; i < n; i++)
    for (uint32_t j = 0; j < n; j++)
      if (i != j && adj[i * n + j] < INF) {
        uint32_t p = pos[i]++;
        ci[p] = j;
        wt[p] = adj[i * n + j];
      }

  free(pos);
  *row_ptr_out = rp;
  *col_idx_out = ci;
  *weight_out = wt;
  *edge_count_out = ne;
  *adj_out = adj;
}

// ---------------------------------------------------------------------------
// Reference: Floyd-Warshall
// ---------------------------------------------------------------------------

static void floyd_warshall(uint32_t n, uint32_t *adj, uint32_t *dist) {
  memcpy(dist, adj, n * n * sizeof(uint32_t));
  for (uint32_t k = 0; k < n; k++)
    for (uint32_t i = 0; i < n; i++)
      for (uint32_t j = 0; j < n; j++) {
        uint32_t via = dist[i * n + k] + dist[k * n + j];
        if (via < dist[i * n + j]) dist[i * n + j] = via;
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

  APPEND("@INF = %u\n@N = %u\n", INF, n);

  // min
  APPENDS("@min = " L "&a. " L "&b. " L "{0: b; " L "n. a}(a < b)\n");

  // nth (list indexing)
  APPENDS(
    "@nth = " L "&i. " L "{"
    "[]: @INF; "
    "<>: " L "&h. " L "t. @ngo(i, h, t)}\n"
    "@ngo = " L "{"
    "0: " L "h. " L "t. h; "
    "" L "k. " L "h. " L "t. @nth(k - 1, t)}\n"
  );

  // dot (tropical dot product)
  APPENDS(
    "@dot = " L "&A. " L "&B. " L "&N. " L "&i. " L "&j. " L "&k. " L "&rem. "
    "@dgo(rem, A, B, N, i, j, k)\n"

    "@dgo = " L "{"
    "0: " L "A. " L "B. " L "N. " L "i. " L "j. " L "k. @INF; "
    "" L "r. " L "&A. " L "&B. " L "&N. " L "&i. " L "&j. " L "&k. "
    "! &aik = @nth(i * N + k, A); "
    "! &bkj = @nth(k * N + j, B); "
    "! &trm = aik + bkj; "
    "! &rst = @dot(A, B, N, i, j, k + 1, r - 1); "
    "@min(trm, rst)}\n"
  );

  // bld (build result cells for mat_mul)
  APPENDS(
    "@bld = " L "&A. " L "&B. " L "&N. " L "&idx. " L "&rem. "
    "@blgo(rem, A, B, N, idx)\n"

    "@blgo = " L "{"
    "0: " L "A. " L "B. " L "N. " L "idx. []; "
    "" L "r. " L "&A. " L "&B. " L "&N. " L "&idx. "
    "! &i = idx / N; ! &j = idx % N; "
    "! &val = @dot(A, B, N, i, j, 0, N); "
    "val <> @bld(A, B, N, idx + 1, r - 1)}\n"
  );

  // mmul (matrix multiply wrapper)
  APPENDS("@mmul = " L "&A. " L "&B. " L "&N. @bld(A, B, N, 0, N * N)\n");

  // bini (build initial matrix from FFI)
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

  // cel (single cell: 0 on diagonal, scan edges otherwise)
  APPENDS(
    "@cel = " L "&i. " L "&j. "
    "@ceq(i == j, i, j)\n"

    "@ceq = " L "{"
    "0: " L "&i. " L "j. @fed(i, j, 0, %graph_deg(i)); "
    "" L "k. " L "i. " L "j. 0}\n"
  );

  // fed (find edge u->j by scanning neighbors)
  APPENDS(
    "@fed = " L "&u. " L "&j. " L "&k. " L "&deg. "
    "@fego(k < deg, u, j, k, deg)\n"

    "@fego = " L "{"
    "0: " L "u. " L "j. " L "k. " L "deg. @INF; "
    "" L "n. " L "&u. " L "&j. " L "&k. " L "deg. "
    "! &t = %graph_target(u, k); "
    "@fck(t == j, u, j, k, deg)}\n"

    "@fck = " L "{"
    "0: " L "u. " L "j. " L "k. " L "deg. @fed(u, j, k + 1, deg); "
    "" L "n. " L "u. " L "j. " L "k. " L "deg. %graph_weight(u, k)}\n"
  );

  // mmin (element-wise min of two flat matrices)
  APPENDS(
    "@mmin = " L "a. " L "b. @mmga(a, b)\n"

    "@mmga = " L "{"
    "[]: " L "b. []; "
    "<>: " L "ah. " L "at. " L "b. @mm2(b, ah, at)}\n"

    "@mm2 = " L "{"
    "[]: " L "ah. " L "at. ah <> at; "
    "<>: " L "bh. " L "bt. " L "ah. " L "at. "
    "! &val = @min(ah, bh); "
    "val <> @mmin(at, bt)}\n"
  );

  // clos (repeated squaring)
  APPENDS(
    "@clos = " L "&iter. " L "&D. " L "&N. "
    "@clgo(iter, D, N)\n"

    "@clgo = " L "{"
    "0: " L "D. " L "N. D; "
    "" L "k. " L "&D. " L "&N. "
    "! &D2 = @mmul(D, D, N); "
    "! &Dnw = @mmin(D, D2); "
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

  printf("=== APSP benchmark (tropical semiring): V=%u ===\n", V);

  // Generate dense graph
  uint32_t *rp, *ci, *wt, ne;
  uint32_t *adj;
  gen_graph(V, 42 + V, &rp, &ci, &wt, &ne, &adj);
  printf("Graph: V=%u  E=%u (directed, ~50%% density)\n", V, ne);

  // Floyd-Warshall reference
  uint32_t src = 0, dest = V - 1;
  uint32_t *dist = malloc(V * V * sizeof(uint32_t));
  floyd_warshall(V, adj, dist);
  uint32_t ref = dist[src * V + dest];
  printf("Reference: dist[%u->%u] = %u\n", src, dest, ref);

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
    free(hvm_src); free(rp); free(ci); free(wt); free(adj); free(dist);
    return 1;
  }

  printf("HVM4 result: %u\n", result);

  int ok = (count >= 1 && result == ref);
  if (ok) {
    printf("PASS: dist[%u->%u] = %u matches Floyd-Warshall\n", src, dest, result);
  } else {
    printf("FAIL: got %u (count=%d), expected %u\n", result, count, ref);
  }

  free(hvm_src); free(rp); free(ci); free(wt); free(adj); free(dist);
  hvm4_lib_cleanup();
  return ok ? 0 : 1;
}
