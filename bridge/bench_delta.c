#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct VineState VineState;
extern VineState* vine_init(const char* lib_path, const char* main_path, const char* root_path);
extern void vine_destroy(VineState* state);
extern int vine_route_graph(
    const VineState* state,
    unsigned node_count,
    const unsigned* row_ptr,
    const unsigned* col_idx,
    const unsigned* weights,
    unsigned source,
    unsigned target,
    unsigned algorithm,
    unsigned workers,
    unsigned char* out_buf,
    unsigned out_len,
    unsigned* out_written);

// Generate a random grid graph: rows x cols with 4-connected neighbors
// Mimics H3 hex grid structure (regular, degree ~4-6)
static void gen_grid(unsigned rows, unsigned cols,
                     unsigned** out_row_ptr, unsigned** out_col_idx, unsigned** out_weights,
                     unsigned* out_n, unsigned* out_e) {
    unsigned n = rows * cols;
    unsigned max_edges = n * 4;  // at most 4 neighbors per node
    unsigned* row_ptr = calloc(n + 1, sizeof(unsigned));
    unsigned* col_idx = malloc(max_edges * sizeof(unsigned));
    unsigned* weights = malloc(max_edges * sizeof(unsigned));

    unsigned ei = 0;
    for (unsigned r = 0; r < rows; r++) {
        for (unsigned c = 0; c < cols; c++) {
            unsigned u = r * cols + c;
            row_ptr[u] = ei;
            // right
            if (c + 1 < cols) {
                col_idx[ei] = u + 1;
                weights[ei] = 800 + (rand() % 400);  // 800-1200 (mimics ~1km H3 edges)
                ei++;
            }
            // left
            if (c > 0) {
                col_idx[ei] = u - 1;
                weights[ei] = 800 + (rand() % 400);
                ei++;
            }
            // down
            if (r + 1 < rows) {
                col_idx[ei] = u + cols;
                weights[ei] = 800 + (rand() % 400);
                ei++;
            }
            // up
            if (r > 0) {
                col_idx[ei] = u - cols;
                weights[ei] = 800 + (rand() % 400);
                ei++;
            }
        }
    }
    row_ptr[n] = ei;

    *out_row_ptr = row_ptr;
    *out_col_idx = col_idx;
    *out_weights = weights;
    *out_n = n;
    *out_e = ei;
}

static double elapsed_ms(struct timespec* start) {
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &end);
    return (end.tv_sec - start->tv_sec) * 1000.0 + (end.tv_nsec - start->tv_nsec) / 1e6;
}

int main(int argc, char** argv) {
    unsigned rows = 290, cols = 290;  // ~84k nodes (like H3 grid)
    if (argc >= 3) {
        rows = atoi(argv[1]);
        cols = atoi(argv[2]);
    }

    srand(42);

    unsigned n, e;
    unsigned *row_ptr, *col_idx, *weights;
    gen_grid(rows, cols, &row_ptr, &col_idx, &weights, &n, &e);
    printf("Graph: %u nodes, %u edges (avg degree %.1f)\n", n, e, (double)e / n);

    printf("Initializing Vine...\n");
    VineState* state = vine_init("vine/pathfind", "vine/main_ffi.vi", NULL);
    if (!state) { fprintf(stderr, "vine_init failed\n"); return 1; }

    unsigned src = 0, tgt = n - 1;  // top-left to bottom-right
    unsigned char out_buf[1048576];
    unsigned out_written;
    struct timespec t0;

    // Benchmark Dijkstra
    for (unsigned w = 1; w <= 4; w *= 4) {
        clock_gettime(CLOCK_MONOTONIC, &t0);
        int rc = vine_route_graph(state, n, row_ptr, col_idx, weights,
                                  src, tgt, 0, w, out_buf, sizeof(out_buf), &out_written);
        double ms = elapsed_ms(&t0);
        if (rc == 0) {
            // Extract cost
            char* cost_line = strstr((char*)out_buf, "COST ");
            int cost = cost_line ? atoi(cost_line + 5) : -1;
            printf("Dijkstra     (%u workers): %7.1f ms, cost=%d\n", w, ms, cost);
        } else {
            printf("Dijkstra     (%u workers): FAILED (rc=%d)\n", w, rc);
        }
    }

    // Benchmark Delta-Stepping
    for (unsigned w = 1; w <= 16; w *= 2) {
        clock_gettime(CLOCK_MONOTONIC, &t0);
        int rc = vine_route_graph(state, n, row_ptr, col_idx, weights,
                                  src, tgt, 1, w, out_buf, sizeof(out_buf), &out_written);
        double ms = elapsed_ms(&t0);
        if (rc == 0) {
            char* cost_line = strstr((char*)out_buf, "COST ");
            int cost = cost_line ? atoi(cost_line + 5) : -1;
            printf("Delta-Step   (%2u workers): %7.1f ms, cost=%d\n", w, ms, cost);
        } else {
            printf("Delta-Step   (%2u workers): FAILED (rc=%d)\n", w, rc);
        }
    }

    vine_destroy(state);
    free(row_ptr); free(col_idx); free(weights);
    return 0;
}
