#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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

// 5-node graph: expected shortest path 0→2→1→3, cost=9
// Edges: 0→1(10), 0→2(3), 1→3(2), 2→1(4), 2→3(8), 2→4(2), 4→3(5)
static const unsigned ROW_PTR[] = {0, 2, 3, 6, 6, 7};  // node 0: 2 edges, node 1: 1, node 2: 3, node 3: 0, node 4: 1
static const unsigned COL_IDX[] = {1, 2, 3, 1, 3, 4, 3};
static const unsigned WEIGHTS[] = {10, 3, 2, 4, 8, 2, 5};

static int test_graph(VineState* state, const char* name, unsigned algo, unsigned workers, int expected_cost) {
    unsigned char out_buf[4096];
    unsigned out_written = 0;

    int rc = vine_route_graph(state, 5,
        ROW_PTR, COL_IDX, WEIGHTS,
        0, 3, algo, workers,
        out_buf, sizeof(out_buf), &out_written);

    if (rc != 0) {
        printf("FAIL: %s — vine_route_graph returned %d\n", name, rc);
        return 1;
    }

    char expected[64];
    snprintf(expected, sizeof(expected), "COST %d", expected_cost);
    // Remove trailing newlines for display
    while (out_written > 0 && (out_buf[out_written-1] == '\n' || out_buf[out_written-1] == '\0'))
        out_written--;
    out_buf[out_written] = '\0';

    if (strstr((char*)out_buf, expected)) {
        printf("PASS: %-35s → %s\n", name, (char*)out_buf);
        return 0;
    } else {
        printf("FAIL: %-35s → %s (expected COST %d)\n", name, (char*)out_buf, expected_cost);
        return 1;
    }
}

int main(void) {
    printf("Initializing Vine (FFI mode)...\n");
    VineState* state = vine_init("vine/pathfind", "vine/main_ffi.vi", NULL);
    if (!state) {
        fprintf(stderr, "vine_init failed\n");
        return 1;
    }
    printf("Vine initialized.\n\n");

    int fails = 0;
    fails += test_graph(state, "Dijkstra (1 worker)", 0, 1, 9);
    fails += test_graph(state, "Dijkstra (4 workers)", 0, 4, 9);
    fails += test_graph(state, "Delta-Stepping (1 worker)", 1, 1, 9);
    fails += test_graph(state, "Delta-Stepping (4 workers)", 1, 4, 9);

    vine_destroy(state);

    printf("\n%s (%d/4 passed)\n", fails == 0 ? "ALL PASSED" : "SOME FAILED", 4 - fails);
    return fails;
}
