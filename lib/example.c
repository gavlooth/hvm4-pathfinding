/**
 * example.c - Demonstration of libhvm4_graph usage
 * 
 * Builds a simple graph and runs all available algorithms.
 */

#include "libhvm4_graph.h"
#include <stdio.h>
#include <stdlib.h>

static void print_error(const char *func, hvm4_result_t result) {
    const char *msg = "Unknown error";
    switch (result) {
        case HVM4_OK: msg = "Success"; break;
        case HVM4_ERR_INVALID_PARAM: msg = "Invalid parameter"; break;
        case HVM4_ERR_ALLOC: msg = "Allocation failed"; break;
        case HVM4_ERR_HVM4_RUNTIME: msg = "HVM4 runtime error"; break;
        case HVM4_ERR_NO_PATH: msg = "No path found"; break;
    }
    fprintf(stderr, "%s failed: %s\n", func, msg);
}

int main(void) {
    hvm4_result_t result;
    
    printf("=== libhvm4_graph Example ===\n\n");
    
    // Initialize HVM4 runtime
    printf("Initializing HVM4 runtime...\n");
    result = hvm4_init();
    if (result != HVM4_OK) {
        print_error("hvm4_init", result);
        return 1;
    }
    
    // Create a simple graph (6 nodes)
    // Graph structure:
    //   0 --2--> 1 --1--> 2
    //   |        |        |
    //   3        2        1
    //   |        |        |
    //   v        v        v
    //   3 --1--> 4 --3--> 5
    printf("Creating graph with 6 nodes...\n");
    hvm4_graph_t *g = hvm4_graph_new(6);
    if (!g) {
        fprintf(stderr, "Failed to create graph\n");
        hvm4_cleanup();
        return 1;
    }
    
    // Add edges
    hvm4_graph_add_edge(g, 0, 1, 2);
    hvm4_graph_add_edge(g, 0, 3, 3);
    hvm4_graph_add_edge(g, 1, 2, 1);
    hvm4_graph_add_edge(g, 1, 4, 2);
    hvm4_graph_add_edge(g, 2, 5, 1);
    hvm4_graph_add_edge(g, 3, 4, 1);
    hvm4_graph_add_edge(g, 4, 5, 3);
    
    printf("  Nodes: 6\n");
    printf("  Edges: 7 (directed)\n\n");
    
    // ===================================================================
    // 1. Transitive Closure
    // ===================================================================
    printf("--- Test 1: Transitive Closure ---\n");
    uint8_t *closure_matrix = malloc(6 * 6 * sizeof(uint8_t));
    if (!closure_matrix) {
        fprintf(stderr, "Allocation failed\n");
        hvm4_graph_free(g);
        hvm4_cleanup();
        return 1;
    }
    
    result = hvm4_closure(g, 6, closure_matrix);
    if (result != HVM4_OK) {
        print_error("hvm4_closure", result);
    } else {
        printf("Closure matrix (can i reach j?):\n");
        printf("  ");
        for (int j = 0; j < 6; j++) printf("%d ", j);
        printf("\n");
        for (int i = 0; i < 6; i++) {
            printf("%d ", i);
            for (int j = 0; j < 6; j++) {
                printf("%d ", closure_matrix[i * 6 + j]);
            }
            printf("\n");
        }
    }
    free(closure_matrix);
    printf("\n");
    
    // ===================================================================
    // 2. Shortest Path (SSSP from node 0)
    // ===================================================================
    printf("--- Test 2: Shortest Paths from node 0 ---\n");
    uint32_t distances[6];
    
    result = hvm4_shortest_path(g, 0, distances);
    if (result != HVM4_OK) {
        print_error("hvm4_shortest_path", result);
    } else {
        printf("Distances from node 0:\n");
        for (int i = 0; i < 6; i++) {
            if (distances[i] >= 999999) {
                printf("  0 -> %d: unreachable\n", i);
            } else {
                printf("  0 -> %d: %u\n", i, distances[i]);
            }
        }
    }
    printf("\n");
    
    // ===================================================================
    // 3. Point-to-Point Reachability
    // ===================================================================
    printf("--- Test 3: Point-to-Point Reachability ---\n");
    uint32_t dist;
    
    result = hvm4_reachable(g, 0, 5, 10, &dist);
    if (result == HVM4_OK) {
        printf("  0 can reach 5 (distance: %u)\n", dist);
    } else if (result == HVM4_ERR_NO_PATH) {
        printf("  0 cannot reach 5\n");
    } else {
        print_error("hvm4_reachable", result);
    }
    
    result = hvm4_reachable(g, 5, 0, 10, &dist);
    if (result == HVM4_OK) {
        printf("  5 can reach 0 (distance: %u)\n", dist);
    } else if (result == HVM4_ERR_NO_PATH) {
        printf("  5 cannot reach 0\n");
    } else {
        print_error("hvm4_reachable", result);
    }
    printf("\n");
    
    // ===================================================================
    // 4. MST (Borůvka)
    // ===================================================================
    printf("--- Test 4: Minimum Spanning Tree (Borůvka) ---\n");
    printf("Creating undirected graph for MST...\n");
    
    hvm4_graph_t *g_undir = hvm4_graph_new(4);
    if (!g_undir) {
        fprintf(stderr, "Failed to create undirected graph\n");
        hvm4_graph_free(g);
        hvm4_cleanup();
        return 1;
    }
    
    // Classic MST example: 4 nodes, 5 edges
    // 0-1: 4, 0-2: 1, 1-2: 2, 1-3: 5, 2-3: 3
    // Expected MST: 0-2 (1) + 1-2 (2) + 2-3 (3) = 6
    hvm4_graph_add_biedge(g_undir, 0, 1, 4);
    hvm4_graph_add_biedge(g_undir, 0, 2, 1);
    hvm4_graph_add_biedge(g_undir, 1, 2, 2);
    hvm4_graph_add_biedge(g_undir, 1, 3, 5);
    hvm4_graph_add_biedge(g_undir, 2, 3, 3);
    
    uint32_t mst_weight;
    uint32_t boruvka_rounds = 2; // ceil(log2(4)) = 2
    
    result = hvm4_mst_boruvka(g_undir, boruvka_rounds, &mst_weight);
    if (result != HVM4_OK) {
        print_error("hvm4_mst_boruvka", result);
    } else {
        printf("MST total weight: %u (expected: 6)\n", mst_weight);
    }
    
    hvm4_graph_free(g_undir);
    printf("\n");
    
    // Cleanup
    hvm4_graph_free(g);
    hvm4_cleanup();
    
    printf("=== All tests complete ===\n");
    return 0;
}
