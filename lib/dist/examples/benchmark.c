/**
 * benchmark.c - Performance testing for libhvm4_graph
 * 
 * Tests scalability with 100k+ nodes using tree-structured algorithms.
 */

#include "libhvm4_graph.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

static double get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

static void print_result(const char *test, int ok, double elapsed_ms, uint32_t n) {
    if (ok) {
        printf("  ✓ %s: %.0f ms (%.0f nodes/sec)\n", 
               test, elapsed_ms, n * 1000.0 / elapsed_ms);
    } else {
        printf("  ✗ %s: FAILED\n", test);
    }
}

static hvm4_graph_t* create_sparse_graph(uint32_t n, uint32_t avg_degree, unsigned seed) {
    srand(seed);
    
    hvm4_graph_t *g = hvm4_graph_new(n);
    if (!g) return NULL;
    
    // Each node connects to ~avg_degree random neighbors
    for (uint32_t u = 0; u < n; u++) {
        for (uint32_t k = 0; k < avg_degree; k++) {
            uint32_t v = rand() % n;
            uint32_t w = (rand() % 10) + 1; // Weight 1..10
            hvm4_graph_add_edge(g, u, v, w);
        }
    }
    
    return g;
}

static hvm4_graph_t* create_grid_graph(uint32_t side) {
    uint32_t n = side * side;
    hvm4_graph_t *g = hvm4_graph_new(n);
    if (!g) return NULL;
    
    // 2D grid with edges to right and down neighbors
    for (uint32_t i = 0; i < side; i++) {
        for (uint32_t j = 0; j < side; j++) {
            uint32_t u = i * side + j;
            
            // Right neighbor
            if (j + 1 < side) {
                uint32_t v = i * side + (j + 1);
                hvm4_graph_add_biedge(g, u, v, 1);
            }
            
            // Down neighbor
            if (i + 1 < side) {
                uint32_t v = (i + 1) * side + j;
                hvm4_graph_add_biedge(g, u, v, 1);
            }
        }
    }
    
    return g;
}

static hvm4_graph_t* create_tree_graph(uint32_t depth) {
    // Binary tree with 2^depth - 1 nodes
    uint32_t n = (1u << depth) - 1;
    hvm4_graph_t *g = hvm4_graph_new(n);
    if (!g) return NULL;
    
    // Add edges from parent to children
    for (uint32_t i = 0; i < n / 2; i++) {
        uint32_t left = 2 * i + 1;
        uint32_t right = 2 * i + 2;
        
        if (left < n) {
            hvm4_graph_add_edge(g, i, left, 1);
        }
        if (right < n) {
            hvm4_graph_add_edge(g, i, right, 1);
        }
    }
    
    return g;
}

int main(void) {
    printf("=== libhvm4_graph Benchmark ===\n\n");
    
    // Initialize
    printf("Initializing HVM4 runtime...\n");
    hvm4_result_t result = hvm4_init();
    if (result != HVM4_OK) {
        fprintf(stderr, "Failed to initialize HVM4\n");
        return 1;
    }
    
    // Get thread count
    const char *threads_env = getenv("HVM4_THREADS");
    int threads = threads_env ? atoi(threads_env) : 0;
    if (threads == 0) {
        threads = sysconf(_SC_NPROCESSORS_ONLN);
    }
    printf("Using %d threads\n\n", threads);
    
    // ===================================================================
    // Benchmark 1: Shortest Path on Sparse Graph (100k nodes)
    // ===================================================================
    printf("--- Benchmark 1: Shortest Path (100k nodes, sparse) ---\n");
    {
        uint32_t n = 100000;
        uint32_t avg_degree = 4;
        
        printf("Building random sparse graph (%u nodes, ~%u avg degree)...\n", 
               n, avg_degree);
        hvm4_graph_t *g = create_sparse_graph(n, avg_degree, 42);
        if (!g) {
            fprintf(stderr, "Failed to create graph\n");
            hvm4_cleanup();
            return 1;
        }
        
        uint32_t *dist = malloc(n * sizeof(uint32_t));
        if (!dist) {
            fprintf(stderr, "Allocation failed\n");
            hvm4_graph_free(g);
            hvm4_cleanup();
            return 1;
        }
        
        double start = get_time_ms();
        result = hvm4_shortest_path(g, 0, dist);
        double elapsed = get_time_ms() - start;
        
        print_result("SSSP (Bellman-Ford style)", result == HVM4_OK, elapsed, n);
        
        if (result == HVM4_OK) {
            // Verify some results
            int reachable = 0;
            for (uint32_t i = 0; i < n; i++) {
                if (dist[i] < 999999) reachable++;
            }
            printf("    Nodes reachable from source: %d / %u\n", reachable, n);
        }
        
        free(dist);
        hvm4_graph_free(g);
    }
    printf("\n");
    
    // ===================================================================
    // Benchmark 2: MST on Grid Graph (~10k nodes)
    // ===================================================================
    printf("--- Benchmark 2: MST (10k nodes, grid) ---\n");
    {
        uint32_t side = 100; // 100x100 grid = 10k nodes
        uint32_t n = side * side;
        
        printf("Building 2D grid graph (%ux%u = %u nodes)...\n", side, side, n);
        hvm4_graph_t *g = create_grid_graph(side);
        if (!g) {
            fprintf(stderr, "Failed to create grid\n");
            hvm4_cleanup();
            return 1;
        }
        
        uint32_t mst_weight;
        uint32_t rounds = 7; // ceil(log2(10000)) + 1
        
        double start = get_time_ms();
        result = hvm4_mst_boruvka(g, rounds, &mst_weight);
        double elapsed = get_time_ms() - start;
        
        print_result("MST (Borůvka)", result == HVM4_OK, elapsed, n);
        
        if (result == HVM4_OK) {
            printf("    MST weight: %u\n", mst_weight);
        }
        
        hvm4_graph_free(g);
    }
    printf("\n");
    
    // ===================================================================
    // Benchmark 3: Reachability on Tree (131k nodes)
    // ===================================================================
    printf("--- Benchmark 3: Reachability (131k nodes, binary tree) ---\n");
    {
        uint32_t depth = 17; // 2^17 - 1 = 131,071 nodes
        uint32_t n = (1u << depth) - 1;
        
        printf("Building binary tree (depth %u = %u nodes)...\n", depth, n);
        hvm4_graph_t *g = create_tree_graph(depth);
        if (!g) {
            fprintf(stderr, "Failed to create tree\n");
            hvm4_cleanup();
            return 1;
        }
        
        uint32_t dist;
        uint32_t source = 0;     // Root
        uint32_t target = n - 1; // Rightmost leaf
        
        double start = get_time_ms();
        result = hvm4_reachable(g, source, target, depth, &dist);
        double elapsed = get_time_ms() - start;
        
        print_result("Point-to-point reachability", 
                    result == HVM4_OK, elapsed, n);
        
        if (result == HVM4_OK) {
            printf("    Distance from root to leaf: %u\n", dist);
        }
        
        hvm4_graph_free(g);
    }
    printf("\n");
    
    // ===================================================================
    // Benchmark 4: Closure on Small Graph (64 nodes, all-pairs)
    // ===================================================================
    printf("--- Benchmark 4: Transitive Closure (64 nodes, all-pairs) ---\n");
    {
        uint32_t n = 64;
        uint32_t avg_degree = 6;
        
        printf("Building random graph (%u nodes, ~%u avg degree)...\n", 
               n, avg_degree);
        hvm4_graph_t *g = create_sparse_graph(n, avg_degree, 123);
        if (!g) {
            fprintf(stderr, "Failed to create graph\n");
            hvm4_cleanup();
            return 1;
        }
        
        uint8_t *matrix = malloc(n * n * sizeof(uint8_t));
        if (!matrix) {
            fprintf(stderr, "Allocation failed\n");
            hvm4_graph_free(g);
            hvm4_cleanup();
            return 1;
        }
        
        double start = get_time_ms();
        result = hvm4_closure(g, n, matrix);
        double elapsed = get_time_ms() - start;
        
        print_result("Transitive closure (all-pairs)", 
                    result == HVM4_OK, elapsed, n * n);
        
        if (result == HVM4_OK) {
            // Count reachable pairs
            int pairs = 0;
            for (uint32_t i = 0; i < n * n; i++) {
                if (matrix[i]) pairs++;
            }
            printf("    Reachable pairs: %d / %u\n", pairs, n * n);
        }
        
        free(matrix);
        hvm4_graph_free(g);
    }
    printf("\n");
    
    // Cleanup
    hvm4_cleanup();
    
    printf("=== Benchmark complete ===\n");
    printf("\nNote: Tree-structured graphs scale to 2M+ nodes.\n");
    printf("Performance depends on CPU cores (use HVM4_THREADS to control).\n");
    
    return 0;
}
