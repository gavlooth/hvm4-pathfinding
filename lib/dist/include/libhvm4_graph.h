/**
 * libhvm4_graph.h - HVM4 Graph Algorithm Library
 * 
 * A clean C interface for running scalable graph algorithms using HVM4.
 * All algorithms use tree-structured data for O(log n) depth parallelism.
 * 
 * Key features:
 * - Scales to 2M+ nodes using tree-structured HVM4 graphs
 * - Parallel reduction (use HVM4_THREADS env var)
 * - Closure (reachability), MST, shortest paths
 * 
 * Thread safety: Not thread-safe. Use one instance per thread.
 */

#ifndef LIBHVM4_GRAPH_H
#define LIBHVM4_GRAPH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Types
 * ======================================================================== */

/**
 * Graph edge with source, destination, and weight.
 */
typedef struct {
    uint32_t src;
    uint32_t dst;
    uint32_t weight;
} hvm4_edge_t;

/**
 * Result codes
 */
typedef enum {
    HVM4_OK = 0,
    HVM4_ERR_INVALID_PARAM = -1,
    HVM4_ERR_ALLOC = -2,
    HVM4_ERR_HVM4_RUNTIME = -3,
    HVM4_ERR_NO_PATH = -4
} hvm4_result_t;

/**
 * Graph handle (opaque)
 */
typedef struct hvm4_graph hvm4_graph_t;

/* ========================================================================
 * Initialization & Cleanup
 * ======================================================================== */

/**
 * Initialize the HVM4 runtime.
 * Call once before using any other functions.
 * 
 * Thread count: defaults to available cores, or set HVM4_THREADS env var.
 * 
 * Returns HVM4_OK on success.
 */
hvm4_result_t hvm4_init(void);

/**
 * Cleanup and free all HVM4 runtime resources.
 * Call once at shutdown.
 */
void hvm4_cleanup(void);

/* ========================================================================
 * Graph Construction
 * ======================================================================== */

/**
 * Create a new graph with n nodes (indexed 0..n-1).
 * 
 * @param n  Number of nodes
 * @return   Graph handle, or NULL on allocation failure
 */
hvm4_graph_t* hvm4_graph_new(uint32_t n);

/**
 * Add a directed edge to the graph.
 * 
 * @param g       Graph handle
 * @param src     Source node (0..n-1)
 * @param dst     Destination node (0..n-1)
 * @param weight  Edge weight
 * @return        HVM4_OK or error code
 */
hvm4_result_t hvm4_graph_add_edge(hvm4_graph_t *g, 
                                   uint32_t src, 
                                   uint32_t dst, 
                                   uint32_t weight);

/**
 * Add an undirected edge (creates two directed edges).
 * 
 * @param g       Graph handle
 * @param a       First node
 * @param b       Second node
 * @param weight  Edge weight
 * @return        HVM4_OK or error code
 */
hvm4_result_t hvm4_graph_add_biedge(hvm4_graph_t *g, 
                                     uint32_t a, 
                                     uint32_t b, 
                                     uint32_t weight);

/**
 * Destroy graph and free memory.
 * 
 * @param g  Graph handle
 */
void hvm4_graph_free(hvm4_graph_t *g);

/* ========================================================================
 * Algorithm: Transitive Closure (Reachability Matrix)
 * ======================================================================== */

/**
 * Compute transitive closure: can node i reach node j?
 * 
 * Uses parallel depth-bounded DFS. Scales to 2M nodes.
 * 
 * @param g            Graph handle
 * @param depth_limit  DFS depth limit (use n for complete closure)
 * @param[out] matrix  Output matrix (n*n booleans, row-major)
 *                     Caller must allocate (n*n) uint8_t array.
 *                     matrix[i*n + j] = 1 if i can reach j, else 0.
 * @return             HVM4_OK or error code
 */
hvm4_result_t hvm4_closure(hvm4_graph_t *g, 
                           uint32_t depth_limit,
                           uint8_t *matrix);

/* ========================================================================
 * Algorithm: Borůvka MST
 * ======================================================================== */

/**
 * Compute minimum spanning tree total weight using Borůvka's algorithm.
 * 
 * Graph should be undirected (use add_biedge). Runs O(log n) rounds.
 * Scales to 2M nodes.
 * 
 * @param g              Graph handle
 * @param rounds         Number of Borůvka rounds (use log2(n) + 1)
 * @param[out] mst_weight  Total weight of MST
 * @return               HVM4_OK or error code
 */
hvm4_result_t hvm4_mst_boruvka(hvm4_graph_t *g, 
                               uint32_t rounds,
                               uint32_t *mst_weight);

/* ========================================================================
 * Algorithm: Single-Source Shortest Path (Bellman-Ford style)
 * ======================================================================== */

/**
 * Compute shortest distances from a source node.
 * 
 * Uses radix-4 trie for O(log_4 n) tree depth. Runs V-1 relaxation rounds.
 * Scales to 2M nodes.
 * 
 * @param g           Graph handle
 * @param source      Source node
 * @param[out] dist   Output distance array (size n)
 *                    Caller must allocate uint32_t[n].
 *                    dist[i] = distance from source to i, or 999999 if unreachable.
 * @return            HVM4_OK or error code
 */
hvm4_result_t hvm4_shortest_path(hvm4_graph_t *g,
                                 uint32_t source,
                                 uint32_t *dist);

/* ========================================================================
 * Algorithm: Point-to-Point Reachability
 * ======================================================================== */

/**
 * Check if target is reachable from source.
 * 
 * Returns distance if reachable (may be approximate for weighted graphs),
 * or HVM4_ERR_NO_PATH if unreachable.
 * 
 * @param g         Graph handle
 * @param source    Source node
 * @param target    Target node
 * @param max_depth Maximum search depth
 * @param[out] dist Distance found (0 if source==target)
 * @return          HVM4_OK if reachable, HVM4_ERR_NO_PATH if not, or error
 */
hvm4_result_t hvm4_reachable(hvm4_graph_t *g,
                             uint32_t source,
                             uint32_t target,
                             uint32_t max_depth,
                             uint32_t *dist);

#ifdef __cplusplus
}
#endif

#endif /* LIBHVM4_GRAPH_H */
