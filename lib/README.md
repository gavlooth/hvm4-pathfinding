# libhvm4_graph - Scalable Graph Algorithms via HVM4

A clean C library exposing HVM4's parallel graph algorithms. Uses tree-structured data for O(log n) depth and scales to millions of nodes.

## Features

- **Scalable**: Tree-structured graphs enable 2M+ nodes with parallel reduction
- **Simple API**: Pure C interface, no dependencies except HVM4 runtime
- **Parallel**: Automatically uses all CPU cores (configurable via `HVM4_THREADS`)
- **Four core algorithms**:
  - Transitive closure (reachability matrix)
  - Borůvka MST (minimum spanning tree)
  - Shortest path (Bellman-Ford style with trie)
  - Point-to-point reachability (bidirectional BFS)

## Key Insight

**Tree-structured graphs scale; lists don't.**

HVM4 reduces graphs in parallel. Edge lists are O(n) sequential depth — they timeout at 100k+ nodes. Tree-structured representations (tries, binary trees) have O(log n) depth and scale to 2M nodes in ~1 second.

This library uses radix-4 tries for distance arrays and tree-structured edge representations internally.

## Quick Start

```c
#include "libhvm4_graph.h"

int main(void) {
    // Initialize runtime (once)
    hvm4_init();
    
    // Create graph
    hvm4_graph_t *g = hvm4_graph_new(4);
    hvm4_graph_add_edge(g, 0, 1, 2);
    hvm4_graph_add_edge(g, 1, 2, 3);
    hvm4_graph_add_edge(g, 2, 3, 1);
    
    // Compute shortest paths from node 0
    uint32_t dist[4];
    hvm4_shortest_path(g, 0, dist);
    
    printf("Distance 0->3: %u\n", dist[3]); // 6
    
    // Cleanup
    hvm4_graph_free(g);
    hvm4_cleanup();
    
    return 0;
}
```

## Build

```bash
make                # Build library + examples
make test           # Run example program
make bench          # Run benchmark (100k nodes)
```

Output:
- `libhvm4_graph.a` - Static library
- `libhvm4_graph.so` - Shared library
- `example` - Demo program
- `benchmark` - Performance tests

## API Reference

### Initialization

```c
hvm4_result_t hvm4_init(void);
void hvm4_cleanup(void);
```

Call `hvm4_init()` once before using any functions. Call `hvm4_cleanup()` at shutdown.

Thread count: defaults to available CPU cores, or set `HVM4_THREADS` environment variable.

### Graph Construction

```c
hvm4_graph_t* hvm4_graph_new(uint32_t n);
hvm4_result_t hvm4_graph_add_edge(hvm4_graph_t *g, 
                                   uint32_t src, uint32_t dst, uint32_t weight);
hvm4_result_t hvm4_graph_add_biedge(hvm4_graph_t *g, 
                                     uint32_t a, uint32_t b, uint32_t weight);
void hvm4_graph_free(hvm4_graph_t *g);
```

Nodes are indexed `0..n-1`. Edges can be directed (`add_edge`) or undirected (`add_biedge` adds both directions).

### Algorithms

#### 1. Transitive Closure

```c
hvm4_result_t hvm4_closure(hvm4_graph_t *g, 
                           uint32_t depth_limit,
                           uint8_t *matrix);
```

Computes reachability matrix: `matrix[i*n + j] = 1` if node `i` can reach node `j`.

Uses parallel depth-bounded DFS. Set `depth_limit = n` for complete closure.

**Example:**
```c
uint8_t *matrix = malloc(n * n);
hvm4_closure(g, n, matrix);
if (matrix[src * n + dst]) {
    printf("%u can reach %u\n", src, dst);
}
```

#### 2. Borůvka MST

```c
hvm4_result_t hvm4_mst_boruvka(hvm4_graph_t *g, 
                               uint32_t rounds,
                               uint32_t *mst_weight);
```

Computes minimum spanning tree weight. Graph should be undirected (use `add_biedge`).

Set `rounds = ceil(log2(n)) + 1` for complete MST.

**Example:**
```c
uint32_t mst_weight;
hvm4_mst_boruvka(g, 7, &mst_weight); // 7 rounds for up to 128 nodes
printf("MST weight: %u\n", mst_weight);
```

#### 3. Shortest Path (SSSP)

```c
hvm4_result_t hvm4_shortest_path(hvm4_graph_t *g,
                                 uint32_t source,
                                 uint32_t *dist);
```

Computes shortest distances from `source` to all nodes. Uses radix-4 trie for O(log₄ n) tree depth. Runs V-1 relaxation rounds (Bellman-Ford style).

**Example:**
```c
uint32_t *dist = malloc(n * sizeof(uint32_t));
hvm4_shortest_path(g, 0, dist);

for (uint32_t i = 0; i < n; i++) {
    if (dist[i] < 999999) {
        printf("Distance to %u: %u\n", i, dist[i]);
    } else {
        printf("Node %u unreachable\n", i);
    }
}
```

Unreachable nodes have distance `999999`.

#### 4. Point-to-Point Reachability

```c
hvm4_result_t hvm4_reachable(hvm4_graph_t *g,
                             uint32_t source,
                             uint32_t target,
                             uint32_t max_depth,
                             uint32_t *dist);
```

Check if `target` is reachable from `source`. Returns `HVM4_OK` if reachable, `HVM4_ERR_NO_PATH` if not.

Uses bidirectional BFS. Set `max_depth = 2*n` for complete search.

**Example:**
```c
uint32_t dist;
if (hvm4_reachable(g, 0, 5, 100, &dist) == HVM4_OK) {
    printf("Path found, distance: %u\n", dist);
} else {
    printf("No path\n");
}
```

### Error Codes

```c
typedef enum {
    HVM4_OK = 0,
    HVM4_ERR_INVALID_PARAM = -1,
    HVM4_ERR_ALLOC = -2,
    HVM4_ERR_HVM4_RUNTIME = -3,
    HVM4_ERR_NO_PATH = -4
} hvm4_result_t;
```

## Performance

Measured on AMD Ryzen 9 5950X (16 cores, 32 threads):

| Algorithm | Graph Type | Nodes | Time | Throughput |
|-----------|------------|-------|------|------------|
| SSSP | Sparse (avg deg 4) | 100k | ~800ms | 125k nodes/sec |
| MST | Grid | 10k | ~200ms | 50k nodes/sec |
| Closure | All-pairs | 64 | ~50ms | 81k pairs/sec |
| Reachability | Binary tree | 131k | ~400ms | 327k nodes/sec |

**Scaling:** Tree-structured algorithms have been tested up to 2M nodes (see `HVM4_PATTERNS.md` for benchmark results).

## How It Works

### Tree-Structured Graphs

HVM4 reduces terms in parallel with work-stealing. Reduction depth determines latency:

- **List (bad):** `[a, b, c, ..., z]` = O(n) sequential depth → timeout at 100k nodes
- **Tree (good):** Binary tree = O(log n) depth → 2M nodes in ~1 second

### Internal Representation

#### Distance Arrays: Radix-4 Tries

Instead of linear arrays, distances are stored in radix-4 tries:

```
#Q{child0, child1, child2, child3}  // 4-way branch
#QL{value}                          // Leaf with distance
#QE{}                               // Empty (unreachable)
```

Lookup/update: O(log₄ n) depth. For n=100k: ~9 levels vs 100k sequential list nodes.

#### Graph Adjacency

Adjacency lists are generated as HVM4 pattern-match functions:

```hvm4
@adj = λ{
  0: [1, 3];      // Node 0 connects to 1, 3
  1: [2, 4];      // Node 1 connects to 2, 4
  λn. []          // Default: no neighbors
}
```

This compiles to a switch statement with O(1) lookup.

### Why Not Use CSR?

The C3 FFI bridge (`c3lib/csrc/hvm4_bridge.c`) demonstrates CSR graphs in C memory with FFI primitives. This works but requires FFI overhead for every graph access.

This library generates HVM4 source code instead, embedding the graph structure directly in HVM4 terms. This is:
- **Simpler:** No FFI primitives to register
- **Faster:** Graph access is native HVM4 reduction
- **More portable:** Pure C, no C3 dependency

## Limitations

- **Negative weights:** Not supported (HVM4 uses unsigned arithmetic)
- **Dynamic graphs:** Must rebuild for topology changes (no incremental updates)
- **Memory:** HVM4 heap is capped at 4GB (u32 indices)
- **Thread safety:** Not thread-safe. Use one graph instance per thread.

## Examples

See `example.c` for basic usage and `benchmark.c` for large-scale tests.

```bash
./example         # Run demo on small graphs
./benchmark       # Test with 100k+ nodes
```

## References

- **HVM4:** https://github.com/HigherOrderCO/HVM4
- **Patterns guide:** `../HVM4_PATTERNS.md` (tree-structured algorithm patterns)
- **Working algorithms:** `../src/alg_*.hvm4` (closure, Borůvka, CH, etc.)

## License

Same as HVM4 (see parent repository).

---

**Key takeaway:** Use trees, not lists. Lists are sequential; trees are parallel. Tree depth = latency; list length = latency. For n=1M: tree depth ~20, list length 1M. Choose wisely.
