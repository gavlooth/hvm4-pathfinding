# libhvm4_graph Usage Guide

## Installation

### Option 1: Static Linking (Recommended)
```bash
gcc -O3 -o myapp myapp.c -I./include -L./lib -l:libhvm4_graph.a -lm -pthread
```

### Option 2: Dynamic Linking
```bash
gcc -O3 -o myapp myapp.c -I./include -L./lib -lhvm4_graph -lm -pthread

# Set library path before running:
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
./myapp
```

### Option 3: System Install
```bash
sudo make install
# Then just:
gcc -O3 -o myapp myapp.c -lhvm4_graph -lm -pthread
```

---

## Quick Start

```c
#include "libhvm4_graph.h"

int main() {
    // Initialize runtime
    hvm4_graph_ctx *ctx = hvm4_graph_init(NULL);
    
    // Create graph with 4 nodes
    hvm4_graph *g = hvm4_graph_create(ctx, 4);
    
    // Add directed edges: (src, dst, weight)
    hvm4_graph_add_edge(g, 0, 1, 2);
    hvm4_graph_add_edge(g, 0, 2, 3);
    hvm4_graph_add_edge(g, 1, 3, 1);
    hvm4_graph_add_edge(g, 2, 3, 1);
    
    // Run algorithms
    int *dist = hvm4_shortest_path(ctx, g, 0);   // SSSP from node 0
    int mst = hvm4_mst(ctx, g);                   // MST weight
    bool *closure = hvm4_closure(ctx, g);         // Reachability matrix
    bool can_reach = hvm4_reachable(ctx, g, 0, 3); // Point query
    
    // Results
    printf("Distance 0->3: %d\n", dist[3]);       // 3
    printf("MST weight: %d\n", mst);              // 4
    printf("0 can reach 3: %s\n", can_reach ? "yes" : "no");
    
    // Cleanup
    free(dist);
    free(closure);
    hvm4_graph_free(g);
    hvm4_graph_cleanup(ctx);
    return 0;
}
```

---

## API Reference

### Context Management

```c
// Initialize HVM4 runtime
// config: NULL for defaults, or JSON config string
hvm4_graph_ctx* hvm4_graph_init(const char *config);

// Cleanup and free all resources
void hvm4_graph_cleanup(hvm4_graph_ctx *ctx);
```

### Graph Construction

```c
// Create empty graph with n nodes (0 to n-1)
hvm4_graph* hvm4_graph_create(hvm4_graph_ctx *ctx, int num_nodes);

// Add directed edge with weight
void hvm4_graph_add_edge(hvm4_graph *g, int src, int dst, int weight);

// Add undirected edge (adds both directions)
void hvm4_graph_add_undirected_edge(hvm4_graph *g, int u, int v, int weight);

// Free graph memory
void hvm4_graph_free(hvm4_graph *g);
```

### Algorithms

```c
// Single-source shortest paths (Bellman-Ford)
// Returns: array of n distances (caller must free)
// dist[i] = shortest distance from source to i, or INT_MAX if unreachable
int* hvm4_shortest_path(hvm4_graph_ctx *ctx, hvm4_graph *g, int source);

// Minimum Spanning Tree (Borůvka's algorithm)
// Returns: total weight of MST
int hvm4_mst(hvm4_graph_ctx *ctx, hvm4_graph *g);

// Transitive closure
// Returns: n*n boolean array (caller must free)
// closure[i*n + j] = true if node i can reach node j
bool* hvm4_closure(hvm4_graph_ctx *ctx, hvm4_graph *g);

// Point-to-point reachability
// Returns: true if src can reach dst
bool hvm4_reachable(hvm4_graph_ctx *ctx, hvm4_graph *g, int src, int dst);
```

---

## Performance Characteristics

| Algorithm | Time Complexity | Space | Notes |
|-----------|----------------|-------|-------|
| Shortest Path | O(V × E) | O(V) | Bellman-Ford with tree-structured distance arrays |
| MST | O(E log V) | O(V) | Borůvka with parallel component merging |
| Closure | O(V²) | O(V²) | Parallel DFS from each node |
| Reachability | O(E) | O(V) | Bidirectional BFS |

### Scaling

- **Up to 1k nodes**: Instant (<10ms)
- **1k-10k nodes**: Fast (10-100ms)  
- **10k+ nodes**: Set `HVM4_THREADS=4` environment variable

The library uses tree-structured internal representations for O(log n) parallel reduction depth.

---

## Thread Safety

- `hvm4_graph_ctx`: NOT thread-safe. Use one per thread or protect with mutex.
- `hvm4_graph`: Read-only sharing is safe after construction.
- Algorithm results: Owned by caller, safe to use in any thread.

---

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `HVM4_THREADS` | 1 | Number of parallel threads |
| `HVM4_HEAP_MB` | 256 | Heap size in megabytes |
| `HVM4_DEBUG` | 0 | Enable debug output |

---

## Error Handling

Functions return NULL or -1 on error. Check `hvm4_graph_error(ctx)` for details:

```c
int *dist = hvm4_shortest_path(ctx, g, 0);
if (dist == NULL) {
    fprintf(stderr, "Error: %s\n", hvm4_graph_error(ctx));
    exit(1);
}
```

---

## Building from Source

```bash
cd lib
make clean all    # Build libraries and examples
make test         # Run example program
make bench        # Run benchmarks
make dist         # Create distribution package
```

### Requirements
- GCC 7+ or Clang 6+
- pthreads
- x86_64 architecture

---

## Example Programs

See `examples/` directory:
- `example.c` - Demonstrates all APIs
- `benchmark.c` - Performance measurements

Build and run:
```bash
cd examples
gcc -O3 -o example example.c -I../include -L../lib -l:libhvm4_graph.a -lm -pthread
./example
```
