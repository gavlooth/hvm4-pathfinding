# hvm4-pathfinding

Six pathfinding algorithms implemented in HVM4's interaction calculus.

## Algorithms

| Algorithm | Type | Graph | Method |
|-----------|------|-------|--------|
| Algebraic (tropical semiring) | APSP | 3 nodes, undirected, weighted | Matrix squaring over (min, +) semiring |
| Bellman-Ford | SSSP | 5 nodes, directed, weighted | V-1 rounds of edge relaxation |
| Bidirectional BFS | point-to-point | 7 nodes, undirected, unweighted | Alternating frontier expansion |
| Contraction Hierarchy | point-to-point | 6 nodes, directed, weighted | Bidirectional upward-only search |
| Delta-Stepping | SSSP | 5 nodes, directed, weighted | Light/heavy edge classification with bucket rounds |
| Superposition Enumeration | all-paths | 6 nodes, DAG, weighted | HVM4 SUP/DUP for non-deterministic branching |

## Benchmarks

Best of 5 runs on HVM4 (single-threaded, clang -O2).

| Algorithm | Interactions | Wall Time (us) | Perf (MIPS) | Peak RSS (KB) | Result |
|-----------|-------------:|----------------:|------------:|---------------:|--------|
| algebraic | 2,115 | 2,033 | 15.77 | 1,956 | `[[0,2,5],[2,0,3],[5,3,0]]` |
| bellman_ford | 43,799 | 3,096 | 35.21 | 3,044 | `[0,3,2,6,7]` |
| bidir_bfs | 223 | 2,055 | 3.85 | 1,788 | `2` |
| contraction_hierarchy | 12,076 | 2,403 | 27.73 | 2,136 | `10` |
| delta_step | 63,724 | 3,530 | 33.34 | 3,568 | `[0,1,3,5,4]` |
| sup_enum | 64 | 2,286 | 0.14 | 1,932 | `6, 7, 11, 6` |

**Key observations:**

- **Superposition enumeration** uses only 64 interactions to find all 4 paths — orders of magnitude fewer than classical approaches. The wall time is dominated by process startup (~2ms baseline).
- **Delta-stepping** has the highest interaction count (63K) due to repeated light-edge relaxation rounds, but achieves the highest throughput alongside Bellman-Ford (~33-35 MIPS).
- **Bidirectional BFS** is the most interaction-efficient classical algorithm (223 interactions) thanks to the meet-in-the-middle strategy on an unweighted graph.
- All algorithms fit comfortably in <4 MB RSS. The ~2ms baseline across all runs reflects HVM4 process startup overhead.

Run benchmarks yourself:

```bash
./bench.sh        # 5 runs (default)
./bench.sh 10     # 10 runs
```

## Setup

```bash
git clone --recurse-submodules https://github.com/gavlooth/hvm4-pathfinding.git
cd hvm4-pathfinding
./build.sh
```

## Usage

```bash
# Run a specific algorithm
./run.sh bellman_ford

# Run all algorithms
./run.sh all

# Run with collapse mode (for sup_enum)
./run.sh sup_enum -C10

# Run tests
./test.sh

# Run benchmarks
./bench.sh
```

## Dependencies

[HVM4](https://github.com/HigherOrderCO/HVM4) — included as a git submodule.
