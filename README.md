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
```

## Dependencies

[HVM4](https://github.com/HigherOrderCO/HVM4) — included as a git submodule.
