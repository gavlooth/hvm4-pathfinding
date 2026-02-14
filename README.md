# hvm4-pathfinding

State-of-the-art pathfinding algorithms leveraging HVM4's parallel interaction calculus.

## Design Philosophy: The Bend Pattern

**HVM4 is a VM, not a language.** These algorithms are written the way Bend/Kind would *compile* to HVM4 — think LLVM IR or JVM bytecode, not human-readable source.

### Core Principles

1. **No Variable Explosion**
   - ❌ WRONG: `let d0 = ...; let d1 = ...; ... let d16000 = ...` (one binding per node)
   - ✅ RIGHT: `@dist(n) = ... @dist(predecessors) ...` (recursive with DUP memoization)

2. **DUP = Free Memoization**
   - HVM4's DUP mechanism automatically shares work
   - Write `@dist(n)` recursively, let HVM4 handle caching
   - No manual memoization needed

3. **Trees Scale, Lists Don't**
   - Lists: Sequential reduction `(0+(1+(2+...)))`
   - Trees: Parallel reduction `(((0+1)+(2+3))+((4+5)+(6+7)))`
   - 2M nodes: tree = 1s, list = timeout

4. **Lazy O(reachable), Not Eager O(V)**
   - Only compute what's actually needed
   - Don't materialize full state upfront
   - Let HVM4's laziness do the work

5. **Depth-Controlled Recursion**
   - Use `switch d:` for divide-and-conquer
   - Binary bifurcation: `fork(d+1, i*2+0), fork(d+1, i*2+1)`
   - Natural parallel structure

6. **Treat as Compiler Output**
   - Write how Bend would emit it, not how humans read it
   - Optimize for HVM4's reduction rules, not readability

### Performance Impact

| Pattern | Max Nodes | Time (100k) | Scalability |
|---------|-----------|-------------|-------------|
| List-based (original) | ~10k | Timeout | Poor |
| Tree-based (v2) | 2M+ | <1 second | Excellent |

The `src/*_v2.hvm4` files demonstrate these principles in action.

## Algorithms

Two implementations:
- **Original** (`src/path_*.hvm4`) - List-based, scales to ~10k nodes
- **v2** (`src/alg_*_v2.hvm4`) - Bend patterns, scales to 2M+ nodes

| Algorithm | Type | Graph | Method | v2 Status |
|-----------|------|-------|--------|-----------|
| Algebraic (tropical semiring) | APSP | 3 nodes, undirected, weighted | Matrix squaring over (min, +) semiring | 🔄 Planned |
| Bellman-Ford | SSSP | 5 nodes, directed, weighted | V-1 rounds of edge relaxation | 🔄 Planned |
| Bidirectional BFS | point-to-point | 7 nodes, undirected, unweighted | Alternating frontier expansion | 🔄 Planned |
| Contraction Hierarchy | point-to-point | 6 nodes, directed, weighted | Bidirectional upward-only search | ✅ `alg_ch_v2.hvm4` |
| Delta-Stepping | SSSP | 5 nodes, directed, weighted | Light/heavy edge classification with bucket rounds | 🔄 Planned |
| Superposition Enumeration | all-paths | 6 nodes, DAG, weighted | HVM4 SUP/DUP for non-deterministic branching | 🔄 Planned |
| Transitive Closure | Reachability | DAG | BFS with fuel limit | ✅ `alg_closure_v2.hvm4` |
| Borůvka MST | Minimum spanning tree | Weighted | Parallel edge contraction | ✅ `alg_boruvka_v2.hvm4` |

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

## Learning Path

**Start here:**
1. Read the Design Philosophy section above
2. Study `src/alg_closure_v2.hvm4` - simplest Bend pattern example
3. Compare with `src/path_*.hvm4` to see the difference
4. Review `src/alg_boruvka_v2.hvm4` and `src/alg_ch_v2.hvm4` for more complex patterns

**Key insight:** The v2 algorithms look "weird" because they're written as compiler output, not human code. That's intentional.

## Additional Resources

- **C3 Hybrid Library** (`c3lib/`) - Call HVM4 from C3 via FFI
- **libhvm4_graph** (`lib/`) - Clean C API with FFI hybrid for 100k+ node scaling
- **HVM4 Patterns Guide** (`docs/HVM4_PATTERNS.md`) - Complete reference

## Dependencies

[HVM4](https://github.com/HigherOrderCO/HVM4) — included as a git submodule.
