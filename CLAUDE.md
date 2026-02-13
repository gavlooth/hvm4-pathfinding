# CLAUDE.md - HVM4 Pathfinding Project

## ⚠️ MANDATORY: HVM4 is a VM, NOT a Language

**HVM4 is a virtual machine** — treat it like LLVM IR, JVM bytecode, or WASM.

You MUST write HVM4 code **the way Bend would emit it**, not as human-readable source.

### Why This Matters

HVM4's power comes from its parallel reduction model. Hand-written "readable" HVM4 code typically:
- Forces sequential patterns (bad)
- Misses parallelization opportunities (bad)
- Doesn't leverage SUP/DUP sharing (bad)

Bend-emitted patterns:
- Tree-structured (parallelizes naturally)
- Depth-controlled recursion
- Fold/bend for creation/consumption
- Automatic sharing via DUP

### Core Patterns (MANDATORY)

**1. Binary trees, not lists**
```
# ✅ Parallel: (((0+1)+(2+3))+((4+5)+(6+7)))
# ❌ Sequential: (0+(1+(2+(3+...))))
```

**2. Depth-controlled recursion with `switch d:`**
```hvm4
@f = λd λi (switch d {
  0: (base_case i)
  _: (combine (@f (- d 1) (* i 2)) (@f (- d 1) (+ (* i 2) 1)))
})
```

**3. Bifurcation pattern (bend)**
```hvm4
// fork(d+1, i*2+0), fork(d+1, i*2+1)
@bend = λd λi (switch d {
  0: (leaf i)
  _: let d1 = (- d 1)
     ((@bend d1 (* i 2)), (@bend d1 (+ (* i 2) 1)))
})
```

**4. Tree fold pattern**
```hvm4
@fold = λtree (match tree {
  (l, r): (op (@fold l) (@fold r))
  leaf: leaf
})
```

### Reference Materials

- `sketches/bend_patterns.md` — Bend-style pseudocode for 6 algorithms
- `sketches/hvm4_translations.md` — HVM4 syntax translations
- https://github.com/HigherOrderCO/Bend — Study compilation patterns

### Anti-Patterns (DO NOT USE)

❌ Sequential list traversal  
❌ Bellman-Ford, Dijkstra, delta-stepping (inherently sequential)  
❌ Let-binding explosion (one binding per node)  
❌ Dense arrays instead of trees  
❌ Hand-written "readable" HVM4  

### Algorithms That Fit

✅ Algebraic/semiring paths (matrix ops = parallel fold)  
✅ SUP-based exploration (tree branching + collapse)  
✅ Parallel prefix/scan (foundational)  
✅ Borůvka MST (parallel component selection)  
✅ Contraction hierarchies (tree structure)  
✅ Transitive closure (matrix squaring)  

## Build & Test

```bash
./build.sh          # Build HVM4
./run.sh <file>     # Run .hvm4 file
./test.sh           # Run test suite
./bench.sh          # Benchmarks
```

## Project Structure

```
src/           — HVM4 source files (Bend-emitted style)
sketches/      — Design patterns and translations
bench/         — Benchmark code
lib/           — Shared HVM4 libraries
c3lib/         — C3 FFI components
HVM4/          — HVM4 submodule
```
