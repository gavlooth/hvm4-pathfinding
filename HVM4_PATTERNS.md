# HVM4 Patterns: Complete Reference

A comprehensive guide to writing scalable HVM4 code, including Bend→HVM4 translation.

## Table of Contents
1. [Core Syntax](#core-syntax)
2. [Affine Variables](#affine-variables)
3. [Pattern Matching](#pattern-matching)
4. [Data Structures](#data-structures)
5. [Scalable Algorithms](#scalable-algorithms)
6. [Bend→HVM4 Translation](#bend-to-hvm4-translation)
7. [Common Pitfalls](#common-pitfalls)

---

## Core Syntax

### Definitions
```hvm4
@name = expression
@main = @entrypoint(args)
```

### Lambda & Application
```hvm4
λx. body              // Lambda
@f(a)                 // Single arg
@f(a, b, c)           // Multiple args (curried)
```

### Let Bindings
```hvm4
!x = expr;            // Simple let (x used once)
!&x = expr;           // Let with auto-dup (x can be reused)
!A&B = expr;          // Explicit dup with label B, access via A₀, A₁
```

### Arithmetic
```hvm4
a + b    a - b    a * b    a / b    a % b
a == b   a < b    a > b    a <= b   a >= b
```

---

## Affine Variables

HVM4 enforces **affine typing**: each variable used at most once unless marked.

### ❌ Wrong: Variable used twice
```hvm4
@double = λx. x + x   // ERROR: 'x' used 2 times
```

### ✅ Fix: Mark with `&`
```hvm4
@double = λ&x. x + x  // OK: &x allows reuse
```

### ✅ Alternative: Explicit dup
```hvm4
@double = λx. !X&D = x; X₀ + X₁
```

### Rule of Thumb
- Mark parameter as `&x` if used in multiple places
- Mark let-binding as `!&x` if result used multiple times

---

## Pattern Matching

### Numeric Switch
```hvm4
// Match on number
λ{
  0: zero_case;
  1: one_case;
  λn. default_case    // n binds the predecessor for n > matched
}(value)

// Example: factorial
@fact = λ&n. λ{0: 1; λp. n * @fact(p)}(n)
```

### List Match
```hvm4
λ{
  []: nil_case;
  <>: λhead.λtail. cons_case
}(list)

// Example: length
@len = λ{[]: 0; <>: λh.λt. 1 + @len(t)}
```

### Tagged Union Match
```hvm4
λ{
  #Tag1: λfield1.λfield2. case1;
  #Tag2: λfield. case2;
  λx. default
}(value)

// Example: tree operations
@count = λ{
  #Leaf: λid. 1;
  #Node: λid.λl.λr. 1 + @count(l) + @count(r)
}
```

---

## Data Structures

### Lists
```hvm4
[]                    // Empty list
[1, 2, 3]             // Literal
x <> xs               // Cons (prepend)

// Operations
@append = λ{[]: λys.ys; <>: λx.λxs.λys. x <> @append(xs,ys)}
@len = λ{[]: 0; <>: λh.λt. 1 + @len(t)}
@map = λ&f. λ{[]: []; <>: λh.λt. f(h) <> @map(f,t)}
```

### Tagged Tuples (ADTs)
```hvm4
#Pair{a, b}           // Constructor
#Node{id, left, right}
#Leaf{value}

// Access via pattern match
@fst = λ{#Pair: λa.λb. a}
@snd = λ{#Pair: λa.λb. b}
```

### Trees (Scalable!)
```hvm4
// Binary tree
@build = λ&id. λ{
  0: #Leaf{id};
  λ&d. #Node{id, @build(id * 2, d - 1), @build(id * 2 + 1, d - 1)}
}

// Operations
@count = λ{#Leaf: λid. 1; #Node: λid.λl.λr. 1 + @count(l) + @count(r)}
@sum = λ{#Leaf: λv. v; #Node: λv.λl.λr. v + @sum(l) + @sum(r)}
```

---

## Scalable Algorithms

### ❌ Lists Don't Scale
```hvm4
// This is O(n) sequential - BAD for large n
@edges = [#E{0,1}, #E{1,2}, ..., #E{99999,100000}]
@process = λ{[]: done; <>: λh.λt. work(h); @process(t)}
```

### ✅ Trees Scale
```hvm4
// This is O(log n) depth, O(n) parallel work - GOOD
@build = λ&id. λ{0: #Leaf{id}; λ&d. #Node{id, @build(id*2,d-1), @build(id*2+1,d-1)}}

@parallel_reduce = λ{
  #Leaf: λv. base(v);
  #Node: λv.λl.λr. combine(@parallel_reduce(l), @parallel_reduce(r))
}
```

### Pattern: Tree-Structured Graph Algorithms

**Closure (BFS/reachability):**
```hvm4
@closure = λ{
  #Leaf: λid. 1;
  #Node: λid.λl.λr. 1 + @closure(l) + @closure(r)
}
// 2M nodes in 457ms
```

**Borůvka MST:**
```hvm4
@build = λ&id. λ{
  0: #Leaf{id};
  λ&d. !w1 = (id * 7) % 10 + 1;
       !w2 = (id * 13) % 10 + 1;
       #Node{id, w1, @build(id*2,d-1), w2, @build(id*2+1,d-1)}
}

@mst_weight = λ{
  #Leaf: λid. 0;
  #Node: λid.λw1.λl.λw2.λr. w1 + w2 + @mst_weight(l) + @mst_weight(r)
}
// 2M nodes in 836ms
```

**Shortest Path (CH-style):**
```hvm4
@minpath = λ{
  #Leaf: λid. 0;
  #Node: λid.λw1.λl.λw2.λr.
    !&pl = w1 + @minpath(l);
    !&pr = w2 + @minpath(r);
    λ{0: pr; λx. pl}(pl < pr)
}
// 2M nodes in 983ms
```

---

## Bend to HVM4 Translation

### Workflow
```bash
# 1. Write in Bend
vim algo.bend

# 2. Test in Bend
bend run algo.bend

# 3. Generate HVM1
bend gen-hvm algo.bend > algo.hvm

# 4. Study patterns, translate to HVM4
```

### Syntax Mapping

| Bend | HVM1 | HVM4 |
|------|------|------|
| `def f(x): x + 1` | `@f = (λx (+ x 1))` | `@f = λx. x + 1` |
| `match x: case A: ...` | `?((case0 case1) x)` | `λ{#A: ...; λx. ...}(x)` |
| `[1, 2, 3]` | `@List/Cons(1, ...)` | `[1, 2, 3]` or `1 <> 2 <> 3 <> []` |
| `if cond: a else: b` | `?(((* b) (* a)) cond)` | `λ{0: b; λ_. a}(cond)` |
| `x * 2` | `$([*] $(x 2))` | `x * 2` |

### Type Constructors

**Bend:**
```bend
type Edge:
  E { src, dst, weight }
```

**HVM1 (generated):**
```
@Edge/E = (a (b (c ((@Edge/E/tag (a (b (c d)))) d))))
@Edge/E/tag = 0
```

**HVM4:**
```hvm4
// Just use tagged tuple directly
#Edge{src, dst, weight}

// Accessors via pattern match
@get_src = λ{#Edge: λsrc.λdst.λw. src}
@get_dst = λ{#Edge: λsrc.λdst.λw. dst}
@get_weight = λ{#Edge: λsrc.λdst.λw. w}
```

### Function Translation

**Bend:**
```bend
def min_edge(edges):
  match edges:
    case List/Nil:
      return Edge/E { src: 999, dst: 999, weight: 9999 }
    case List/Cons:
      rest_min = min_edge(edges.tail)
      if get_weight(edges.head) < get_weight(rest_min):
        return edges.head
      else:
        return rest_min
```

**HVM4:**
```hvm4
@min_edge = λ{
  []: #Edge{999, 999, 9999};
  <>: λ&head.λ&tail.
    !&rest = @min_edge(tail);
    !&hw = @get_weight(head);
    !&rw = @get_weight(rest);
    λ{0: rest; λz. head}(hw < rw)
}
```

### Recursion Translation

**Bend:**
```bend
def sum(n):
  if n == 0:
    return 0
  else:
    return n + sum(n - 1)
```

**HVM4:**
```hvm4
@sum = λ&n. λ{0: 0; λp. n + @sum(p)}(n)
```

---

## Common Pitfalls

### 1. Variable Suffix Collision (BUG)
```hvm4
// ❌ BROKEN: Variables with same suffix merge incorrectly
@f = λsrc.λdst.
  !&fwd_start = [src];
  !&bwd_start = [dst];  // Both become [dst]!
  [fwd_start, bwd_start]

// ✅ FIXED: Use distinct names
@f = λsrc.λdst.
  !&fwdQ = [src];
  !&bwdR = [dst];
  [fwdQ, bwdR]
```
See: https://github.com/HigherOrderCO/HVM4/issues/41

### 2. Forgetting `&` on Reused Variables
```hvm4
// ❌ ERROR: 'x' used 2 times
@double = λx. x + x

// ✅ FIXED
@double = λ&x. x + x
```

### 3. List Algorithms Don't Scale
```hvm4
// ❌ O(n) sequential - 100k nodes = timeout
@edges = [e1, e2, ..., e100000]
@process = λ{[]: 0; <>: λh.λt. 1 + @process(t)}

// ✅ O(log n) depth - 2M nodes = 1 second
@tree = @build(1, 20)
@process = λ{#Leaf: λv. 1; #Node: λv.λl.λr. 1 + @process(l) + @process(r)}
```

### 4. Wrong Switch Syntax
```hvm4
// ❌ WRONG
λ{0: a; 1: b; _: c}(x)

// ✅ CORRECT (default uses λvar.)
λ{0: a; 1: b; λn. c}(x)
```

### 5. Negative Numbers Not Supported
```hvm4
// ❌ PARSE ERROR
@x = -1

// ✅ Use large sentinel instead
@NOT_FOUND = 999999
```

---

## Performance Guidelines

| Pattern | Depth | Scalability | Use Case |
|---------|-------|-------------|----------|
| List fold | O(n) | ❌ Bad | Small data (<1k) |
| Tree reduce | O(log n) | ✅ Good | Large data (1M+) |
| SUP branching | O(1) | ✅ Parallel | Search/exploration |

### Target Performance
- 30-50 MIPS (million interactions per second) single-threaded
- Near-linear scaling with `-T4` for tree algorithms
- 2M nodes in ~1 second is achievable

---

## Quick Reference

```hvm4
// Lambda & let
λx. body                    // Lambda
λ&x. body                   // Lambda, x reusable
!x = e;                     // Let
!&x = e;                    // Let, x reusable

// Pattern match
λ{0: a; λn. b}(x)           // Numeric
λ{[]: a; <>: λh.λt. b}(x)   // List
λ{#A: a; #B: λx. b}(x)      // Tagged

// Data
[1, 2, 3]                   // List
x <> xs                     // Cons
#Tag{a, b}                  // Tagged tuple

// Trees (scalable!)
@build = λ&id. λ{0: #Leaf{id}; λ&d. #Node{id, @build(id*2,d-1), @build(id*2+1,d-1)}}
@fold = λ{#Leaf: λv. v; #Node: λv.λl.λr. @fold(l) + @fold(r)}
```
