# Bend → HVM1 → HVM4 Translation Guide

## Key Insight

Write algorithms in **Bend** (high-level), compile to **HVM1** via `bend gen-hvm`, then translate to **HVM4** using this guide.

## Syntax Mapping

### HVM1 → HVM4

| HVM1 | HVM4 | Notes |
|------|------|-------|
| `@name = expr` | `@name = expr` | Same |
| `(a b)` | `(a b)` | Application same |
| `λa. body` | `λa. body` | Lambda same |
| `{a b}` | `! &a = x; ... use a twice` | Dup/clone - HVM4 uses affine `&` |
| `?((case0 case1) x)` | `λ{0: case0; λn. case1}(x)` | Switch pattern |
| `& @fn ~ args` | Sequential call | HVM4 handles parallelism differently |
| `&! @fn ~ args` | Same as above | Parallel spawn in HVM1 |
| `$([+] $(a b))` | `a + b` | Arithmetic |
| `$([=] $(a b))` | `a == b` via switch | Equality check |
| `$([<] $(a b))` | `a < b` via switch | Comparison |

### Type Constructors

**HVM1 (from Bend):**
```
@Edge/E = (a (b (c ((@Edge/E/tag (a (b (c d)))) d))))
@Edge/E/tag = 0
```

**HVM4 equivalent:**
```hvm4
// Constructor: λsrc. λdst. λweight. λf. f(tag)(src)(dst)(weight)
@Edge = λsrc. λdst. λweight. λf. f(0)(src)(dst)(weight)

// Pattern match: edge(λtag. λsrc. λdst. λweight. use_fields...)
```

### Pattern Matching (Switch)

**HVM1:**
```
@get_src = ((@get_src__C0 a) a)
@get_src__C0 = (?(((a (* (* a))) *) b) b)
```

**HVM4:**
```hvm4
@get_src = λedge. edge(λtag. λsrc. λdst. λweight. src)
```

### Lists

**HVM1:**
```
@List/Cons = (a (b ((@List/Cons/tag (a (b c))) c)))
@List/Nil = ((@List/Nil/tag a) a)
```

**HVM4:**
```hvm4
// Nil: λf. f(0)   or just []
// Cons: λh. λt. λf. f(1)(h)(t)   or h <> t

// Pattern match on list:
@match_list = λlist. λ{
  []: nil_case;
  <>: λh. λt. cons_case(h, t)
}
```

### Conditionals

**HVM1:**
```
$([<] $(a ?(((* result_false) (* result_true)) ...)))
```

**HVM4:**
```hvm4
// If a < b then X else Y:
λ{
  0: Y;  // false case (a >= b)
  λ_. X  // true case (a < b)
}(a < b)
```

## Working Example: min_edge

**Bend:**
```bend
def min_edge(edges):
  match edges:
    case List/Nil:
      return Edge/E { src: -1, dst: -1, weight: 9999 }
    case List/Cons:
      rest_min = min_edge(edges.tail)
      head_w = get_weight(edges.head)
      rest_w = get_weight(rest_min)
      if head_w < rest_w:
        return edges.head
      else:
        return rest_min
```

**HVM4:**
```hvm4
@min_edge = λ{
  []: @Edge(-1)(-1)(9999);
  <>: λ&head. λ&tail.
    ! &rest = @min_edge(tail);
    ! &hw = @get_weight(head);
    ! &rw = @get_weight(rest);
    λ{
      0: rest;     // hw >= rw
      λ_. head     // hw < rw
    }(hw < rw)
}
```

## Affine Variables (Critical!)

HVM4 requires marking variables that are used multiple times with `&`:

```hvm4
// WRONG - tree used twice
@tree_sum = λtree. λ{
  0: tree;
  λn. tree(λl. λr. @tree_sum(l) + @tree_sum(r))  // ERROR: tree used in switch AND in body
}

// CORRECT - mark with &
@tree_sum = λ&tree. λ{
  0: tree;
  λn. tree(λ&l. λ&r. @tree_sum(l) + @tree_sum(r))
}(depth)
```

## Compilation Workflow

```bash
# 1. Write algorithm in Bend
vim my_algo.bend

# 2. Test in Bend
bend run my_algo.bend

# 3. Generate HVM1
bend gen-hvm my_algo.bend > my_algo.hvm

# 4. Study HVM1 output patterns
cat my_algo.hvm

# 5. Translate to HVM4 using this guide
# Key transformations:
# - {a b} → use &a for cloning
# - ?((c0 c1) x) → λ{0:c0; λn.c1}(x)
# - $([op] $(a b)) → a op b
# - & @fn ~ args → regular call, HVM4 auto-parallelizes
```

## Common Pitfalls

1. **Forgot `&` on reused variable** → "variable X used 2 times"
2. **Wrong switch syntax** → use `λ{0:...; λn....}(x)` not `λ{0:...; n:...}`
3. **Tuples vs lists** → HVM4 uses spaces not commas: `(a b)` not `(a, b)`
4. **Church pairs** → Extract with `pair(λa. λb. use_a_and_b)`
