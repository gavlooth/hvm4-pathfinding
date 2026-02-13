# Bend → HVM4 Algorithm Sketches

## 1. Algebraic/Semiring Shortest Paths

**Idea:** Matrix multiply in (min, +) semiring. Distance matrix D, multiply D×D until fixpoint.

```python
# Semiring ops
def min_plus(a, b):  # ⊗ = +, ⊕ = min
  return min(a, b)

def add(a, b):
  return a + b

# Parallel matrix cell computation
def mat_cell(row_tree, col_tree):
  fold row_tree, col_tree:
    case (Leaf(a), Leaf(b)):
      return a + b
    case (Node(l1,r1), Node(l2,r2)):
      return min(mat_cell(l1, l2), mat_cell(r1, r2))

# Matrix multiply via tree fold
def mat_mul(A, B, depth):
  bend d = 0, i = 0, j = 0:
    when d < depth:
      cell = min(fork(d+1, i*2, j), fork(d+1, i*2+1, j))
    else:
      cell = A[i] + B[j]  # leaf: one term of dot product
  return cell

# Repeated squaring for transitive closure
def shortest_paths(D, depth, iters):
  fold iters:
    case 0:
      return D
    case _:
      return mat_mul(D, shortest_paths(D, depth, iters-1), depth)
```

---

## 2. SUP-based Exploration

**Idea:** Superposition of paths. Branch on choices, collapse to minimum.

```python
# Tree of distances - each branch is a path choice
def explore(graph, node, target, depth):
  if node == target:
    return 0  # found
  if depth == 0:
    return INF  # cutoff
  
  # Fork on all neighbors - parallel branches
  bend i = 0, neighbors = graph[node]:
    when i < len(neighbors):
      (next_node, weight) = neighbors[i]
      branch = weight + explore(graph, next_node, target, depth-1)
      result = min(fork(i+1, neighbors), branch)
    else:
      result = INF
  return result

# Collapse via fold - minimum of all branches
def shortest(graph, src, dst, max_depth):
  fold explore(graph, src, dst, max_depth):
    case Leaf(d):
      return d
    case Node(l, r):
      return min(l, r)
```

---

## 3. Parallel Prefix/Scan

**Idea:** Associative op over sequence. Build tree, fold up, sweep down.

```python
# Up-sweep: build tree of partial results
def up_sweep(arr, depth):
  bend d = 0, i = 0:
    when d < depth:
      left = fork(d+1, i*2)
      right = fork(d+1, i*2+1)
      node = (left, right, left.sum + right.sum)
    else:
      node = (arr[i], arr[i], arr[i])
  return node

# Down-sweep: propagate prefix from root
def down_sweep(tree, prefix):
  fold tree with prefix:
    case Leaf(v):
      return prefix
    case Node(l, r, sum):
      left_prefix = prefix
      right_prefix = prefix + l.sum
      return (down_sweep(l, left_prefix), down_sweep(r, right_prefix))

# Full scan
def scan(arr, depth):
  tree = up_sweep(arr, depth)
  return down_sweep(tree, 0)
```

---

## 4. Borůvka MST

**Idea:** Each component picks min outgoing edge simultaneously. Merge. Repeat log(n) times.

```python
# Find min edge for each component (parallel over components)
def find_min_edges(components, edges, depth):
  bend d = 0, c = 0:
    when d < depth:
      left = fork(d+1, c*2)
      right = fork(d+1, c*2+1)
      result = (left, right)
    else:
      # Leaf: find min edge leaving component c
      result = fold edges:
        case Nil:
          return INF_EDGE
        case Cons(e, rest):
          if crosses_component(e, components, c):
            return min_edge(e, rest)
          else:
            return rest
  return result

# One Borůvka iteration
def boruvka_step(components, edges, depth):
  min_edges = find_min_edges(components, edges, depth)
  new_components = merge_components(components, min_edges)
  return (new_components, min_edges)

# Full MST: log(n) iterations
def mst(graph, depth):
  bend iter = 0, components = init_components(graph):
    when iter < depth:  # log(n) iterations
      (new_comp, edges) = boruvka_step(components, graph.edges, depth)
      result = (edges, fork(iter+1, new_comp))
    else:
      result = Nil
  return result
```

---

## 5. Contraction Hierarchies

**Idea:** Preprocess into hierarchy (tree). Query = bidirectional tree search.

```python
# Preprocessing: contract nodes by importance (bottom-up tree build)
def build_hierarchy(graph, depth):
  bend d = 0, g = graph:
    when d < depth:
      # Contract least important node
      (contracted, shortcuts) = contract_least_important(g)
      level = (contracted.node, shortcuts)
      rest = fork(d+1, contracted.remaining)
      result = Node(level, rest)
    else:
      result = Leaf(g)  # top of hierarchy
  return result

# Query: bidirectional search up hierarchy (parallel forward/backward)
def ch_query(hierarchy, src, dst):
  # Forward search from src (going UP hierarchy)
  forward = bend d = 0, node = src, dist = 0:
    when d < depth and node != top:
      up_edges = hierarchy.up_edges[node]
      result = fold up_edges:
        case Nil: return (node, dist)
        case Cons((next, w), rest):
          return min((fork(d+1, next, dist+w)), rest)
    else:
      result = (node, dist)
  
  # Backward search from dst (parallel with forward)
  backward = bend d = 0, node = dst, dist = 0:
    # ... same pattern ...
  
  # Meeting point: min over all meeting nodes
  return fold forward, backward:
    case (f_node, f_dist), (b_node, b_dist):
      if f_node == b_node:
        return f_dist + b_dist
      else:
        return INF
```

---

## 6. Transitive Closure (Matrix Squaring)

**Idea:** Boolean matrix, square log(n) times. D^(2^k) gives paths of length 2^k.

```python
# Boolean matrix multiply (OR of ANDs)
def bool_mat_mul(A, B, depth):
  bend d = 0, i = 0, j = 0:
    when d < depth:
      # Tree fold over k dimension
      left = fork(d+1, i, j, k_range_left)
      right = fork(d+1, i, j, k_range_right)
      cell = left | right  # OR
    else:
      cell = A[i][k] & B[k][j]  # AND at leaf
  return cell

# Repeated squaring
def transitive_closure(adj, depth):
  bend iter = 0, D = adj:
    when iter < log2(depth):
      D_next = bool_mat_mul(D, D, depth)
      result = D | fork(iter+1, D_next)  # accumulate reachability
    else:
      result = D
  return result
```

---

## Key Patterns Summary

| Pattern | Bend Construct | HVM4 Equivalent |
|---------|----------------|-----------------|
| Parallel creation | `bend d, i: when ... fork()` | `@f = λd λi (switch d ... (@f (+ d 1) (* i 2)))` |
| Parallel reduction | `fold tree: case Node: l ⊕ r` | `@fold = λt (match t { Node(l,r): (⊕ (@fold l) (@fold r)) })` |
| Tree structure | `Node { left, right }` / `Leaf { value }` | `(left, right)` / `value` |
| Depth control | `switch d: case 0: ... case _: ...` | `(switch d { 0: base; _: rec(d-1) })` |
