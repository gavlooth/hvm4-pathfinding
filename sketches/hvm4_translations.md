# HVM4 Translations (Bend-emitted style)

## 1. Algebraic/Semiring Shortest Paths

```hvm4
// Tree representation: (left, right) for Node, value for Leaf
// Semiring: ⊕ = min, ⊗ = +

// Min of two values
@min = λa λb (< a b a b)

// Parallel dot product via tree fold (one row × one col)
@dot = λdepth λrow λcol
  (switch depth {
    0: (+ row col)
    _: let d1 = (- depth 1)
       let (r0, r1) = row
       let (c0, c1) = col
       (@min (@dot d1 r0 c0) (@dot d1 r1 c1))
  })

// Build result matrix cell (i,j) via tree
@mat_cell = λdepth λA λB λi λj
  (@dot depth (@row A i) (@col B j))

// Matrix multiply - builds tree of cells
@mat_mul = λdepth λA λB
  (switch depth {
    0: (@mat_cell 0 A B 0 0)
    _: let d1 = (- depth 1)
       ((@mat_mul d1 A B), (@mat_mul d1 A B))
  })

// Repeated squaring: D^(2^k)
@shortest = λiters λD λdepth
  (switch iters {
    0: D
    _: (@mat_mul depth D (@shortest (- iters 1) D depth))
  })
```

---

## 2. SUP-based Exploration

```hvm4
// INF represented as large number
@INF = 16777215

// Explore all paths as parallel branches
@explore = λgraph λnode λtarget λdepth
  (switch (== node target) {
    1: 0  // found target
    0: (switch depth {
      0: @INF  // cutoff
      _: let neighbors = (@get_neighbors graph node)
         (@explore_neighbors graph neighbors target (- depth 1))
    })
  })

// Parallel over neighbors - tree of branches
@explore_neighbors = λgraph λneighbors λtarget λdepth
  (match neighbors {
    Nil: @INF
    (Cons (next, weight) rest):
      let branch = (+ weight (@explore graph next target depth))
      let others = (@explore_neighbors graph rest target depth)
      (@min branch others)
  })

// Main entry - builds SUP tree, collapses to min
@shortest_path = λgraph λsrc λdst λmax_depth
  (@explore graph src dst max_depth)
```

---

## 3. Parallel Prefix/Scan

```hvm4
// Up-sweep: build tree of partial sums
@up_sweep = λdepth λarr λi
  (switch depth {
    0: let v = (@get arr i) in (v, v, v)  // (val, sum, val)
    _: let d1 = (- depth 1)
       let left = (@up_sweep d1 arr (* i 2))
       let right = (@up_sweep d1 arr (+ (* i 2) 1))
       let (_, lsum, _) = left
       let (_, rsum, _) = right
       (left, right, (+ lsum rsum))
  })

// Down-sweep: propagate prefix
@down_sweep = λtree λprefix
  (match tree {
    (val, val, val): prefix  // Leaf
    (left, right, sum):
      let (_, lsum, _) = left
      let left_result = (@down_sweep left prefix)
      let right_result = (@down_sweep right (+ prefix lsum))
      (left_result, right_result)
  })

// Full scan
@scan = λdepth λarr
  let tree = (@up_sweep depth arr 0)
  (@down_sweep tree 0)
```

---

## 4. Borůvka MST

```hvm4
// Find min edge for component c (fold over edges)
@find_min_edge = λedges λcomponents λc
  (match edges {
    Nil: (@INF, 0, 0)  // (weight, from, to)
    (Cons (w, u, v) rest):
      let rest_min = (@find_min_edge rest components c)
      (switch (@crosses c components u v) {
        1: (@min_edge (w, u, v) rest_min)
        0: rest_min
      })
  })

// Parallel over all components (tree structure)
@find_all_mins = λdepth λcomponents λedges λc
  (switch depth {
    0: (@find_min_edge edges components c)
    _: let d1 = (- depth 1)
       let left = (@find_all_mins d1 components edges (* c 2))
       let right = (@find_all_mins d1 components edges (+ (* c 2) 1))
       (left, right)
  })

// One Borůvka iteration
@boruvka_step = λdepth λcomponents λedges
  let mins = (@find_all_mins depth components edges 0)
  let new_comp = (@merge_components components mins)
  (mins, new_comp)

// Full MST: log(n) iterations
@mst = λiters λdepth λgraph
  (switch iters {
    0: Nil
    _: let (edges, new_comp) = (@boruvka_step depth graph.components graph.edges)
       (Cons edges (@mst (- iters 1) depth new_comp graph.edges))
  })
```

---

## 5. Contraction Hierarchies

```hvm4
// Query: bidirectional search meeting at top
// Forward search (up the hierarchy)
@search_up = λhierarchy λnode λdist λdepth
  (switch depth {
    0: (node, dist)
    _: let up_edges = (@get_up_edges hierarchy node)
       (@search_up_edges hierarchy up_edges dist (- depth 1))
  })

@search_up_edges = λhierarchy λedges λdist λdepth
  (match edges {
    Nil: (@INF, @INF)
    (Cons (next, w) rest):
      let this_path = (@search_up hierarchy next (+ dist w) depth)
      let rest_paths = (@search_up_edges hierarchy rest dist depth)
      (@min_pair this_path rest_paths)
  })

// Parallel forward + backward
@ch_query = λhierarchy λsrc λdst λdepth
  let forward = (@search_up hierarchy src 0 depth)
  let backward = (@search_up hierarchy dst 0 depth)
  // Find meeting point with min total distance
  (@find_meeting forward backward)

@find_meeting = λfwd λbwd
  (match fwd {
    (f_node, f_dist):
      (match bwd {
        (b_node, b_dist):
          (switch (== f_node b_node) {
            1: (+ f_dist b_dist)
            0: @INF
          })
      })
  })
```

---

## 6. Transitive Closure (Matrix Squaring)

```hvm4
// Boolean AND
@band = λa λb (* a b)

// Boolean OR  
@bor = λa λb (| a b)

// Boolean dot product (OR of ANDs)
@bool_dot = λdepth λrow λcol
  (switch depth {
    0: (@band row col)
    _: let d1 = (- depth 1)
       let (r0, r1) = row
       let (c0, c1) = col
       (@bor (@bool_dot d1 r0 c0) (@bool_dot d1 r1 c1))
  })

// Boolean matrix multiply
@bool_mat_mul = λdepth λA λB
  (switch depth {
    0: (@bool_dot depth (@row A 0) (@col B 0))
    _: let d1 = (- depth 1)
       ((@bool_mat_mul d1 A B), (@bool_mat_mul d1 A B))
  })

// Transitive closure via repeated squaring
@closure = λiters λD λdepth
  (switch iters {
    0: D
    _: let D2 = (@bool_mat_mul depth D D)
       let D_acc = (@mat_or depth D D2)  // D | D^2
       (@closure (- iters 1) D_acc depth)
  })

// Element-wise OR of matrices
@mat_or = λdepth λA λB
  (switch depth {
    0: (@bor A B)
    _: let d1 = (- depth 1)
       let (a0, a1) = A
       let (b0, b1) = B
       ((@mat_or d1 a0 b0), (@mat_or d1 a1 b1))
  })
```

---

## Pattern Summary: Bend → HVM4

| Bend | HVM4 |
|------|------|
| `bend d=0: when d<n: fork(d+1)` | `@f = λd (switch d { 0: base; _: (@f (- d 1)) })` |
| `fold tree: case Node(l,r): op(l,r)` | `(match tree { (l,r): (op (@f l) (@f r)) })` |
| `(a, b)` tuple | `(a, b)` |
| `switch n: case 0: ... case _: ...` | `(switch n { 0: ...; _: ... })` |
| `type List: Nil, Cons{h,~t}` | `Nil` / `(Cons head tail)` |
