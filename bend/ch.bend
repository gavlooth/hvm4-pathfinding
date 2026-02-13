# Contraction Hierarchies (CH) - Simplified
# Bidirectional Dijkstra on hierarchical graph

type Edge:
  E { src, dst, weight }

def get_src(e):
  match e:
    case Edge/E:
      return e.src

def get_dst(e):
  match e:
    case Edge/E:
      return e.dst

def get_weight(e):
  match e:
    case Edge/E:
      return e.weight

# State: (node, distance) pairs
type State:
  S { node, dist }

def get_node(s):
  match s:
    case State/S:
      return s.node

def get_dist(s):
  match s:
    case State/S:
      return s.dist

# Get neighbors going "up" in hierarchy (dst > src)
def up_neighbors(node, edges):
  match edges:
    case List/Nil:
      return List/Nil
    case List/Cons:
      rest = up_neighbors(node, edges.tail)
      src = get_src(edges.head)
      dst = get_dst(edges.head)
      if src == node:
        if dst > src:
          w = get_weight(edges.head)
          return List/Cons { head: State/S { node: dst, dist: w }, tail: rest }
        else:
          return rest
      else:
        return rest

# BFS-style search upward, collect all reachable (node, dist) pairs
def search_up(frontier, edges, reached, fuel):
  if fuel == 0:
    return reached
  else:
    match frontier:
      case List/Nil:
        return reached
      case List/Cons:
        state = frontier.head
        rest = frontier.tail
        node = get_node(state)
        dist = get_dist(state)
        nbrs = up_neighbors(node, edges)
        # Add dist to neighbor distances
        updated_nbrs = add_dist(dist, nbrs)
        new_frontier = append(rest, updated_nbrs)
        new_reached = List/Cons { head: state, tail: reached }
        return search_up(new_frontier, edges, new_reached, fuel - 1)

def add_dist(d, states):
  match states:
    case List/Nil:
      return List/Nil
    case List/Cons:
      rest = add_dist(d, states.tail)
      n = get_node(states.head)
      w = get_dist(states.head)
      return List/Cons { head: State/S { node: n, dist: d + w }, tail: rest }

def append(xs, ys):
  match xs:
    case List/Nil:
      return ys
    case List/Cons:
      return List/Cons { head: xs.head, tail: append(xs.tail, ys) }

# Find if node exists in reached with its distance
def find_in_reached(node, reached):
  match reached:
    case List/Nil:
      return -1  # Not found
    case List/Cons:
      n = get_node(reached.head)
      if n == node:
        return get_dist(reached.head)
      else:
        return find_in_reached(node, reached.tail)

# Find meeting point: min(forward_dist + backward_dist)
def find_meeting(fwd, bwd):
  match fwd:
    case List/Nil:
      return 9999
    case List/Cons:
      rest_min = find_meeting(fwd.tail, bwd)
      n = get_node(fwd.head)
      fd = get_dist(fwd.head)
      bd = find_in_reached(n, bwd)
      if bd == -1:
        return rest_min
      else:
        total = fd + bd
        if total < rest_min:
          return total
        else:
          return rest_min

# CH query: bidirectional search
def ch_query(src, dst, edges, fuel):
  fwd_start = [State/S { node: src, dist: 0 }]
  bwd_start = [State/S { node: dst, dist: 0 }]
  fwd_reached = search_up(fwd_start, edges, [], fuel)
  bwd_reached = search_up(bwd_start, edges, [], fuel)
  return find_meeting(fwd_reached, bwd_reached)

# Test: hierarchical graph
# Level 0: 0, 1
# Level 1: 2, 3  
# Edges: 0->2 (w=1), 1->2 (w=2), 2->3 (w=1), 1->3 (w=3)
def test_edges():
  return [
    Edge/E { src: 0, dst: 2, weight: 1 },
    Edge/E { src: 1, dst: 2, weight: 2 },
    Edge/E { src: 2, dst: 3, weight: 1 },
    Edge/E { src: 1, dst: 3, weight: 3 }
  ]

def main():
  edges = test_edges()
  return ch_query(0, 1, edges, 10)  # Should find path 0->2->... meeting 1->2
