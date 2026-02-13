# Transitive Closure in Bend - Simplified
# Use edge list instead of adjacency list

# Check if node is in list
def member(x, lst):
  match lst:
    case List/Nil:
      return 0
    case List/Cons:
      if lst.head == x:
        return 1
      else:
        return member(x, lst.tail)

# Append two lists
def append(xs, ys):
  match xs:
    case List/Nil:
      return ys
    case List/Cons:
      return List/Cons { head: xs.head, tail: append(xs.tail, ys) }

# Edge type
type Edge:
  E { src, dst }

def get_src(e):
  match e:
    case Edge/E:
      return e.src

def get_dst(e):
  match e:
    case Edge/E:
      return e.dst

# Get all destinations from a node (edges where src == node)
def neighbors(node, edges):
  match edges:
    case List/Nil:
      return List/Nil
    case List/Cons:
      rest = neighbors(node, edges.tail)
      src = get_src(edges.head)
      dst = get_dst(edges.head)
      if src == node:
        return List/Cons { head: dst, tail: rest }
      else:
        return rest

# BFS step: process frontier, return visited
def bfs(frontier, edges, visited, fuel):
  if fuel == 0:
    return visited
  else:
    match frontier:
      case List/Nil:
        return visited
      case List/Cons:
        node = frontier.head
        rest = frontier.tail
        if member(node, visited) == 1:
          return bfs(rest, edges, visited, fuel - 1)
        else:
          nbrs = neighbors(node, edges)
          new_frontier = append(rest, nbrs)
          new_visited = List/Cons { head: node, tail: visited }
          return bfs(new_frontier, edges, new_visited, fuel - 1)

# Test graph: 0 -> 1 -> 2, 0 -> 3
def test_edges():
  return [
    Edge/E { src: 0, dst: 1 },
    Edge/E { src: 0, dst: 3 },
    Edge/E { src: 1, dst: 2 }
  ]

# All nodes reachable from 0
def main():
  edges = test_edges()
  return bfs([0], edges, [], 10)
