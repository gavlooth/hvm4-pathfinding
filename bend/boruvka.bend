# Borůvka MST in Bend - compile to HVM to see correct patterns

# Edge: (src, dst, weight)
type Edge:
  E { src, dst, weight }

# Get weight from edge
def get_weight(e):
  match e:
    case Edge/E:
      return e.weight

# Find minimum edge from a list
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

# Get source from edge
def get_src(e):
  match e:
    case Edge/E:
      return e.src

# Filter edges that connect to a node
def edges_from(node, edges):
  match edges:
    case List/Nil:
      return List/Nil
    case List/Cons:
      rest = edges_from(node, edges.tail)
      src = get_src(edges.head)
      if src == node:
        return List/Cons { head: edges.head, tail: rest }
      else:
        return rest

# Simple graph: edges as list
# 0 --1-- 1 --2-- 2
# |       |
# 3       4
# |       |
# 3 --5-- 4

def test_edges():
  return [
    Edge/E { src: 0, dst: 1, weight: 1 },
    Edge/E { src: 1, dst: 2, weight: 2 },
    Edge/E { src: 0, dst: 3, weight: 3 },
    Edge/E { src: 1, dst: 4, weight: 4 },
    Edge/E { src: 3, dst: 4, weight: 5 }
  ]

# Find min edge from node 0
def main():
  edges = test_edges()
  from_0 = edges_from(0, edges)
  return min_edge(from_0)
