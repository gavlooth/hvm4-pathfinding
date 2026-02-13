# Borůvka MST - Simplified working version
# Uses component labels directly (not union-find)

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

# Get component label at index i
def get_comp(i, comp):
  match comp:
    case List/Nil:
      return -1
    case List/Cons:
      if i == 0:
        return comp.head
      else:
        return get_comp(i - 1, comp.tail)

# Check if edge crosses components
def crosses(e, comp):
  u = get_src(e)
  v = get_dst(e)
  cu = get_comp(u, comp)
  cv = get_comp(v, comp)
  if cu == cv:
    return 0
  else:
    return 1

# Filter edges that cross components
def cross_edges(edges, comp):
  match edges:
    case List/Nil:
      return List/Nil
    case List/Cons:
      rest = cross_edges(edges.tail, comp)
      if crosses(edges.head, comp) == 1:
        return List/Cons { head: edges.head, tail: rest }
      else:
        return rest

# Find min weight edge
def min_edge(edges):
  match edges:
    case List/Nil:
      return Edge/E { src: -1, dst: -1, weight: 9999 }
    case List/Cons:
      rest_min = min_edge(edges.tail)
      hw = get_weight(edges.head)
      rw = get_weight(rest_min)
      if hw < rw:
        return edges.head
      else:
        return rest_min

# Update component: replace all cv with cu
def update_comp(cu, cv, comp):
  match comp:
    case List/Nil:
      return List/Nil
    case List/Cons:
      rest = update_comp(cu, cv, comp.tail)
      if comp.head == cv:
        return List/Cons { head: cu, tail: rest }
      else:
        return List/Cons { head: comp.head, tail: rest }

# Borůvka iteration
def boruvka(edges, comp, mst, fuel):
  if fuel == 0:
    return mst
  else:
    crossing = cross_edges(edges, comp)
    e = min_edge(crossing)
    w = get_weight(e)
    if w == 9999:
      return mst
    else:
      u = get_src(e)
      v = get_dst(e)
      cu = get_comp(u, comp)
      cv = get_comp(v, comp)
      new_comp = update_comp(cu, cv, comp)
      new_mst = List/Cons { head: e, tail: mst }
      return boruvka(edges, new_comp, new_mst, fuel - 1)

# Test graph: 4 nodes, 4 edges
# MST should have 3 edges
def test_edges():
  return [
    Edge/E { src: 0, dst: 1, weight: 1 },
    Edge/E { src: 1, dst: 2, weight: 2 },
    Edge/E { src: 0, dst: 3, weight: 3 },
    Edge/E { src: 2, dst: 3, weight: 4 }
  ]

def main():
  edges = test_edges()
  comp = [0, 1, 2, 3]
  return boruvka(edges, comp, [], 5)
