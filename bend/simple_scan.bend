# Simple prefix scan in Bend to see compilation output

# Tree type for binary tree of values
type BinTree:
  Leaf { val }
  Node { left, right }

# Build a tree of depth d with values 1,2,3,...
def build(d, offset):
  if d == 0:
    return BinTree/Leaf { val: offset + 1 }
  else:
    return BinTree/Node { 
      left: build(d - 1, offset * 2),
      right: build(d - 1, offset * 2 + 1)
    }

# Sum all leaves in tree
def tree_sum(tree):
  match tree:
    case BinTree/Leaf:
      return tree.val
    case BinTree/Node:
      return tree_sum(tree.left) + tree_sum(tree.right)

# Append two lists
def append(xs, ys):
  match xs:
    case List/Nil:
      return ys
    case List/Cons:
      return List/Cons { head: xs.head, tail: append(xs.tail, ys) }

# Flatten tree to list
def flatten(tree):
  match tree:
    case BinTree/Leaf:
      return [tree.val]
    case BinTree/Node:
      return append(flatten(tree.left), flatten(tree.right))

# Prefix scan - compute exclusive prefix sums
def prefix(tree, pfx):
  match tree:
    case BinTree/Leaf:
      return [pfx]
    case BinTree/Node:
      lsum = tree_sum(tree.left)
      return append(prefix(tree.left, pfx), prefix(tree.right, pfx + lsum))

# Main: scan depth-2 tree (4 elements: 1,2,3,4)
def main():
  tree = build(2, 0)
  return prefix(tree, 0)
