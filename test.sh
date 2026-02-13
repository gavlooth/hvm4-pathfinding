#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

HVM=./HVM4/clang/main

if [ ! -x "$HVM" ]; then
  echo "HVM4 not built. Run ./build.sh first."
  exit 1
fi

pass=0
fail=0

check() {
  local file="$1"
  local expected="$2"
  local flags="${3:-}"
  local name
  name=$(basename "$file" .hvm4)

  actual=$("$HVM" "$file" $flags 2>/dev/null | sed 's/ \x1b\[2m#[0-9]*\x1b\[0m//g' | tr '\n' '|' | sed 's/|$//')

  if [ "$actual" = "$expected" ]; then
    echo "PASS  $name"
    pass=$((pass + 1))
  else
    echo "FAIL  $name"
    echo "  expected: $expected"
    echo "  actual:   $actual"
    fail=$((fail + 1))
  fi
}

echo "Running pathfinding tests..."
echo ""

check src/path_algebraic.hvm4                "[[0,2,5],[2,0,3],[5,3,0]]"
check src/path_bellman_ford.hvm4             "[0,3,2,6,7]"
check src/path_bellman_ford_trie.hvm4        "[0,3,2,6,7]"
check src/path_bellman_ford_trie32.hvm4      "[0,3,2,6,7]"
check src/path_bellman_ford_btrie.hvm4       "[0,3,2,6,7]"
check src/path_bellman_ford_btrie_et.hvm4    "[0,3,2,6,7]"
check src/path_bellman_ford_q4_et.hvm4      "[0,3,2,6,7]"
check src/path_bellman_ford_q4_adj_et.hvm4  "[0,3,2,6,7]"
check src/path_bidir_bfs.hvm4               "2"
check src/path_contraction_hierarchy.hvm4     "10"
check src/path_contraction_hierarchy_trie.hvm4 "10"
check src/path_contraction_hierarchy_trie32.hvm4 "10"
check src/path_contraction_hierarchy_btrie.hvm4  "10"
check src/path_delta_step.hvm4               "[0,1,3,5,4]"
check src/path_delta_step_trie.hvm4          "[0,1,3,5,4]"
check src/path_delta_step_trie32.hvm4        "[0,1,3,5,4]"
check src/path_delta_step_btrie_et.hvm4      "[0,1,3,5,4]"
check src/path_sup_enum.hvm4                "6|7|11|6" "-C10"

echo ""
echo "Results: $pass passed, $fail failed"
[ "$fail" -eq 0 ]
