#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

HVM=./HVM4/clang/main

if [ ! -x "$HVM" ]; then
  echo "HVM4 not built. Run ./build.sh first."
  exit 1
fi

if [ $# -eq 0 ]; then
  echo "Usage: ./run.sh <algorithm> [hvm4 flags]"
  echo ""
  echo "Algorithms:"
  echo "  algebraic              Tropical semiring APSP (3 nodes)"
  echo "  bellman_ford           Bellman-Ford SSSP (5 nodes)"
  echo "  bidir_bfs              Bidirectional BFS (7 nodes)"
  echo "  contraction_hierarchy  Contraction Hierarchy query (6 nodes)"
  echo "  delta_step             Delta-stepping SSSP (5 nodes)"
  echo "  sup_enum               Superposition path enumeration (6 nodes)"
  echo "  all                    Run all algorithms"
  exit 0
fi

algo="$1"
shift

run_one() {
  local name="$1"
  shift
  echo "=== $name ==="
  "$HVM" "src/path_${name}.hvm4" -s "$@"
  echo ""
}

if [ "$algo" = "all" ]; then
  for f in src/path_*.hvm4; do
    name=$(basename "$f" .hvm4)
    name=${name#path_}
    run_one "$name" "$@"
  done
else
  run_one "$algo" "$@"
fi
