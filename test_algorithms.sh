#!/usr/bin/env bash
# Test all implemented HVM4 algorithms

set -euo pipefail
cd "$(dirname "$0")"

HVM=./HVM4/clang/main

echo "================================"
echo "Testing HVM4 Algorithm Implementations"
echo "================================"
echo ""

run_test() {
  local name="$1"
  local file="src/alg_${name}.hvm4"
  
  echo "--- $name ---"
  if "$HVM" "$file" -s 2>&1; then
    echo "✅ PASSED"
  else
    echo "❌ FAILED"
  fi
  echo ""
}

echo "✅ WORKING ALGORITHMS:"
echo ""

run_test "prefix_scan"
run_test "semiring"
run_test "sup_explore"

echo "⚠️  ALGORITHMS NEEDING DEBUG:"
echo ""

run_test "closure"
run_test "boruvka"
run_test "ch"

echo "================================"
echo "Test run complete. See IMPLEMENTATION_REPORT.md for details."
echo "================================"
