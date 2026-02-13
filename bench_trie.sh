#!/usr/bin/env bash
# Benchmark radix-16 vs radix-32 trie (and assoc-list where available)
# at various graph sizes.
#
# Usage: ./bench_trie.sh [runs]    (default: 3)
set -euo pipefail

cd "$(dirname "$0")"

HVM=./HVM4/clang/main
RUNS="${1:-3}"

if [ ! -x "$HVM" ]; then
  echo "HVM4 not built. Run ./build.sh first."
  exit 1
fi

# Generate benchmark files if missing
if ! ls bench/bf_trie16_*.hvm4 &>/dev/null; then
  echo "Generating benchmark files..."
  python3 bench/gen.py
  echo ""
fi

# Collect sizes from generated files
SIZES=()
for f in bench/bf_trie16_*.hvm4; do
  sz=$(basename "$f" .hvm4 | sed 's/bf_trie16_//')
  SIZES+=("$sz")
done
for f in bench/bf_btrie_*.hvm4; do
  sz=$(basename "$f" .hvm4 | sed 's/bf_btrie_//')
  # Only add if not already in SIZES
  if [[ ! " ${SIZES[*]} " =~ " ${sz} " ]]; then
    SIZES+=("$sz")
  fi
done

# Sort numerically
IFS=$'\n' SIZES=($(sort -n <<<"${SIZES[*]}")); unset IFS

echo "Bellman-Ford: assoc-list vs radix-16 trie vs radix-32 trie vs linear btrie"
echo "Best of $RUNS runs.  Itrs = HVM4 interactions.  Time = wall-clock us."
echo ""

for sz in "${SIZES[@]}"; do
  # Header for this size
  printf "=== V=%s " "$sz"

  # Show depth info from file comment
  t16_depth=$(head -1 "bench/bf_trie16_${sz}.hvm4" | grep -oP 'depth=\K[0-9]+' || echo "?")
  t32_depth=$(head -1 "bench/bf_trie32_${sz}.hvm4" | grep -oP 'depth=\K[0-9]+' || echo "?")
  bt_depth=$(head -1 "bench/bf_btrie_${sz}.hvm4" | grep -oP 'depth=\K[0-9]+' || echo "?")
  edges=$(head -1 "bench/bf_trie16_${sz}.hvm4" | grep -oP 'E=\K[0-9]+' || echo "?")
  printf "(E=%s, r16_depth=%s, r32_depth=%s, bt_depth=%s) ===\n" "$edges" "$t16_depth" "$t32_depth" "$bt_depth"

  printf "  %-12s %12s %12s %14s %10s\n" "Variant" "Itrs" "Time (us)" "Perf (MIPS)" "RSS (KB)"
  printf '  %.0s-' {1..64}
  echo ""

  FILES=()
  LABELS=()
  if [ -f "bench/bf_assoc_${sz}.hvm4" ]; then
    FILES+=("bench/bf_assoc_${sz}.hvm4")
    LABELS+=("assoc-list")
  fi
  FILES+=("bench/bf_trie16_${sz}.hvm4" "bench/bf_trie32_${sz}.hvm4")
  LABELS+=("radix-16" "radix-32")
  if [ -f "bench/bf_btrie_${sz}.hvm4" ]; then
    FILES+=("bench/bf_btrie_${sz}.hvm4")
    LABELS+=("lin-btrie")
  fi

  for idx in "${!FILES[@]}"; do
    f="${FILES[$idx]}"
    label="${LABELS[$idx]}"

    best_us=""
    best_itrs=""
    best_perf=""
    best_rss=""

    for ((r = 1; r <= RUNS; r++)); do
      t0=$(date +%s%N)
      raw=$("$HVM" "$f" -s 2>&1) || true
      t1=$(date +%s%N)
      wall_us=$(( (t1 - t0) / 1000 ))

      # Check for OOM
      if echo "$raw" | grep -q "Out of heap"; then
        best_itrs="OOM"
        best_us="-"
        best_perf="-"
        best_rss="-"
        break
      fi

      itrs=$(echo "$raw" | grep -oP 'Itrs: \K[0-9]+' || echo "0")
      perf=$(echo "$raw" | grep -oP 'Perf: \K[0-9.]+' || echo "0")
      rss=$( { /usr/bin/time -v "$HVM" "$f" > /dev/null; } 2>&1 | grep -oP 'Maximum resident set size.*: \K[0-9]+' || echo "0")

      if [ -z "$best_us" ] || [ "$wall_us" -lt "$best_us" ]; then
        best_us="$wall_us"
        best_itrs="$itrs"
        best_perf="$perf"
        best_rss="$rss"
      fi
    done

    printf "  %-12s %12s %12s %14s %10s\n" \
      "$label" "$best_itrs" "$best_us" "$best_perf" "$best_rss"
  done
  echo ""
done
