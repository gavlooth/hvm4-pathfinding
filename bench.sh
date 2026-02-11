#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

HVM=./HVM4/clang/main
RUNS="${1:-5}"

if [ ! -x "$HVM" ]; then
  echo "HVM4 not built. Run ./build.sh first."
  exit 1
fi

declare -A FLAGS
FLAGS[sup_enum]="-C10"

FILES=(src/path_*.hvm4)

printf "%-26s %10s %12s %14s %10s\n" \
  "Algorithm" "Itrs" "Time (us)" "Perf (MIPS)" "RSS (KB)"
printf '%.0s-' {1..76}
echo ""

for f in "${FILES[@]}"; do
  name=$(basename "$f" .hvm4)
  short=${name#path_}
  extra="${FLAGS[$short]:-}"

  best_us=""
  best_itrs=""
  best_perf=""
  best_result=""
  best_rss=""

  for ((r = 1; r <= RUNS; r++)); do
    # Wall-clock via nanosecond timestamps
    t0=$(date +%s%N)
    raw=$("$HVM" "$f" -s $extra 2>&1)
    t1=$(date +%s%N)
    wall_us=$(( (t1 - t0) / 1000 ))

    # Result lines (strip ANSI interaction counts)
    result=$(echo "$raw" | sed 's/ \x1b\[2m#[0-9]*\x1b\[0m//g' | grep -v '^- ' | tr '\n' ' ' | sed 's/ *$//')

    # Interactions
    itrs=$(echo "$raw" | grep -oP 'Itrs: \K[0-9]+' || echo "0")

    # Perf (MIPS) from HVM4
    perf=$(echo "$raw" | grep -oP 'Perf: \K[0-9.]+' || echo "0")

    # Peak RSS via /usr/bin/time -v
    rss=$( { /usr/bin/time -v "$HVM" "$f" $extra > /dev/null; } 2>&1 | grep -oP 'Maximum resident set size.*: \K[0-9]+' || echo "0")

    if [ -z "$best_us" ] || [ "$wall_us" -lt "$best_us" ]; then
      best_us="$wall_us"
      best_itrs="$itrs"
      best_perf="$perf"
      best_result="$result"
      best_rss="$rss"
    fi
  done

  printf "%-26s %10s %12s %14s %10s\n" \
    "$short" "$best_itrs" "$best_us" "$best_perf" "$best_rss"
done

echo ""
echo "Best of $RUNS runs. Time = wall-clock microseconds. Perf = M interactions/s (from HVM4). RSS = peak resident set (KB)."
