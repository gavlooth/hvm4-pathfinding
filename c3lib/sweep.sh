#!/bin/bash
# HVM4 Pathfinding Benchmark Sweep
# Runs each algorithm at multiple scales, measuring HVM4 interactions/sec.
# Each (algorithm, V) runs as a separate process so OOMs don't kill the sweep.

set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BENCH="$SCRIPT_DIR/build/bench"

export HVM4_THREADS=1

echo "============================================================"
echo "HVM4 Pathfinding Benchmark Sweep"
echo "Date: $(date)"
echo "HVM4_THREADS=$HVM4_THREADS"
echo "============================================================"
echo ""

run_sweep() {
    local algo=$1
    local epn=$2
    shift 2
    local scales=("$@")

    echo "--- $algo (epn=$epn) ---"
    printf "%-6s  %-12s  %-8s  %14s  %10s  %10s  %10s\n" \
           "STAT" "ALGO" "V" "INTERACTIONS" "TIME_MS" "MIPS" "RSS_KB"
    for v in "${scales[@]}"; do
        result=$("$BENCH" "$algo" "$v" "$epn" 2>/dev/null)
        rc=$?
        if [ $rc -eq 0 ]; then
            echo "$result"
        elif [ $rc -eq 137 ] || [ $rc -eq 134 ]; then
            printf "OOM    %-12s  V=%-6s  (killed)\n" "$algo" "$v"
        else
            # Capture stderr too for OOM message
            result2=$("$BENCH" "$algo" "$v" "$epn" 2>&1)
            if echo "$result2" | grep -q "Out of heap"; then
                printf "OOM    %-12s  V=%-6s  (heap exhausted)\n" "$algo" "$v"
            else
                printf "CRASH  %-12s  V=%-6s  (exit=%d)\n" "$algo" "$v" "$rc"
            fi
        fi
    done
    echo ""
}

# Build first
echo "Building..."
cd "$SCRIPT_DIR" && c3c build bench 2>&1 | grep "^Program"
echo ""

# ---- FFI-graph algorithms (can scale higher) ----
run_sweep bf        4   50 100 200 500 1000 2000 3000 4000
run_sweep ds        4   50 100 200 500 1000 2000 3000 4000
run_sweep ch        4   50 100 200 500 1000 2000 3000
run_sweep bidir_bfs 4   50 100 200 500 1000 2000 3000 4000

# ---- Matrix algorithms (O(V^3), limited scale) ----
run_sweep apsp      0   5 8 10 12 15
run_sweep closure   0   5 8 10 12 15 20

# ---- Path enumeration (depends on graph structure + eval_collapse) ----
run_sweep path_enum 8   20 50 100 200 500

# ---- DAG-DP (graph embedded in source) ----
run_sweep dag_dp    4   20 50 100 200 500

echo "============================================================"
echo "Sweep complete."
echo "============================================================"
