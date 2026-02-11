#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

echo "Building HVM4 runtime..."
cd HVM4/clang && clang -O2 -o main main.c && cd ../..

echo "Done. Binary at HVM4/clang/main"
