#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${1:-$ROOT/build/arm}"
cmake -S "$ROOT/app" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$BUILD" --parallel "${BUILD_JOBS:-1}"
ctest --test-dir "$BUILD" --output-on-failure
printf 'Source build and tests completed: %s\n' "$BUILD/adas_app"
