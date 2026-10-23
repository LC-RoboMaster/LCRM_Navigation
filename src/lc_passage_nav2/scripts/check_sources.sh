#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${TDT_NATIVE_BUILD_DIR:-$project_root/build-native}"
cmake -S "$project_root/test" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$build_dir" --parallel "${TDT_BUILD_JOBS:-2}"
ctest --test-dir "$build_dir" --output-on-failure
