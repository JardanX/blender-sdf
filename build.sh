#!/usr/bin/env bash
#
# Incremental Blender build helper for the SDF/NURB fork (Linux, Ninja).
#
# Usage:
#   ./build.sh                 Incremental build + install
#   ./build.sh -r              Build, then launch Blender on success
#   ./build.sh -r -- file.blend --debug   Build, then run with extra args
#   ./build.sh --reconfigure   Re-run CMake configure before building
#   ./build.sh -j 8            Override parallel job count
#
# Env overrides:
#   BUILD_DIR=/path/to/build   Use a different build directory
#
set -euo pipefail

# Repo root = directory containing this script.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build dir defaults to the sibling Ninja dir created by `make full ninja ccache`.
BUILD_DIR="${BUILD_DIR:-$(cd "$REPO_ROOT/.." && pwd)/build_linux_full}"

JOBS="$(nproc)"
RUN_AFTER=0
RECONFIGURE=0
BLENDER_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -r|--run)         RUN_AFTER=1; shift ;;
    --reconfigure)    RECONFIGURE=1; shift ;;
    -j|--jobs)        JOBS="$2"; shift 2 ;;
    --)               shift; BLENDER_ARGS=("$@"); break ;;
    -h|--help)        sed -n '2,15p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done

if [[ ! -f "$BUILD_DIR/build.ninja" ]]; then
  echo "error: no Ninja build at '$BUILD_DIR'." >&2
  echo "       Configure first with:  make full ninja ccache" >&2
  echo "       Or set BUILD_DIR=/path/to/your/build" >&2
  exit 1
fi

echo ">> Build dir: $BUILD_DIR"
echo ">> Jobs:      $JOBS"

if [[ "$RECONFIGURE" == 1 ]]; then
  echo ">> Reconfiguring (cmake)..."
  cmake "$BUILD_DIR" >/dev/null
fi

START=$SECONDS
echo ">> Building (ninja install)..."
ninja -C "$BUILD_DIR" -j "$JOBS" install
echo ">> Done in $((SECONDS - START))s."

BLENDER_BIN="$BUILD_DIR/bin/blender"
echo ">> Binary: $BLENDER_BIN"

if [[ "$RUN_AFTER" == 1 ]]; then
  echo ">> Launching Blender..."
  exec "$BLENDER_BIN" "${BLENDER_ARGS[@]}"
fi
