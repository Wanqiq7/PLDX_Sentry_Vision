#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"
set -u
BUILD_DIR="${VISION_BUILD_DIR:-${VISION_ROOT}/build-jazzy}"
if [[ ! -x "$BUILD_DIR/sentry" ]] ||
   ! grep -qx 'BUILD_SENTRY:BOOL=ON' "$BUILD_DIR/CMakeCache.txt"; then
    echo "Production build unavailable. Run bash tools/build_sentry.sh first." >&2
    exit 1
fi
cd "$VISION_ROOT"
exec "$BUILD_DIR/sentry" "$@"
