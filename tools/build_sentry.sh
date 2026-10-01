#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"
set -u
BUILD_DIR="${VISION_BUILD_DIR:-${VISION_ROOT}/build-jazzy}"
cmake -S "$VISION_ROOT" -B "$BUILD_DIR" -DBUILD_SENTRY=ON
cmake --build "$BUILD_DIR" --parallel "${BUILD_JOBS:-2}" --target \
    sentry sentry_bp sentry_debug sentry_multithread
