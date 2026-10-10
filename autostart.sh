#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/tools/ros_env.sh"
set -u
cd "$VISION_ROOT"
bash "$VISION_ROOT/tools/run_sentry.sh" --help >/dev/null
mkdir -p logs
exec screen -L -Logfile "logs/$(date '+%Y-%m-%d_%H-%M-%S').screenlog" \
    -dmS pldx_sentry bash "$VISION_ROOT/tools/run_sentry.sh" "$@"
