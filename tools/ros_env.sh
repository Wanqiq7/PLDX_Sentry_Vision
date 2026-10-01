#!/usr/bin/env bash
# Sourced by build and launch entry points; ROS setup files need nounset disabled.
VISION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -z "${PLDX_VISION_WS:-}" ]]; then
    if [[ -f "${VISION_ROOT}/../pldx_ws/install/setup.bash" ]]; then
        PLDX_VISION_WS="${VISION_ROOT}/../pldx_ws"
    elif [[ -f "${VISION_ROOT}/../navi_minco_bit_jazzy/install/pldx_vision_interfaces/share/pldx_vision_interfaces/package.xml" ]]; then
        # pldx_vision_interfaces is vendored in the navigation workspace.
        PLDX_VISION_WS="${VISION_ROOT}/../navi_minco_bit_jazzy"
    elif [[ -f "/opt/sentry/ros2/install/setup.bash" ]]; then
        PLDX_VISION_WS="/opt/sentry/ros2"
    else
        PLDX_VISION_WS="${VISION_ROOT}/../pldx_ws"
    fi
fi
if [[ ! -f "${PLDX_VISION_WS}/install/setup.bash" ]]; then
    echo "Missing interface workspace: ${PLDX_VISION_WS}/install/setup.bash" >&2
    return 1
fi
source "${PLDX_VISION_WS}/install/setup.bash"
ros2 pkg prefix pldx_vision_interfaces >/dev/null

# The repository ships the amd64 Hikrobot SDK; keep it local and opt-in via
# the environment instead of requiring a system-wide SDK installation.
HIKROBOT_LIB_DIR="${VISION_ROOT}/io/hikrobot/lib/amd64"
if [[ -d "${HIKROBOT_LIB_DIR}" ]]; then
    export LD_LIBRARY_PATH="${HIKROBOT_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
