#!/usr/bin/env bash
# Sourced by build and launch entry points; ROS setup files need nounset disabled.
VISION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -z "${PLDX_VISION_WS:-}" ]]; then
    if [[ -f "${VISION_ROOT}/../navi_minco_bit_jazzy/install-link/setup.bash" ]]; then
        PLDX_VISION_WS="${VISION_ROOT}/../navi_minco_bit_jazzy"
    elif [[ -f "${VISION_ROOT}/../pldx_ws/install/setup.bash" ]]; then
        PLDX_VISION_WS="${VISION_ROOT}/../pldx_ws"
    elif [[ -f "/opt/sentry/ros2/install/setup.bash" ]]; then
        PLDX_VISION_WS="/opt/sentry/ros2"
    else
        PLDX_VISION_WS="${VISION_ROOT}/../pldx_ws"
    fi
fi
VISION_INTERFACE_SETUP="${PLDX_VISION_WS}/install/setup.bash"
if [[ -f "${PLDX_VISION_WS}/install-link/setup.bash" ]]; then
    VISION_INTERFACE_SETUP="${PLDX_VISION_WS}/install-link/setup.bash"
fi
if [[ ! -f "$VISION_INTERFACE_SETUP" ]]; then
    echo "Missing interface workspace: ${PLDX_VISION_WS}/install/setup.bash" >&2
    return 1
fi
source "$VISION_INTERFACE_SETUP"
ros2 pkg prefix pldx_vision_interfaces >/dev/null

# Prefer one complete SDK installation, including its USB transport library.
HIKROBOT_SDK_ROOT="${HIKROBOT_SDK_ROOT:-/opt/MVS}"
if [[ -f "${HIKROBOT_SDK_ROOT}/lib/64/libMvCameraControl.so" ]]; then
    HIKROBOT_LIB_DIR="${HIKROBOT_SDK_ROOT}/lib/64"
    export MVCAM_SDK_PATH="${HIKROBOT_SDK_ROOT}"
    export MVCAM_COMMON_RUNENV="${HIKROBOT_SDK_ROOT}/lib"
else
    HIKROBOT_LIB_DIR="${VISION_ROOT}/io/hikrobot/lib/amd64"
fi
if [[ -d "${HIKROBOT_LIB_DIR}" ]]; then
    export LD_LIBRARY_PATH="${HIKROBOT_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
