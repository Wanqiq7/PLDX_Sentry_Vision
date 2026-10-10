# Sentry Build and Startup

The production build requires ROS 2 Jazzy and a built
`pldx_vision_interfaces` and `ros_interfaces` packages. The scripts load `/opt/ros/jazzy/setup.bash`
and the sibling `../navi_minco_bit_jazzy/install-link/setup.bash`. Set `ROS_DISTRO` or
`PLDX_VISION_WS` to use another installation.

From the vision repository:

```bash
bash tools/build_sentry.sh
bash tools/run_sentry.sh configs/sentry.yaml
```

`BUILD_JOBS` defaults to 2. `VISION_BUILD_DIR` defaults to `build-link` under
the repository. Use the same override for building and running.

The build produces `sentry`, `sentry_bp`, `sentry_debug`, and
`sentry_multithread`. Missing production dependencies fail CMake configuration.
For standalone tests without ROS, explicitly configure a separate build tree
with `-DBUILD_SENTRY=OFF`; that tree cannot be used by the production launcher.

For background startup, use `bash autostart.sh configs/sentry.yaml`. This uses
the reference project's `screen` approach, runs `sentry`, and records output
under `logs/`. Attach with `screen -r pldx_sentry`. Exiting the program ends the
session; the launcher does not automatically restart it.

For desktop login startup, set the existing `.desktop` entry's `Exec` to the
absolute path of this repository's `autostart.sh`. The script locates the
repository independently of the desktop session's working directory.

`bash tools/run_sentry.sh --help` checks the environment and executable without
opening the camera or robot transport.

The single USB CDC owner is `GimbalRuntime`, selected by VID `16d0`, PID
`1492`, and interface `XRUSB CDC Control`. All four sentry entry points attach
`NavigationBridge` to this runtime. A second standalone CDC bridge must not run
alongside vision.

For a bounded check without cameras:

```bash
source tools/ros_env.sh
build-link/sentry_link_check configs/sentry.yaml 15
ctest --test-dir build-link --output-on-failure
```

The checker always sends `fire=false`; do not run navigation publishers during a
stationary hardware check. `/sentry/link_diagnostics` reports MCU receive counters
in the order `[nav_data, behavior_data, target_euler, fire_notify]`.

The bridge sends navigation and behavior at 100 Hz. Velocity and behavior input
expire after 500 ms. It converts ROS forward/left velocities to MCU right/forward
using a fresh `camera_init -> base_link` transform, and converts scan/tunnel angles
from degrees to radians. Missing/stale TF suppresses translation and x/y force;
missing/stale commands suppress all chassis velocity. Odometry and force inputs
also expire after 500 ms. `use_speed_control` stays true. Tunnel escape overrides
velocity; tunnel preparation suppresses movement until `tunnel_ready`.
`control_mode` remains under the MCU CMD/remote authority, and `gyro_vel` is not
substituted for `cmd_vel.angular.z`; `use_gyro_mode` is carried in the behavior frame.
Feedback topics retain the existing `ros_interfaces` message types and one USB owner.
