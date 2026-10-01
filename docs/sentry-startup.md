# Sentry Build and Startup

The production build requires ROS 2 Jazzy and a built
`pldx_vision_interfaces` package, which is built by the navigation workspace
(`navi_minco_bit_jazzy/src/interfaces`). The scripts load
`/opt/ros/jazzy/setup.bash` and then the first available of
`../pldx_ws/install/setup.bash`, `../navi_minco_bit_jazzy/install/setup.bash`
or `/opt/sentry/ros2/install/setup.bash`. Set `ROS_DISTRO` or
`PLDX_VISION_WS` to use another installation.

From the vision repository:

```bash
bash tools/build_sentry.sh
bash tools/run_sentry.sh configs/sentry.yaml
```

`BUILD_JOBS` defaults to 2. `VISION_BUILD_DIR` defaults to `build-jazzy` under
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
