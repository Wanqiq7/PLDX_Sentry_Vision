# LibXR SharedTopic Gimbal Communication Design

> **文档状态：历史设计记录。** 当前工程的编译验证基线为 Ubuntu 24.04、
> ROS 2 Jazzy、GCC 13 和 C++20。现行构建、运行及测试命令以仓库根目录
> `readme.md` 为准；本文保留设计决策，不作为部署手册。

## Goal

Replace the legacy `SP`/CRC16 protocol inside `io::Gimbal` with the exact
LibXR SharedTopic protocol used by `/home/sb/PLDX_Template` over its
`usb_otg_hs_cdc` link. Preserve the `io::Gimbal` business API and the unused
`io::CBoard` implementation. The first version exchanges exactly three topics:

- Vision to controller: `target_euler`
- Vision to controller: `fire_notify`
- Controller to vision: `ahrs_quaternion`

The controller firmware and `/home/sb/PLDX_Template` are out of scope and must
not be modified.

## Dependency And Build

Add `https://github.com/Jiu-Xiao/libxr.git` as a Git submodule at
`third_party/libxr`, pinned to the controller's exact commit:

```text
c512b364ab1f0646bb86c4929ac5877e1bc7b62d
```

The root CMake project will move to C++20, configure LibXR with the Linux
system and Linux driver, add the submodule directly, and link the LibXR target
into `io`. There will be no `libxr_topic` adapter directory and no copied
controller `SharedTopic` or `SharedTopicClient` module source.

The legacy serial library remains available to other modules, but
`io::Gimbal` will no longer depend on `serial::Serial`. `io::CBoard` remains in
the repository and build without being selected by production programs.

## Architecture

`io::Gimbal` remains the only business-facing API. Its implementation accesses
a minimal, process-wide LibXR communication runtime that owns:

- One `LibXR::LinuxUART` for the configured CDC device.
- Local `target_euler`, `fire_notify`, and `ahrs_quaternion` topics in the
  default LibXR domain.
- One `LibXR::Topic::Server` that parses incoming bytes and publishes the AHRS
  topic locally.
- A bounded outgoing bridge that packs local target/fire topic updates with
  LibXR and writes them to the same UART.
- The quaternion time mapping, sample queue, compatibility state, and link
  freshness state.

The runtime does not create `ApplicationManager` or `HardwareContainer`.
LibXR's topic registry is process-global and `LinuxUART` has perpetual worker
threads, so the runtime is created by the first `Gimbal` and intentionally
lives until process exit. A second simultaneous `Gimbal` instance is rejected.

```text
Vision business code
  -> Gimbal::send()
  -> target_euler / fire_notify local topics
  -> LibXR packet encoder and bounded transmit bridge
  -> LinuxUART -> USB CDC
  -> controller SharedTopic -> HostData

Controller MadgwickAHRS
  -> controller SharedTopicClient
  -> USB CDC -> LinuxUART
  -> Topic::Server -> ahrs_quaternion local topic
  -> timestamp mapping and quaternion queue
  -> Gimbal::q(t) / imu_at(t)
```

## Wire Contract

The implementation uses LibXR's packet implementation rather than duplicating
the codec. At the pinned commit, each packet is:

```text
offset  size  field
0       1     prefix = 0x5A
1       3     payload length, little-endian uint24
4       4     CRC32 of topic name
8       6     timestamp in microseconds, little-endian uint48
14      1     version = 0x01
15      1     header CRC8
16      N     payload
16 + N  1     full-packet CRC8
```

The fixed non-payload overhead is 17 bytes. Timestamps exist only in this
header and are not duplicated in any payload.

### `target_euler`

The vision-side DTO mirrors `HostData::HostGimbalTarget` and has nine
contiguous `float` fields, for an asserted size of 36 bytes:

```text
rol, pit, yaw,
rol_dot, pit_dot, yaw_dot,
rol_ddot, pit_ddot, yaw_ddot
```

The mapping is:

```text
rol       = 0
pit       = vision_pitch
yaw       = vision_yaw
rol_dot   = 0
pit_dot   = vision_pitch_velocity
yaw_dot   = vision_yaw_velocity
rol_ddot  = 0
pit_ddot  = vision_pitch_acceleration
yaw_ddot  = vision_yaw_acceleration
```

Angles use radians, angular velocities use radians per second, and angular
accelerations use radians per second squared.

### `fire_notify`

The vision-side DTO mirrors `HostData::LauncherCMD`:

```cpp
struct FirePayload {
  bool isfire;
};
```

Its size is asserted to be one byte. `Gimbal::send()` publishes this topic on
every caller cycle with `isfire = control && fire`.

### `ahrs_quaternion`

The controller publishes `LibXR::Quaternion<float>`. Although its constructor
and accessors use `(w, x, y, z)`, the pinned Eigen implementation stores the
object's four raw floats as `(x, y, z, w)`. SharedTopic transports the object's
raw bytes, so the asserted 16-byte payload is decoded as `xyzw` and converted
to `Eigen::Quaterniond(w, x, y, z)`.

This raw layout must be protected by a known-value byte test. A future LibXR or
Eigen update cannot be accepted without rerunning that compatibility test.

## Control Semantics

The new target payload has no equivalent of the legacy `control` flag.
Publishing any target makes `HostData` mark the AI gimbal input online.
Therefore:

- When `control` is true, publish `target_euler` on every caller cycle.
- When `control` is false, do not publish `target_euler`.
- Continue publishing `fire_notify=false` while control is false.
- Let the controller's existing 150 ms `HostData` timeout mark the AI gimbal
  target offline and fall back to its other control source.

The caller remains responsible for the approximately 100 Hz call rate. The
communication runtime does not invent target updates on a separate timer.

## Time Mapping And Quaternion Queue

The AHRS packet timestamp is controller uptime in microseconds, while camera
timestamps use the vision host's `steady_clock`. On the first valid AHRS packet,
the runtime establishes an offset between the controller timestamp and the
local receive time. Later samples use the packet timestamp plus that offset,
so packet timing, rather than USB arrival jitter, determines interpolation.

A controller timestamp regression indicates a controller restart or clock
reset. The runtime then clears all quaternion samples, establishes a new
offset, and waits for enough new samples before serving interpolation again.
The same recovery state begins after 150 ms without a valid AHRS update. The
first valid packet after that gap always clears the old queue and rebuilds the
clock offset, even when its controller timestamp did not regress.

Startup and link loss are fail-closed: construction waits for the first valid
quaternion, and `q(t)` blocks when it cannot obtain the required valid samples.
It never continues aiming with stale attitude. LibXR's Linux UART worker keeps
retrying the configured device path. Fresh AHRS packets after reconnect reset
the queue/time mapping and automatically resume operation.

## Compatibility State

The three-topic scope does not provide the mode, gimbal Euler state, angular
velocity, bullet speed, or bullet count that existed in the legacy packet.
The existing public API remains source-compatible and reports:

- Configured default mode, defaulting to `AUTO_AIM`.
- Configured bullet speed, defaulting to `23.0 m/s`.
- Zero for the remaining gimbal state fields and bullet count.
- The existing fixed shoot mode value.

The runtime emits one startup warning that these values are compatibility
defaults rather than controller feedback. Programs that require live mode
switching or bullet count feedback are not fully supported by this first
three-topic version.

## Backpressure And Reconnect Safety

The outgoing bridge permits one packet in flight and retains only the latest
pending value for each outgoing topic. Repeated updates replace pending values;
they never create an unbounded history. After AHRS freshness is lost, the
latest pending fire state is forced to false and neither target nor true fire
updates resume until a fresh AHRS packet arrives.

This prevents a series of stale target/fire packets from being replayed after
reconnect. One packet already handed to the kernel or LibXR driver at the
instant of physical disconnection cannot be withdrawn. That is an explicit
residual risk of the existing protocol, which has no acknowledgement or
sequence mechanism. The controller's RC fire interlock and 150 ms HostData
timeout remain additional safety controls.

## Configuration

`Gimbal` reads these top-level YAML keys:

```yaml
com_port: /dev/gimbal
baudrate: 921600
default_mode: AUTO_AIM
default_bullet_speed: 23.0
```

`com_port` is required. The other keys remain optional for backward
compatibility and use the values shown above when absent. Existing project
configuration files will explicitly include all three optional keys so deployed
behavior does not depend on an invisible default.

Unknown mode strings, an invalid baud rate, or a non-positive default bullet
speed are startup configuration errors. They are not silently corrected.

## Error Handling

- `Topic::Server` is responsible for stream resynchronization and rejection of
  invalid prefix, version, unknown topic CRC32, header CRC8, and packet CRC8.
- Payload sizes are fixed by typed local topics and compile-time assertions.
- AHRS quaternions with a non-finite component or norm at or below `1e-6` are
  rejected. Every other finite quaternion is normalized before entering the
  interpolation queue. Norm deviation is rate-limited in logs but is not a
  hard rejection condition.
- UART open/read/write failures are logged, and LibXR retries the stable device
  path.
- Transmit queue saturation coalesces to the latest topic value instead of
  blocking the vision loop or retaining old commands.
- Loss of AHRS freshness disables new target/true-fire transmission and leaves
  attitude consumers blocked until recovery.

At the pinned commit, `Topic::Server` does not require the packet header's
payload length to equal the registered topic payload size. A deliberately
formed packet with a valid CRC8 and a wrong length may therefore reach typed
dispatch. The only peer on this point-to-point CDC link is the trusted
controller, and random line corruption is covered by CRC8. The design accepts
this upstream behavior rather than adding a second custom stream parser or
modifying the pinned submodule. Quaternion finite-value, nonzero-norm, and
timestamp checks provide the final semantic gate. This is a documented
residual risk, not a claimed length-rejection guarantee.

## Verification

### Protocol Tests

Use the pinned LibXR implementation and fixed byte vectors to verify:

- All three payload sizes and field order.
- Topic-name CRC32 values.
- The 16-byte header and 17-byte fixed overhead.
- Header and full-packet CRC8 behavior.
- Little-endian uint24 length and uint48 timestamp.
- The raw quaternion `xyzw` layout and conversion to `wxyz` constructor order.
- Rejection/resynchronization for corrupt, fragmented, and concatenated input.
- A regression test that records LibXR's current handling of a valid-CRC packet
  whose declared payload length differs from the registered topic size.

### Behavior Tests

Verify control/fire mapping, the 150 ms target silence policy, the 150 ms AHRS
freshness threshold, timestamp reset and post-gap clock remapping,
finite/nonzero quaternion validation and normalization, fail-closed quaternion
behavior, latest-value coalescing, and compatibility defaults.

### Pseudo-Terminal Integration

Exercise both directions against a pseudo-terminal using the actual LibXR Linux
transport. Cover approximately 100 Hz traffic, fragmented and concatenated
packets, disconnect/reconnect, and absence of queued historical command replay.

### Build Verification

Configure and build all existing production and test targets under C++20. Run
all hardware-independent CTest tests. Generated `build-serial/`, `build-typed/`,
and unrelated untracked files must not enter commits.

### Hardware Safety Gate

The default `/dev/gimbal` smoke test may receive `ahrs_quaternion` and transmit
only `fire_notify=false`; it must not transmit target commands. A target-control
test requires a separate, immediate confirmation that the launcher is disabled
and the gimbal motion envelope is safe. Hardware tests never transmit
`fire_notify=true`.

## Scope Limits

- No controller firmware changes.
- No `yawmotor_angle`, `sentry_state`, chassis, live mode, bullet speed, or
  bullet count topic in the first version.
- No `ApplicationManager` or `HardwareContainer` integration.
- No CAN fallback and no deletion of the preserved `io::CBoard` implementation.
- No changes to unrelated generated or user-owned files.
