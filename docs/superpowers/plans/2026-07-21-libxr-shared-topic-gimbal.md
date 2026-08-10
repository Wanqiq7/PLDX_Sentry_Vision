# LibXR SharedTopic Gimbal Communication Implementation Plan

> **文档状态：历史实施计划。** 当前工程的编译验证基线为 Ubuntu 24.04、
> ROS 2 Jazzy、GCC 13 和 C++20，统一使用 `build-jazzy` 作为本地验证目录。
> 下文的任务步骤和中间构建目录仅用于追溯实现过程；现行命令以仓库根目录
> `readme.md` 为准。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax and must be completed in order.

**Goal:** Replace the legacy `io::Gimbal` SP/CRC16 transport with the pinned LibXR SharedTopic protocol over USB CDC while preserving the business API and unused `io::CBoard` implementation.

**Architecture:** Add LibXR as a `third_party/libxr` submodule and compile it for Linux/C++20. Keep protocol DTOs, timestamped AHRS interpolation, and latest-value transmit coalescing in small testable components; let a process-wide `GimbalRuntime` own one `LinuxUART`, three local Topics, the LibXR packet server, and the async write bridge. `io::Gimbal` remains a thin business facade.

**Tech Stack:** C++20, CMake, pinned LibXR Linux system/driver, Eigen, yaml-cpp, plain-main C++ tests, Linux pseudo-terminals, `/dev/gimbal` USB CDC.

## Global Constraints

- Pin `third_party/libxr` to `c512b364ab1f0646bb86c4929ac5877e1bc7b62d`.
- Use LibXR Topic encoding/decoding; do not copy controller modules or create `io/libxr_topic`.
- Configure `LIBXR_SYSTEM=Linux`, `LIBXR_DRIVER=Linux`, and C++20.
- Exchange only `target_euler`, `fire_notify`, and `ahrs_quaternion`.
- Decode raw AHRS floats as Eigen `xyzw`, then construct `Eigen::Quaterniond(w,x,y,z)`.
- `control=false` stops target publication; fire is always `control && fire`.
- AHRS freshness is 150 ms; recovery clears samples and rebuilds the clock offset.
- Reject only non-finite or norm `<=1e-6` quaternions; normalize other finite values.
- Use one process-wide runtime and one `io::Gimbal` per process.
- Keep `/home/sb/PLDX_Template` unchanged and preserve `io::CBoard`.
- Never transmit `fire_notify=true` in automated hardware tests.
- Do not commit generated build directories or unrelated user files.

## File Map

- Create `third_party/libxr` submodule and `.gitmodules` entry.
- Create `io/gimbal/libxr_protocol.hpp` for DTOs and layout helpers.
- Create `io/gimbal/ahrs_timeline.hpp/.cpp` for clock mapping and interpolation.
- Create `io/gimbal/outgoing_bridge.hpp/.cpp` for latest-value scheduling.
- Create `io/gimbal/gimbal_runtime.hpp/.cpp` for UART, Topics, and workers.
- Create `io/gimbal/gimbal_config.hpp/.cpp` for validated YAML defaults.
- Modify `io/gimbal/gimbal.hpp/.cpp` to delegate the existing facade.
- Create protocol, timeline, bridge, and PTY tests under `tests/`.
- Create `tests/libxr_cdc_smoke.cpp` as the only default physical-device smoke tool.
- Modify CMake, matching YAML configs, and `readme.md`.

---

## Task 1: Add LibXR And Lock Wire DTOs

**Files:**
- Create: `.gitmodules`, `third_party/libxr`, `io/gimbal/libxr_protocol.hpp`, `tests/libxr_protocol_test.cpp`
- Modify: `CMakeLists.txt`, `io/CMakeLists.txt`

**Interfaces:**
- Produces `TargetEulerPayload`, `FirePayload`, `QuaternionPayload`, `EncodeTarget`, and `DecodeQuaternion` in `io::libxr_protocol`.

- [ ] **Step 1: Add and pin the submodule.**

```bash
git submodule add https://github.com/Jiu-Xiao/libxr.git third_party/libxr
git -C third_party/libxr checkout c512b364ab1f0646bb86c4929ac5877e1bc7b62d
```

Expected: `git -C third_party/libxr rev-parse HEAD` prints the pinned commit.

- [ ] **Step 2: Configure C++20 and LibXR.**

Set `CMAKE_CXX_STANDARD 20`, then add before `add_subdirectory(io)`:

```cmake
set(LIBXR_SYSTEM Linux CACHE STRING "LibXR host system" FORCE)
set(LIBXR_DRIVER Linux CACHE STRING "LibXR host driver" FORCE)
add_subdirectory(third_party/libxr EXCLUDE_FROM_ALL)
```

Link `io` publicly to `xr`. Keep the existing `serial` target because unrelated legacy code still builds.

Register the test target:

```cmake
add_executable(libxr_protocol_test tests/libxr_protocol_test.cpp)
target_link_libraries(libxr_protocol_test Eigen3::Eigen xr)
add_test(NAME libxr_protocol_test COMMAND libxr_protocol_test)
```

- [ ] **Step 3: Write the failing DTO layout test.**

```cpp
static_assert(sizeof(io::libxr_protocol::TargetEulerPayload) == 36);
static_assert(sizeof(io::libxr_protocol::FirePayload) == 1);
static_assert(sizeof(io::libxr_protocol::QuaternionPayload) == 16);
const io::libxr_protocol::QuaternionPayload raw{2.0F, 3.0F, 4.0F, 1.0F};
const auto q = io::libxr_protocol::DecodeQuaternion(raw);
assert(q.w() == 1.0 && q.x() == 2.0 && q.y() == 3.0 && q.z() == 4.0);
```

Also assert all nine target fields and `FirePayload{true}.isfire`. Pack a real `QuaternionPayload` with LibXR and assert prefix `0x5A`, version `1`, payload length `16`, 17-byte overhead, topic CRC32, timestamp, header CRC8, and packet CRC8. Construct a valid-CRC short-payload variant and record that the pinned `Topic::Server` invokes its callback despite the declared-size mismatch; this is a regression test of the accepted upstream behavior.

- [ ] **Step 4: Implement the exact DTOs.**

```cpp
namespace io::libxr_protocol {
struct TargetEulerPayload {
  float rol, pit, yaw, rol_dot, pit_dot, yaw_dot, rol_ddot, pit_ddot, yaw_ddot;
};
struct FirePayload { bool isfire = false; };
struct QuaternionPayload { float x, y, z, w; };
static_assert(sizeof(TargetEulerPayload) == 36);
static_assert(sizeof(FirePayload) == 1);
static_assert(sizeof(QuaternionPayload) == 16);
Eigen::Quaterniond DecodeQuaternion(const QuaternionPayload & raw);
}
```

`EncodeTarget` sets all roll terms to zero and copies pitch/yaw position, velocity, and acceleration without unit conversion.

- [ ] **Step 5: Build and run the focused test.**

```bash
cmake -S . -B build-libxr -DCMAKE_BUILD_TYPE=Release
cmake --build build-libxr --target libxr_protocol_test -j2
ctest --test-dir build-libxr -R libxr_protocol_test --output-on-failure
```

Expected: PASS and LibXR builds as a Linux static library.

- [ ] **Step 6: Commit Task 1.**

```bash
git add .gitmodules third_party/libxr CMakeLists.txt io/CMakeLists.txt io/gimbal/libxr_protocol.hpp tests/libxr_protocol_test.cpp
git commit -m "feat: add LibXR SharedTopic wire contract"
```

## Task 2: Implement AHRS Timestamp State

**Files:**
- Create: `io/gimbal/ahrs_timeline.hpp`, `io/gimbal/ahrs_timeline.cpp`, `tests/ahrs_timeline_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- `enum class SampleResult { REJECTED, ACCEPTED, RECOVERED };`
- `AhrsTimeline::OnSample(uint64_t, Eigen::Quaterniond, Clock::time_point)` returns `SampleResult`.
- `AhrsTimeline::MarkStale(Clock::time_point)` returns true only on the fresh-to-stale transition; `WaitAt`, `Fresh`, and `Reset` support runtime behavior.

- [ ] **Step 1: Write deterministic failing tests.**

Use synthetic steady-clock points to assert midpoint SLERP, remote timestamp regression reset, 150 ms freshness expiry, post-gap remapping, rejection of NaN/zero norm, and normalization of a finite non-unit quaternion.

```cpp
AhrsTimeline timeline;
timeline.OnSample(1000, Eigen::Quaterniond::Identity(), t0);
timeline.OnSample(2000, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())), t0 + 1ms);
const auto mid = timeline.WaitAt(t0 + 500us);
assert(mid.angularDistance(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()))) < 1e-9);
```

- [ ] **Step 2: Implement bounded, blocking interpolation.**

Store at most 1000 samples in a mutex-protected `std::deque`. Map local time as `local_anchor + (remote_us - remote_anchor)`. First sample, timestamp regression, or first sample after a 150 ms stale transition clears the deque and creates a new anchor. Reject any non-finite component or norm `<=1e-6`; normalize every accepted quaternion. `WaitAt` blocks until valid samples bracket the requested time, then returns normalized SLERP; it never returns stale samples.

- [ ] **Step 3: Run and commit the focused test.**

Register it before building:

```cmake
add_executable(ahrs_timeline_test tests/ahrs_timeline_test.cpp io/gimbal/ahrs_timeline.cpp)
target_link_libraries(ahrs_timeline_test Eigen3::Eigen)
add_test(NAME ahrs_timeline_test COMMAND ahrs_timeline_test)
```

```bash
cmake --build build-libxr --target ahrs_timeline_test -j2
ctest --test-dir build-libxr -R ahrs_timeline_test --output-on-failure
git add io/gimbal/ahrs_timeline.hpp io/gimbal/ahrs_timeline.cpp tests/ahrs_timeline_test.cpp CMakeLists.txt
git commit -m "feat: add timestamped AHRS timeline"
```

## Task 3: Implement Latest-Value Scheduling

**Files:**
- Create: `io/gimbal/outgoing_bridge.hpp`, `io/gimbal/outgoing_bridge.cpp`, `tests/outgoing_bridge_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- `Submit(TopicKind, std::span<const uint8_t>)` replaces one topic's pending packet.
- `BeginNext()` moves one packet to in-flight; `Complete(bool)` releases it.
- `ForceSafeFire(std::span<const uint8_t>)` removes target and replaces fire with false.

- [ ] **Step 1: Write failing coalescing tests.**

Submit `T1`, then `T2`, and assert the next target is `T2`. Assert no second packet starts before `Complete`. Assert `ForceSafeFire` removes a target and replaces a true-fire packet. Assert failed completion never replays the in-flight bytes.

- [ ] **Step 2: Implement the mutex-protected state machine.**

Use owned byte vectors for two optional pending packets and one optional in-flight packet. `Submit` replaces and never blocks. Preserve packet priority as safe fire first, then target, then ordinary fire so recovery cannot send a target ahead of the false-fire update.

- [ ] **Step 3: Run and commit.**

Register it before building:

```cmake
add_executable(outgoing_bridge_test tests/outgoing_bridge_test.cpp io/gimbal/outgoing_bridge.cpp)
add_test(NAME outgoing_bridge_test COMMAND outgoing_bridge_test)
```

```bash
cmake --build build-libxr --target outgoing_bridge_test -j2
ctest --test-dir build-libxr -R outgoing_bridge_test --output-on-failure
git add io/gimbal/outgoing_bridge.hpp io/gimbal/outgoing_bridge.cpp tests/outgoing_bridge_test.cpp CMakeLists.txt
git commit -m "feat: coalesce outgoing SharedTopic packets"
```

## Task 4: Build The Process-Wide LibXR Runtime

**Files:**
- Create: `io/gimbal/gimbal_runtime.hpp`, `io/gimbal/gimbal_runtime.cpp`
- Modify: `io/CMakeLists.txt`

**Interfaces:**
- `RuntimeConfig` carries device, baudrate, default mode, and default bullet speed.
- `GimbalRuntime::Instance(const RuntimeConfig&)` creates the sole runtime.
- Runtime methods are `SendTarget`, `SendFire`, `WaitReady`, `WaitQuaternion`, `Snapshot`, and `HasFreshAhrs`.

- [ ] **Step 1: Add a compile-only singleton skeleton.**

Define `GimbalRuntime` in the `io` namespace, delete copy/move operations, and keep the constructed instance for process lifetime. A second acquisition while a `Gimbal` facade is active throws `std::logic_error`. Do not instantiate `ApplicationManager` or `HardwareContainer`.

- [ ] **Step 2: Construct the exact Linux UART and local Topics.**

```cpp
uart_ = std::make_unique<LibXR::LinuxUART>(
  config.device.c_str(), config.baudrate, LibXR::UART::Parity::NO_PARITY,
  8, 1, 1, 512);
target_topic_ = LibXR::Topic::CreateTopic<TargetEulerPayload>("target_euler");
fire_topic_ = LibXR::Topic::CreateTopic<FirePayload>("fire_notify");
ahrs_topic_ = LibXR::Topic::CreateTopic<QuaternionPayload>("ahrs_quaternion");
server_.Register(ahrs_topic_);
```

The write queue depth is one. LibXR Linux system static initialization supplies its monotonic Timebase.

- [ ] **Step 3: Implement the receive loop.**

Use `ReadOperation wait_op(rx_sem_)` to arm reads. While `uart_->read_port_->Size() > 0`, read at most 512 bytes and call `server_.ParseData`. Register an AHRS raw Topic callback, copy the 16-byte view into `QuaternionPayload`, decode it, and inspect `AhrsTimeline::OnSample(static_cast<uint64_t>(message.timestamp), q, Clock::now())`. `RECOVERED` re-enables target/true-fire submission; `REJECTED` changes no runtime state. When an accepted input norm differs from 1 by more than `1e-3`, normalize it and emit at most one warning per second.

- [ ] **Step 4: Implement Topic-to-UART transmission.**

Register raw callbacks on `target_euler` and `fire_notify`. Each callback owns a buffer of `message.payload.size_ + Topic::PACK_BASE_SIZE`, calls `Topic(topic_handle).PackRaw(message.payload, packet, message.timestamp)`, and submits it to `OutgoingBridge`. A TX worker calls `BeginNext`, retains the returned bytes in a member until completion, writes them using a guarded `WriteOperation`, then calls `Complete(status == ErrorCode::OK)`.

- [ ] **Step 5: Implement stale/recovery gating.**

Every 10 ms call `AhrsTimeline::MarkStale(Clock::now())`. When it returns true, remove pending target and submit a packed false-fire packet through `ForceSafeFire`. Do not accept new target or true-fire updates until `OnSample` returns `RECOVERED`.

- [ ] **Step 6: Compile and commit the runtime.**

```bash
cmake --build build-libxr --target io -j2
git add io/gimbal/gimbal_runtime.hpp io/gimbal/gimbal_runtime.cpp io/CMakeLists.txt
git commit -m "feat: add LibXR Linux gimbal runtime"
```

Expected: `io` links `xr` and `libudev`; runtime buffers remain alive until async write completion.

## Task 5: Migrate The `io::Gimbal` Facade

**Files:**
- Modify: `io/gimbal/gimbal.hpp`, `io/gimbal/gimbal.cpp`
- Create: `io/gimbal/gimbal_config.hpp`, `io/gimbal/gimbal_config.cpp`, `tests/gimbal_config_test.cpp`
- Modify: every `configs/*.yaml` containing `com_port`
- Modify: `readme.md`

**Interfaces:**
- Preserve `mode`, `state`, `q`, `imu_at`, `legacy_mode`, `bullet_speed_value`, `shoot_mode_value`, and both `send` overloads.
- Remove `GimbalToVision` and the old wire-only storage. Retain `VisionToGimbal` as a source-compatible business command type because `tests/fire_test.cpp` uses it.

- [ ] **Step 1: Add a testable configuration parser.**

Define `RuntimeConfig LoadGimbalConfig(const std::string& path)`. Parse `com_port` as required. Use defaults `baudrate=921600`, `default_mode=AUTO_AIM`, and `default_bullet_speed=23.0`. Reject unknown mode strings, baudrate outside `1..4000000`, and non-positive bullet speed with `std::invalid_argument` after logging. `gimbal_config_test` writes YAML fixtures in its CTest working directory and covers every default and rejection branch.

Register and run the parser test before facade edits:

```cmake
add_executable(gimbal_config_test tests/gimbal_config_test.cpp io/gimbal/gimbal_config.cpp)
target_link_libraries(gimbal_config_test yaml-cpp)
add_test(NAME gimbal_config_test COMMAND gimbal_config_test)
```

```bash
cmake --build build-libxr --target gimbal_config_test -j2
ctest --test-dir build-libxr -R gimbal_config_test --output-on-failure
```

- [ ] **Step 2: Replace legacy framing with runtime delegation.**

Remove `serial/serial.h`, `serial_`, `GimbalToVision`, wire headers/CRC fields, CRC16 calls, Gimbal read thread, and reconnect method. Keep `VisionToGimbal` fields `mode`, yaw/pitch position, velocity, and acceleration; map mode 0 to no control, mode 1 to control without fire, and mode 2 to control with fire. Constructor obtains `GimbalRuntime::Instance`, emits one compatibility warning, and calls `WaitReady()` for initial AHRS readiness. `q(t)` calls `WaitQuaternion(t)`.

Implement command mapping exactly as:

```cpp
if (control) runtime_->SendTarget(libxr_protocol::EncodeTarget(
  yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc));
runtime_->SendFire(control && fire);
```

- [ ] **Step 3: Preserve compatibility getters.**

Return configured default mode and bullet speed. Return zero yaw, pitch, both velocities, and bullet count. Keep the current fixed shoot mode. Document that MPC mode switching and bullet-count behavior are degraded in this three-topic version.

- [ ] **Step 4: Make configuration explicit.**

For every YAML containing `com_port`, ensure these keys occur once:

```yaml
baudrate: 921600
default_mode: AUTO_AIM
default_bullet_speed: 23.0
```

Update `readme.md` with the three Topic directions, `/dev/gimbal` as the controller `usb_otg_hs_cdc` endpoint, the pinned submodule command, and the no-true-fire automated-test rule.

- [ ] **Step 5: Build all configured targets and commit.**

```bash
cmake -S . -B build-libxr -DCMAKE_BUILD_TYPE=Release
cmake --build build-libxr -j2
git add io/gimbal/gimbal.hpp io/gimbal/gimbal.cpp io/gimbal/gimbal_config.hpp io/gimbal/gimbal_config.cpp tests/gimbal_config_test.cpp CMakeLists.txt configs readme.md
git commit -m "refactor: route Gimbal through LibXR SharedTopic"
```

Expected: all targets compile as C++20, production callers are unchanged, and `io::CBoard` still builds.

## Task 6: Add Pseudo-Terminal Integration Coverage

**Files:**
- Create: `tests/libxr_pty_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- The test creates a PTY pair, runs the runtime against its slave path, and uses the master FD as the trusted controller peer.

- [ ] **Step 1: Create the PTY harness.**

Use `posix_openpt(O_RDWR | O_NOCTTY)`, `grantpt`, `unlockpt`, and `ptsname`. Configure the slave for raw 921600/8N1. Give each integration case its own process invocation because `GimbalRuntime` intentionally cannot be destroyed and recreated in-process.

- [ ] **Step 2: Test fragmented incoming AHRS.**

Use LibXR `Topic::PackData` to create a known `ahrs_quaternion` packet, write it to the master in fragments, send a second timestamped sample, then assert `WaitQuaternion` returns the expected normalized SLERP orientation.

- [ ] **Step 3: Test target and fire output.**

After AHRS readiness, call `send(true, false, ...)`, parse master-side bytes with a LibXR `Topic::Server`, and assert all nine target fields. Call `send(false, true, ...)`; assert a false fire packet appears and no target packet appears during the following controller timeout interval.

- [ ] **Step 4: Test stale recovery and latest-value behavior.**

Stop AHRS for at least 150 ms and assert new targets stop. Inject a post-gap AHRS packet without timestamp regression and assert old interpolation samples are discarded. Delay master reads while replacing multiple pending targets, then assert the latest pending target is the only retained target after the in-flight packet.

- [ ] **Step 5: Register and run integration tests.**

```cmake
add_executable(libxr_pty_test tests/libxr_pty_test.cpp)
target_link_libraries(libxr_pty_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
add_test(NAME libxr_pty_test COMMAND libxr_pty_test)
```

```bash
cmake --build build-libxr --target libxr_pty_test -j2
ctest --test-dir build-libxr -R "(libxr_protocol|ahrs_timeline|outgoing_bridge|libxr_pty)_test" --output-on-failure
git add tests/libxr_pty_test.cpp CMakeLists.txt
git commit -m "test: cover LibXR gimbal CDC bridge"
```

Expected: all tests run without a physical controller and never emit true fire.

## Task 7: Full Verification And Safe Hardware Smoke

**Files:**
- Create: `tests/libxr_cdc_smoke.cpp`
- Modify: `CMakeLists.txt`, `readme.md`
- Do not modify: `/home/sb/PLDX_Template`

- [ ] **Step 1: Check source and Git hygiene.**

```bash
git diff --check
rg -n "serial::Serial|crc16|GimbalToVision" io/gimbal
git status --short
```

Expected: no old transport symbol remains under `io/gimbal`; the source-compatible `VisionToGimbal` business command may remain, and unrelated untracked build directories remain uncommitted.

- [ ] **Step 2: Run full hardware-independent verification.**

```bash
cmake -S . -B build-libxr -DCMAKE_BUILD_TYPE=Release
cmake --build build-libxr -j2
ctest --test-dir build-libxr --output-on-failure
git submodule status third_party/libxr
```

Expected: all configured targets and CTest tests pass; submodule status reports `c512b364ab1f0646bb86c4929ac5877e1bc7b62d`.

- [ ] **Step 3: Build the passive-only hardware smoke tool.**

`libxr_cdc_smoke` accepts a config path, constructs `io::Gimbal`, receives and prints normalized AHRS at 10 Hz, and calls only `gimbal.send(false, false, 0, 0, 0, 0, 0, 0)`. It has no CLI option or code path that can set control or fire true. Register it as an executable but not a CTest test:

```cmake
add_executable(libxr_cdc_smoke tests/libxr_cdc_smoke.cpp)
target_link_libraries(libxr_cdc_smoke fmt::fmt yaml-cpp tools io)
```

- [ ] **Step 4: Run only the passive default hardware smoke.**

Confirm `/dev/gimbal` exists. Receive `ahrs_quaternion` and send only `fire_notify=false`; do not send target or true-fire packets. Unplug/replug CDC and confirm logging plus automatic recovery without a process crash.

- [ ] **Step 5: Gate target-control validation on fresh physical confirmation.**

Only after confirming the launcher is disabled and the motion envelope is clear, send bounded target commands at approximately 100 Hz with fire fixed false. Stop target publication and verify the controller falls back after 150 ms.

- [ ] **Step 6: Commit the smoke tool and documentation.**

```bash
git add tests/libxr_cdc_smoke.cpp CMakeLists.txt readme.md
git commit -m "test: add passive LibXR CDC smoke tool"
```

- [ ] **Step 7: Review commits and hand off residual risk.**

```bash
git status --short
git log --oneline --decorate -8
```

Do not add `build-libxr/`, `build-serial/`, `build-typed/`, or unrelated documents. Report that the pinned LibXR `Topic::Server` accepts valid-CRC packets whose declared length differs from the registered payload size; quaternion semantic checks mitigate but do not eliminate that upstream behavior.
