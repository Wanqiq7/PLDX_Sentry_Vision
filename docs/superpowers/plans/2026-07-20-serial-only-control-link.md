# Serial-Only Control Link Implementation Plan

> **文档状态：已被 LibXR Topic 云台链路取代的历史方案。** 当前工程运行与
> 编译基线为 Ubuntu 24.04、ROS 2 Jazzy、GCC 13 和 C++20。本文中的 C++17、
> `build-serial` 和旧串口协议步骤不适用于现行工程；请使用仓库根目录
> `readme.md` 的 `build-jazzy` 构建与测试流程。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move every vision-to-controller telemetry and command path from SocketCAN to one versioned serial protocol, preserve all current CBoard and Gimbal features, then remove CAN code and deployment requirements.

**Architecture:** Introduce a single `io::SerialBoard` business API backed by a stream parser and the existing serial library. Both legacy `io::CBoard` users and MPC `io::Gimbal` users migrate to typed state/command objects; protocol encoding is isolated from transport so fragmentation, corruption, and field compatibility are unit-testable. Removal of CAN is gated on desktop tests, full builds, and a controller-firmware hardware acceptance run.

**Tech Stack:** C++17, CMake/CTest, Eigen, yaml-cpp, bundled `serial`, existing CRC16 helpers, Linux pseudo-terminal tests, STM32 controller firmware using the same byte-level contract.

## Global Constraints

- Scope is the controller link: IMU/gimbal telemetry and vision control commands must use serial. Camera USB and ROS2/DDS navigation links remain unchanged.
- Use `/dev/gimbal` as the stable udev symlink and `921600` baud, 8 data bits, no parity, one stop bit, no flow control.
- Preserve all current data: quaternion, mode including `OUTPOST`, shoot mode, bullet speed/count, gimbal yaw/pitch and velocities, `ft_angle`, control/fire, yaw/pitch position/velocity/acceleration, and UAV `horizon_distance`.
- Wire values are fixed-width, little-endian, IEEE-754 binary32 for floats; no C++ packed struct may be written directly to the wire.
- Every packet is at most 64 bytes and is protected by the repository's CRC16 implementation.
- No runtime fallback to CAN. Rollback is performed by deploying the previous known-good binary and controller firmware together.
- Do not modify unrelated existing changes in `configs/*.yaml`, `readme.md`, or generated/untracked artifacts.

---

## File Map

- Create `docs/serial-protocol-v2.md`: authoritative host/controller wire contract and rollout compatibility matrix.
- Create `io/serial_board/protocol.hpp` and `io/serial_board/protocol.cpp`: typed packet definitions, byte encoding, and incremental parsing only.
- Create `io/serial_board/serial_board.hpp` and `io/serial_board/serial_board.cpp`: serial lifecycle, receive thread, state snapshot, quaternion interpolation, reconnect, and command sending.
- Create `tests/serial_protocol_test.cpp`: golden vectors, fragmented input, corruption recovery, and invalid-field tests.
- Create `tests/serial_board_test.cpp`: pseudo-terminal integration tests for telemetry reception and command transmission.
- Modify controller firmware outside this repository: implement protocol V2 encode/decode and watchdog behavior before host migration reaches hardware.
- Modify all `src/*.cpp`, `calibration/capture.cpp`, and `tasks/auto_aim/multithread/commandgener.*` that reference `io::CBoard` or `io::Gimbal`.
- Modify communication tests: replace `tests/cboard_test.cpp`, `tests/gimbal_test.cpp`, `tests/gimbal_response_test.cpp`, `tests/fire_test.cpp`, `tests/planner_test.cpp`, and `tests/handeye_test.cpp` references with `io::SerialBoard`.
- Modify `configs/*.yaml`: replace CAN keys and `com_port` with a single `serial_*` block.
- Modify `io/CMakeLists.txt`, root `CMakeLists.txt`, and `readme.md`: build/test/install documentation for serial only.
- Delete `io/cboard.hpp`, `io/cboard.cpp`, and `io/socketcan.hpp` only after the hardware gate passes.

---

### Task 1: Freeze Protocol V2 and Controller-Firmware Contract

**Files:**
- Create: `docs/serial-protocol-v2.md`
- Modify outside this repository: the STM32 controller communication module

**Interfaces:**
- Consumes: existing CRC16 algorithm from `tools/crc.cpp`
- Produces: packet types `0x01 TELEMETRY` and `0x02 COMMAND`, exact payloads consumed by Task 2

- [ ] **Step 1: Write the protocol document**

Document this exact frame layout:

```text
Header (8 bytes)
offset  size  field
0       2     magic = 0x53 0x50 (ASCII "SP")
2       1     version = 0x02
3       1     packet_type (0x01 telemetry, 0x02 command)
4       2     payload_length, little-endian
6       2     sequence, little-endian, wraps modulo 65536

Trailer (2 bytes)
CRC16 over header + payload, little-endian
Maximum payload_length = 54; maximum complete frame = 64 bytes.
```

Document telemetry payload `0x01` as exactly 48 bytes:

```text
uint8   mode          0 IDLE, 1 AUTO_AIM, 2 SMALL_BUFF, 3 BIG_BUFF, 4 OUTPOST
uint8   shoot_mode    0 LEFT, 1 RIGHT, 2 BOTH
uint16  status_flags  bit0 controller_ready; all other bits must be zero
float32 q_w, q_x, q_y, q_z
float32 yaw, yaw_vel, pitch, pitch_vel
float32 bullet_speed
uint16  bullet_count
uint16  reserved      must be zero
float32 ft_angle
```

Document command payload `0x02` as exactly 32 bytes:

```text
uint8   command_flags bit0 control, bit1 fire; all other bits must be zero
uint8   reserved8     must be zero
uint16  reserved16    must be zero
float32 yaw, yaw_vel, yaw_acc
float32 pitch, pitch_vel, pitch_acc
float32 horizon_distance
```

- [ ] **Step 2: Specify safety and timing behavior**

Add these normative requirements to the document:

```text
Controller transmits TELEMETRY at 500 Hz.
Host may transmit COMMAND at up to 500 Hz.
Controller disables vision control and firing if no valid COMMAND arrives for 100 ms.
Host reports disconnected if no valid TELEMETRY arrives for 100 ms.
Host never sets fire unless control is also set.
Parser discards unknown versions/types, invalid lengths, invalid enums, non-finite floats,
and invalid CRC frames, then resumes searching for the next SP magic pair.
On reconnect, host clears interpolation history and sends a zero/no-control COMMAND before
accepting new application commands.
```

- [ ] **Step 3: Implement the same contract in controller firmware**

Use fixed-width integers and explicit little-endian read/write helpers. Do not cast incoming bytes to a packed struct. Add compile-time assertions that telemetry payload is 48 bytes and command payload is 32 bytes, plus firmware tests using the golden frames produced in Task 2.

- [ ] **Step 4: Bench-check firmware fail-safe behavior**

Run the controller with actuators disabled. Verify 500 Hz telemetry, valid CRC, sequence increments, valid enums, command timeout at `100 +/- 10 ms`, and that a malformed or fire-only command cannot enable firing.

- [ ] **Step 5: Commit the repository protocol document**

```bash
git add docs/serial-protocol-v2.md
git commit -m "docs: define serial control protocol v2"
```

Expected: the host and firmware reviews refer to one byte-level specification.

---

### Task 2: Build and Test the Protocol Codec

**Files:**
- Create: `io/serial_board/protocol.hpp`
- Create: `io/serial_board/protocol.cpp`
- Create: `tests/serial_protocol_test.cpp`
- Modify: `io/CMakeLists.txt`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `tools::get_crc16(const uint8_t *, uint32_t)`
- Produces: `io::serial_board::Telemetry`, `Command`, `encode_command`, `encode_telemetry`, and `StreamParser::push`

- [ ] **Step 1: Write codec declarations and failing golden-vector tests**

Declare these public types and signatures:

```cpp
namespace io::serial_board {
enum class Mode : uint8_t { IDLE, AUTO_AIM, SMALL_BUFF, BIG_BUFF, OUTPOST };
enum class ShootMode : uint8_t { LEFT, RIGHT, BOTH };

struct Telemetry {
  Mode mode{Mode::IDLE};
  ShootMode shoot_mode{ShootMode::LEFT};
  bool controller_ready{false};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  float yaw{0}, yaw_vel{0}, pitch{0}, pitch_vel{0};
  float bullet_speed{0}, ft_angle{0};
  uint16_t bullet_count{0}, sequence{0};
};

struct Command {
  bool control{false}, fire{false};
  float yaw{0}, yaw_vel{0}, yaw_acc{0};
  float pitch{0}, pitch_vel{0}, pitch_acc{0};
  float horizon_distance{0};
};

using Packet = std::variant<Telemetry, Command>;
std::vector<uint8_t> encode_telemetry(const Telemetry &, uint16_t sequence);
std::vector<uint8_t> encode_command(const Command &, uint16_t sequence);

class StreamParser {
public:
  std::vector<Packet> push(const uint8_t * data, size_t size);
  void reset();
};
}
```

Tests must assert complete expected byte arrays for an all-zero telemetry frame and a representative command frame, including CRC bytes computed once and recorded in the test. They must also assert exact frame sizes `58` and `42`.

- [ ] **Step 2: Run tests and verify they fail**

```bash
cmake -S . -B build-serial
cmake --build build-serial --target serial_protocol_test -j"$(nproc)"
ctest --test-dir build-serial -R serial_protocol_test --output-on-failure
```

Expected: build fails because the protocol implementation does not exist.

- [ ] **Step 3: Implement explicit encoding and decoding**

Implement `append_u16_le`, `append_f32_le`, `read_u16_le`, and `read_f32_le` with shifts and `memcpy`; never serialize object memory. `StreamParser` must retain partial bytes, bound its buffer, resynchronize on `SP`, validate version/type/length/CRC, and emit only fully validated typed packets.

- [ ] **Step 4: Add parser recovery tests**

Cover one byte at a time, every split position, two concatenated frames, leading noise, embedded false magic, corrupted CRC, length `55`, unknown version/type, invalid enum, NaN quaternion, and a bad frame followed by a good frame. Assert the good frame is emitted exactly once and the parser buffer remains bounded below 128 bytes.

- [ ] **Step 5: Run codec tests and commit**

```bash
cmake --build build-serial --target serial_protocol_test -j"$(nproc)"
ctest --test-dir build-serial -R serial_protocol_test --output-on-failure
git add io/serial_board/protocol.* tests/serial_protocol_test.cpp io/CMakeLists.txt CMakeLists.txt
git commit -m "feat: add versioned serial protocol codec"
```

Expected: all protocol tests pass and the golden vectors are ready for controller-firmware tests.

---

### Task 3: Implement the Unified SerialBoard Transport

**Files:**
- Create: `io/serial_board/serial_board.hpp`
- Create: `io/serial_board/serial_board.cpp`
- Create: `tests/serial_board_test.cpp`
- Modify: `io/CMakeLists.txt`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 2 codec and YAML keys `serial_port`, `serial_baudrate`, `serial_timeout_ms`, `serial_startup_timeout_ms`
- Produces: one API used by every executable in Task 4

- [ ] **Step 1: Declare the unified API and write failing pseudo-terminal tests**

Use this interface:

```cpp
namespace io {
using Mode = serial_board::Mode;
using ShootMode = serial_board::ShootMode;
using BoardState = serial_board::Telemetry;
using BoardCommand = serial_board::Command;

class SerialBoard {
public:
  explicit SerialBoard(const std::string & config_path);
  ~SerialBoard();
  BoardState state() const;
  Mode mode() const;
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  void send(const BoardCommand & command);
  void send(const io::Command & command);
  bool connected() const;
  static std::string str(Mode mode);
};
}
```

Create a pseudo-terminal with `openpty()`. Test that fragmented telemetry written to the master becomes a state snapshot and interpolation sample, and that `send(BoardCommand)` produces the exact command frame on the master. Assert `send(io::Command)` maps yaw, pitch, control, fire, and `horizon_distance` while zeroing velocity/acceleration.

- [ ] **Step 2: Run tests and verify they fail**

```bash
cmake --build build-serial --target serial_board_test -j"$(nproc)"
ctest --test-dir build-serial -R serial_board_test --output-on-failure
```

Expected: build fails because `SerialBoard` is not implemented.

- [ ] **Step 3: Implement serial configuration and receive lifecycle**

Read the YAML values, configure the existing serial library for `921600/8N1/no-flow-control`, and use a finite read timeout. The receive thread must feed arbitrary chunks to `StreamParser`, timestamp each accepted telemetry packet with `steady_clock::now()`, update state under a mutex, and push quaternion samples into a bounded queue.

Do not call `exit(1)`. Throw a descriptive exception if the port cannot produce valid telemetry within `serial_startup_timeout_ms`; after startup, reconnect with bounded one-second backoff while allowing the destructor to stop promptly.

- [ ] **Step 4: Implement safe state, interpolation, and transmission**

Return state snapshots by value. Validate quaternion norm before queueing. Make `imu_at()` preserve the current slerp behavior but reject queries when fewer than two samples exist or telemetry is stale. Serialize writes with a transmit mutex, increment sequence modulo 65536, force `fire=false` when `control=false`, and send a zero command during shutdown/reconnect when the port is writable.

- [ ] **Step 5: Add disconnect/reconnect and concurrency tests**

Test startup timeout, stale telemetry, CRC failure not refreshing connectivity, sequence wrap from `65535` to `0`, simultaneous state reads and sends, parser reset after reconnect, and destructor completion within 200 ms when no bytes arrive.

- [ ] **Step 6: Run transport tests and commit**

```bash
cmake --build build-serial --target serial_board_test -j"$(nproc)"
ctest --test-dir build-serial -R 'serial_(protocol|board)_test' --output-on-failure
git add io/serial_board/serial_board.* tests/serial_board_test.cpp io/CMakeLists.txt CMakeLists.txt
git commit -m "feat: add unified serial board transport"
```

Expected: codec and pseudo-terminal integration tests pass without physical hardware.

---

### Task 4: Migrate Every Application and Communication Test

**Files:**
- Modify: `src/standard.cpp`, `src/mt_standard.cpp`, `src/mt_auto_aim_debug.cpp`
- Modify: `src/auto_buff_debug.cpp`, `src/uav.cpp`, `src/uav_debug.cpp`
- Modify: `src/sentry.cpp`, `src/sentry_bp.cpp`, `src/sentry_debug.cpp`, `src/sentry_multithread.cpp`
- Modify: `src/standard_mpc.cpp`, `src/auto_aim_debug_mpc.cpp`, `src/auto_buff_debug_mpc.cpp`
- Modify: `calibration/capture.cpp`
- Modify: `tasks/auto_aim/multithread/commandgener.hpp`, `tasks/auto_aim/multithread/commandgener.cpp`
- Modify: `tests/cboard_test.cpp`, `tests/gimbal_test.cpp`, `tests/gimbal_response_test.cpp`, `tests/fire_test.cpp`, `tests/planner_test.cpp`, `tests/handeye_test.cpp`

**Interfaces:**
- Consumes: `io::SerialBoard`, `io::BoardState`, and `io::BoardCommand` from Task 3
- Produces: zero production includes or symbols referring to `CBoard` or `Gimbal`

- [ ] **Step 1: Add a compile-time migration guard**

Add a CTest script that runs:

```bash
rg -n 'io/cboard.hpp|io::CBoard|io/gimbal/gimbal.hpp|io::Gimbal' src tasks calibration tests
```

The test passes only when output is empty. Run it now and confirm it fails with the known call sites.

- [ ] **Step 2: Migrate the shared command generator**

Change constructor/member types from `io::CBoard &` to `io::SerialBoard &`, include `io/serial_board/serial_board.hpp`, and retain `send(io::Command)` so its current behavior and UAV horizontal distance are preserved.

- [ ] **Step 3: Migrate CBoard-style executables**

Replace each `io::CBoard cboard(config_path)` with `io::SerialBoard board(config_path)`. Replace public-field reads with one coherent snapshot per loop:

```cpp
const auto board_state = board.state();
const auto q = board.imu_at(timestamp - 1ms);
const auto mode = board_state.mode;
const auto bullet_speed = board_state.bullet_speed;
const auto shoot_mode = board_state.shoot_mode;
board.send(command);
```

Use `io::SerialBoard::str(mode)` for logging. Preserve `OUTPOST`, sentry left/right/both shooting selection, UAV `ft_angle`, and all debug plot fields.

- [ ] **Step 4: Migrate Gimbal/MPC-style executables**

Replace the eight-argument `gimbal.send(...)` calls with a fully initialized `io::BoardCommand`:

```cpp
board.send(io::BoardCommand{
  plan.control, plan.fire,
  plan.yaw, plan.yaw_vel, plan.yaw_acc,
  plan.pitch, plan.pitch_vel, plan.pitch_acc,
  0.0F});
```

Replace `gimbal.q(t)`, `gimbal.state()`, and `gimbal.mode()` with `board.imu_at(t)`, `board.state()`, and the snapshot's mode. Send `io::BoardCommand{}` on idle and shutdown.

- [ ] **Step 5: Migrate communication and calibration tests**

Rename the CTest target `cboard_test` to `serial_board_hardware_test`; consolidate redundant `gimbal_test` coverage into it. Keep response, firing, planner, and hand-eye tests as explicit hardware tests, but make every one instantiate `SerialBoard` and use the V2 state/command types.

- [ ] **Step 6: Build all targets and run non-hardware tests**

```bash
cmake --build build-serial -j"$(nproc)"
ctest --test-dir build-serial --output-on-failure -E 'hardware|camera'
rg -n 'io/cboard.hpp|io::CBoard|io/gimbal/gimbal.hpp|io::Gimbal' src tasks calibration tests
```

Expected: build succeeds, non-hardware tests pass, and `rg` prints nothing.

- [ ] **Step 7: Commit the application migration**

```bash
git add src tasks/auto_aim/multithread calibration/capture.cpp tests CMakeLists.txt
git commit -m "refactor: migrate vision applications to serial board"
```

---

### Task 5: Consolidate Configuration and Deployment Documentation

**Files:**
- Modify: `configs/ascento.yaml`, `configs/calibration.yaml`, `configs/demo.yaml`, `configs/example.yaml`
- Modify: `configs/mvs.yaml`, `configs/sentry.yaml`, `configs/standard3.yaml`, `configs/standard4.yaml`, `configs/uav.yaml`
- Modify: `readme.md`
- Create: `tests/config_serial_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: SerialBoard YAML configuration from Task 3
- Produces: every runnable configuration selects one serial port and contains no CAN keys

- [ ] **Step 1: Write the failing configuration test**

For every `configs/*.yaml` used by a production or calibration executable, assert:

```text
serial_port == "/dev/gimbal"
serial_baudrate == 921600
serial_timeout_ms == 20
serial_startup_timeout_ms == 3000
can_interface, quaternion_canid, bullet_speed_canid, send_canid, and com_port are absent
```

Run `ctest --test-dir build-serial -R config_serial_test --output-on-failure`; expected failure is missing serial keys and present CAN keys.

- [ ] **Step 2: Replace configuration keys carefully**

In each affected YAML file, replace only its communication section with:

```yaml
#####-----serial board-----#####
serial_port: "/dev/gimbal"
serial_baudrate: 921600
serial_timeout_ms: 20
serial_startup_timeout_ms: 3000
```

Preserve all unrelated user edits already present in these files.

- [ ] **Step 3: Update deployment documentation**

Remove `can-utils`, USB2CAN setup, `can0/can1` udev rules, and CAN troubleshooting. Retain and tighten the `/dev/gimbal` udev instructions, add `dialout` group verification, `stty -F /dev/gimbal 921600`, protocol V2 firmware requirement, startup timeout behavior, and the hardware smoke-test command.

- [ ] **Step 4: Run configuration tests and commit**

```bash
cmake --build build-serial --target config_serial_test -j"$(nproc)"
ctest --test-dir build-serial -R config_serial_test --output-on-failure
git add configs readme.md tests/config_serial_test.cpp CMakeLists.txt
git commit -m "config: make serial the only controller transport"
```

Expected: every shipped configuration is serial-only and documentation has no CAN setup path.

---

### Task 6: Hardware Acceptance and CAN Removal

**Files:**
- Delete: `io/cboard.hpp`
- Delete: `io/cboard.cpp`
- Delete: `io/socketcan.hpp`
- Delete after migration or rename: `io/gimbal/gimbal.hpp`, `io/gimbal/gimbal.cpp`
- Modify: `io/CMakeLists.txt`
- Modify: `CMakeLists.txt`
- Modify: `readme.md`

**Interfaces:**
- Consumes: controller firmware V2, host SerialBoard, `/dev/gimbal`
- Produces: repository and deployment with no CAN control path

- [ ] **Step 1: Run controller-link hardware smoke tests with actuators disabled**

```bash
ls -l /dev/gimbal
./build-serial/serial_board_hardware_test configs/sentry.yaml
```

Expected for at least 60 seconds: connected state remains true, telemetry rate is `500 Hz +/- 5%`, CRC failure rate is zero, sequence loss is below 0.1%, quaternion norm stays within `1 +/- 0.01`, and mode/shoot-mode/bullet-speed fields match controller values.

- [ ] **Step 2: Run functional acceptance by platform**

With firing physically inhibited first, verify idle produces zero/no-control commands, auto-aim follows yaw/pitch, small/big buff modes switch correctly, sentry LEFT/RIGHT/BOTH selection survives, and UAV `horizon_distance`/`ft_angle` survive. Then enable firing in a controlled test and confirm the 100 ms watchdog stops control and firing when the serial cable is unplugged.

- [ ] **Step 3: Remove legacy transport sources and targets**

Delete the three CAN files, remove `cboard.cpp` and old `gimbal/gimbal.cpp` from `io/CMakeLists.txt`, remove obsolete communication test targets, and move any still-useful test behavior under `serial_board_hardware_test`. Remove empty directories only after confirming they contain no user files.

- [ ] **Step 4: Add and run a repository-wide no-CAN gate**

```bash
rg -n -i 'socketcan|can_interface|quaternion_canid|bullet_speed_canid|send_canid|can-utils|can0|can1' \
  --glob '!build*/**' --glob '!install/**' --glob '!log/**' --glob '!.git/**' .
```

Expected: no output. Generic English uses of the word "can" are intentionally excluded from the pattern.

- [ ] **Step 5: Run final verification**

```bash
cmake -S . -B build-serial -DCMAKE_BUILD_TYPE=Release
cmake --build build-serial -j"$(nproc)"
ctest --test-dir build-serial --output-on-failure -E 'hardware|camera'
./build-serial/serial_board_hardware_test configs/sentry.yaml
git diff --check
```

Expected: clean configure/build, all automated non-hardware tests pass, hardware telemetry remains valid, and no whitespace errors exist.

- [ ] **Step 6: Commit CAN removal**

```bash
git add -A io CMakeLists.txt readme.md tests
git commit -m "refactor: remove legacy CAN control link"
```

Expected: `git grep -i socketcan` and the no-CAN gate both return no matches.

---

## Rollout and Rollback

1. Tag the last CAN-compatible host and controller firmware as a matched rollback pair.
2. Flash protocol V2 controller firmware before starting the serial-only host; mixed V1/V2 pairs must fail closed because the version byte differs.
3. Run smoke tests with actuators disabled, then control enabled with firing inhibited, then controlled firing.
4. Deploy one robot first and monitor CRC errors, sequence gaps, reconnect count, telemetry age, and controller watchdog events for a full operating session.
5. Roll back both host and controller firmware together if telemetry age exceeds 100 ms repeatedly, packet loss exceeds 0.1%, or any mode/command field differs from the acceptance checklist.

## Completion Criteria

- All production, debug, calibration, and communication-test executables use `io::SerialBoard`.
- Protocol golden vectors pass on host and controller firmware.
- All configuration files use `/dev/gimbal` at 921600 baud and contain no CAN IDs/interfaces.
- Full Release build and all non-hardware CTests pass.
- Hardware acceptance demonstrates field parity and the 100 ms fail-safe.
- `io/cboard.*`, `io/socketcan.hpp`, old `io::Gimbal`, CAN dependencies, and CAN deployment instructions are removed.
