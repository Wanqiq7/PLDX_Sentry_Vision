#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "io/gimbal/gimbal.hpp"
#include "io/gimbal/libxr_protocol.hpp"
#include "libxr.hpp"

namespace
{
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using io::libxr_protocol::FirePayload;
using io::libxr_protocol::QuaternionPayload;
using io::libxr_protocol::TargetEulerPayload;

[[noreturn]] void Fail(const std::string & message) { throw std::runtime_error(message); }

void Check(bool condition, const std::string & message)
{
  if (!condition) Fail(message);
}

int RemainingMs(Clock::time_point deadline)
{
  const auto remaining =
    std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
  return static_cast<int>(std::clamp<int64_t>(remaining, 0, 30000));
}

void WaitFd(int fd, short events, Clock::time_point deadline)
{
  for (;;) {
    pollfd descriptor{fd, events, 0};
    const int result = poll(&descriptor, 1, RemainingMs(deadline));
    if (result > 0 && (descriptor.revents & events)) return;
    if (result < 0 && errno == EINTR) continue;
    if (result == 0) Fail("I/O deadline expired");
    Fail("poll failed or descriptor closed");
  }
}

void WriteAll(int fd, const void * data, size_t size, Clock::time_point deadline)
{
  auto * cursor = static_cast<const uint8_t *>(data);
  while (size != 0) {
    WaitFd(fd, POLLOUT, deadline);
    const auto written = write(fd, cursor, size);
    if (written < 0 && errno == EINTR) continue;
    if (written <= 0) Fail("write failed");
    cursor += written;
    size -= static_cast<size_t>(written);
  }
}

char ReadSignal(int fd, Clock::time_point deadline)
{
  char value = 0;
  WaitFd(fd, POLLIN, deadline);
  const auto count = read(fd, &value, 1);
  Check(count == 1, "child control channel closed");
  Check(value != 'X', "child reported failure");
  return value;
}

void SendSignal(int fd, char value)
{
  WriteAll(fd, &value, 1, Clock::now() + 1s);
}

void SetCloseOnExec(int fd, bool close_on_exec)
{
  const int flags = fcntl(fd, F_GETFD);
  Check(flags >= 0, "fcntl(F_GETFD) failed");
  const int updated = close_on_exec ? flags | FD_CLOEXEC : flags & ~FD_CLOEXEC;
  Check(fcntl(fd, F_SETFD, updated) == 0, "fcntl(F_SETFD) failed");
  const int verified = fcntl(fd, F_GETFD);
  Check(verified >= 0 && static_cast<bool>(verified & FD_CLOEXEC) == close_on_exec,
    "FD_CLOEXEC verification failed");
}

struct Pty
{
  int master = -1;
  int slave_control = -1;
  std::string slave;

  Pty()
  {
    master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    Check(master >= 0, "posix_openpt failed");
    SetCloseOnExec(master, true);
    Check(grantpt(master) == 0, "grantpt failed");
    Check(unlockpt(master) == 0, "unlockpt failed");
    char * name = ptsname(master);
    Check(name != nullptr, "ptsname failed");
    slave = name;

    slave_control = open(slave.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    Check(slave_control >= 0, "open PTY slave failed");
    SetCloseOnExec(slave_control, true);
    termios tty{};
    Check(tcgetattr(slave_control, &tty) == 0, "tcgetattr failed");
    cfmakeraw(&tty);
#ifdef B921600
    Check(cfsetispeed(&tty, B921600) == 0, "cfsetispeed failed");
    Check(cfsetospeed(&tty, B921600) == 0, "cfsetospeed failed");
#endif
    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    Check(tcsetattr(slave_control, TCSANOW, &tty) == 0, "tcsetattr failed");
  }

  ~Pty()
  {
    if (master >= 0) close(master);
    if (slave_control >= 0) close(slave_control);
  }
};

bool KillAndReapWithin(pid_t pid, std::chrono::milliseconds timeout) noexcept
{
  if (pid <= 0) return true;
  static_cast<void>(kill(pid, SIGKILL));
  const auto deadline = Clock::now() + timeout;
  while (Clock::now() < deadline) {
    const auto result = waitpid(pid, nullptr, WNOHANG);
    if (result == pid || (result < 0 && errno == ECHILD)) return true;
    if (result < 0 && errno != EINTR) return false;
    static_cast<void>(poll(nullptr, 0, 5));
  }
  return false;
}

class Child
{
public:
  Child(const char * executable, std::string_view scenario, const std::string & slave)
  {
    int sockets[2];
    Check(socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets) == 0, "socketpair failed");
    pid_ = fork();
    Check(pid_ >= 0, "fork failed");
    if (pid_ == 0) {
      close(sockets[0]);
      SetCloseOnExec(sockets[1], false);
      const std::string fd = std::to_string(sockets[1]);
      execl(executable, executable, "--child", std::string(scenario).c_str(), slave.c_str(),
        fd.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }
    close(sockets[1]);
    control_ = sockets[0];
  }

  Child(const Child &) = delete;
  Child & operator=(const Child &) = delete;

  ~Child() noexcept
  {
    if (control_ >= 0) close(control_);
    if (pid_ > 0 && !reaped_) {
      reaped_ = KillAndReapWithin(pid_, 500ms);
      if (!reaped_) {
        constexpr char warning[] = "libxr_pty_test: child could not be reaped within cleanup bound\n";
        const auto ignored = write(STDERR_FILENO, warning, sizeof(warning) - 1);
        static_cast<void>(ignored);
      }
    }
  }

  int control() const { return control_; }

  void Wait(Clock::time_point deadline)
  {
    for (;;) {
      int status = 0;
      const auto result = waitpid(pid_, &status, WNOHANG);
      if (result == pid_) {
        reaped_ = true;
        Check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "child exited unsuccessfully");
        return;
      }
      Check(result >= 0 || errno == EINTR, "waitpid failed");
      if (Clock::now() >= deadline) Fail("child exit deadline expired");
      poll(nullptr, 0, 5);
    }
  }

private:
  pid_t pid_ = -1;
  int control_ = -1;
  bool reaped_ = false;
};

struct Capture
{
  std::vector<TargetEulerPayload> targets;
  std::vector<FirePayload> fires;
};

class Controller
{
public:
  explicit Controller(int master)
  : master_(master), target_(LibXR::Topic::CreateTopic<TargetEulerPayload>(
      io::libxr_protocol::TARGET_EULER_TOPIC)),
    fire_(LibXR::Topic::CreateTopic<FirePayload>(io::libxr_protocol::FIRE_NOTIFY_TOPIC)),
    ahrs_(LibXR::Topic::CreateTopic<QuaternionPayload>(io::libxr_protocol::AHRS_QUATERNION_TOPIC)),
    server_(512),
    target_callback_(LibXR::Topic::Callback::Create(
      [](bool, Capture * capture, TargetEulerPayload & value) {
        capture->targets.push_back(value);
      },
      &capture_)),
    fire_callback_(LibXR::Topic::Callback::Create(
      [](bool, Capture * capture, FirePayload & value) { capture->fires.push_back(value); },
      &capture_))
  {
    target_.RegisterCallback(target_callback_);
    fire_.RegisterCallback(fire_callback_);
    server_.Register(target_);
    server_.Register(fire_);
  }

  void SendAhrs(const QuaternionPayload & value, uint64_t timestamp, bool fragmented = false)
  {
    LibXR::Topic::PackedData<QuaternionPayload> packet;
    Check(
      ahrs_.PackData(value, packet, LibXR::MicrosecondTimestamp(timestamp)) ==
        LibXR::ErrorCode::OK,
      "failed to pack AHRS");
    const auto * bytes = reinterpret_cast<const uint8_t *>(&packet);
    if (fragmented) {
      WriteAll(master_, bytes, 3, Clock::now() + 1s);
      WriteAll(master_, bytes + 3, 7, Clock::now() + 1s);
      WriteAll(master_, bytes + 10, sizeof(packet) - 10, Clock::now() + 1s);
    } else {
      WriteAll(master_, bytes, sizeof(packet), Clock::now() + 1s);
    }
  }

  bool PumpUntil(Clock::time_point deadline, const std::function<bool()> & predicate)
  {
    std::array<uint8_t, 256> bytes{};
    while (!predicate()) {
      pollfd descriptor{master_, POLLIN, 0};
      const int result = poll(&descriptor, 1, RemainingMs(deadline));
      if (result == 0) return predicate();
      if (result < 0 && errno == EINTR) continue;
      Check(result > 0 && (descriptor.revents & POLLIN), "controller poll failed");
      for (;;) {
        const auto count = read(master_, bytes.data(), bytes.size());
        if (count > 0) {
          server_.ParseData(LibXR::ConstRawData(bytes.data(), static_cast<size_t>(count)));
          continue;
        }
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        if (count < 0 && errno == EINTR) continue;
        Check(count >= 0, "controller read failed");
        break;
      }
    }
    return true;
  }

  void DrainFor(std::chrono::milliseconds duration)
  {
    PumpUntil(Clock::now() + duration, [] { return false; });
  }

  Capture & capture() { return capture_; }

private:
  int master_ = -1;
  Capture capture_;
  LibXR::Topic target_;
  LibXR::Topic fire_;
  LibXR::Topic ahrs_;
  LibXR::Topic::Server server_;
  LibXR::Topic::Callback target_callback_;
  LibXR::Topic::Callback fire_callback_;
};

std::string SelfPath()
{
  std::array<char, 4096> path{};
  const auto length = readlink("/proc/self/exe", path.data(), path.size() - 1);
  Check(length > 0, "readlink /proc/self/exe failed");
  return std::string(path.data(), static_cast<size_t>(length));
}

void StartChild(Child & child, Controller & controller)
{
  Check(ReadSignal(child.control(), Clock::now() + 2s) == 'S', "child did not start");
  // Bound the local anchor relative to the child's pre-construction timestamp.
  poll(nullptr, 0, 50);
  controller.SendAhrs({0.0F, 0.0F, 0.0F, 2.0F}, 1'000'000, true);
  Check(ReadSignal(child.control(), Clock::now() + 2s) == 'R', "child did not become ready");
}

void TestFragmentedAhrs(const std::string & executable)
{
  Pty pty;
  Controller controller(pty.master);
  Child child(executable.c_str(), "quaternion", pty.slave);
  StartChild(child, controller);
  controller.SendAhrs({0.0F, 0.0F, 3.0F, 0.0F}, 1'400'000);
  SendSignal(child.control(), 'Q');
  Check(ReadSignal(child.control(), Clock::now() + 2s) == 'P', "SLERP assertion failed");
  SendSignal(child.control(), 'D');
  child.Wait(Clock::now() + 2s);
}

void TestTargetAndFailClosedFire(const std::string & executable)
{
  Pty pty;
  Controller controller(pty.master);
  Child child(executable.c_str(), "output", pty.slave);
  StartChild(child, controller);
  SendSignal(child.control(), 'T');
  Check(ReadSignal(child.control(), Clock::now() + 1s) == 'A', "target send failed");
  Check(controller.PumpUntil(Clock::now() + 1s, [&] {
    return !controller.capture().targets.empty() && !controller.capture().fires.empty();
  }), "target/fire packets not received");
  const auto & target = controller.capture().targets.back();
  const TargetEulerPayload expected{0.0F, 4.0F, 1.0F, 0.0F, 5.0F, 2.0F, 0.0F, 6.0F, 3.0F};
  Check(std::memcmp(&target, &expected, sizeof(expected)) == 0, "target fields differ");
  Check(std::none_of(controller.capture().fires.begin(), controller.capture().fires.end(),
          [](const FirePayload & value) { return value.isfire; }),
    "runtime emitted true fire");

  const auto target_count = controller.capture().targets.size();
  const auto fire_count = controller.capture().fires.size();
  SendSignal(child.control(), 'F');
  Check(ReadSignal(child.control(), Clock::now() + 1s) == 'A', "fail-closed send failed");
  Check(controller.PumpUntil(Clock::now() + 1s,
          [&] { return controller.capture().fires.size() > fire_count; }),
    "false fire packet not received");
  controller.DrainFor(100ms);
  Check(controller.capture().targets.size() == target_count, "uncontrolled send emitted target");
  Check(std::none_of(controller.capture().fires.begin() + static_cast<ptrdiff_t>(fire_count),
          controller.capture().fires.end(), [](const FirePayload & value) { return value.isfire; }),
    "control=false/fire=true escaped as true");
  SendSignal(child.control(), 'D');
  child.Wait(Clock::now() + 2s);
}

void TestStaleRecovery(const std::string & executable)
{
  Pty pty;
  Controller controller(pty.master);
  Child child(executable.c_str(), "stale", pty.slave);
  StartChild(child, controller);
  poll(nullptr, 0, 180);
  controller.DrainFor(30ms);
  const auto targets_before = controller.capture().targets.size();
  SendSignal(child.control(), 'T');
  Check(ReadSignal(child.control(), Clock::now() + 1s) == 'A', "stale child send failed");
  controller.DrainFor(100ms);
  Check(controller.capture().targets.size() == targets_before, "stale AHRS allowed a target");

  controller.SendAhrs({3.0F, 0.0F, 0.0F, 0.0F}, 1'500'000);
  controller.SendAhrs({0.0F, 3.0F, 0.0F, 0.0F}, 1'900'000);
  SendSignal(child.control(), 'R');
  Check(ReadSignal(child.control(), Clock::now() + 2s) == 'P', "post-gap interpolation failed");
  Check(controller.PumpUntil(Clock::now() + 1s,
          [&] { return controller.capture().targets.size() > targets_before; }),
    "post-gap target did not recover");
  SendSignal(child.control(), 'D');
  child.Wait(Clock::now() + 2s);
}

void TestLatestPendingUnderPtyPressure(const std::string & executable)
{
  Pty pty;
  Controller controller(pty.master);
  Child child(executable.c_str(), "pressure", pty.slave);
  StartChild(child, controller);
  controller.DrainFor(30ms);
  SendSignal(child.control(), 'B');

  uint64_t timestamp = 1'050'000;
  auto next_ahrs = Clock::now() + 40ms;
  const auto deadline = Clock::now() + 7s;
  Clock::time_point full_since{};
  int max_tiocoutq = 0;
  size_t filler_bytes = 0;
  size_t progress_batches = 0;
  size_t progress_at_full = 0;
  const std::array<uint8_t, 512> filler{};
  while (full_since == Clock::time_point{} || Clock::now() - full_since < 100ms ||
         progress_batches - progress_at_full < 2) {
    Check(Clock::now() < deadline,
      "PTY output queue never reached sustained EAGAIN (filler=" +
        std::to_string(filler_bytes) + ", max TIOCOUTQ=" + std::to_string(max_tiocoutq) + ")");
    if (Clock::now() >= next_ahrs) {
      controller.SendAhrs({0.0F, 0.0F, 0.0F, 1.0F}, timestamp);
      timestamp += 50'000;
      next_ahrs += 50ms;
    }
    int queued = 0;
    Check(ioctl(pty.slave_control, TIOCOUTQ, &queued) == 0, "TIOCOUTQ failed");
    max_tiocoutq = std::max(max_tiocoutq, queued);
    const auto written = write(pty.slave_control, filler.data(), filler.size());
    if (written > 0) {
      filler_bytes += static_cast<size_t>(written);
      full_since = {};
      progress_at_full = progress_batches;
    } else if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      Check(filler_bytes >= 4096, "PTY reported EAGAIN before a nontrivial fill");
      if (full_since == Clock::time_point{}) {
        full_since = Clock::now();
        progress_at_full = progress_batches;
      }
      poll(nullptr, 0, 10);
    } else if (written < 0 && errno == EINTR) {
      continue;
    } else {
      Fail("PTY filler write failed");
    }
    for (;;) {
      pollfd progress{child.control(), POLLIN, 0};
      if (poll(&progress, 1, 0) <= 0 || !(progress.revents & POLLIN)) break;
      Check(ReadSignal(child.control(), Clock::now() + 100ms) == 'G',
        "unexpected pressure progress signal");
      ++progress_batches;
    }
  }
  SendSignal(child.control(), 'M');
  Check(ReadSignal(child.control(), deadline) == 'A', "pressure child failed");

  Check(controller.PumpUntil(Clock::now() + 3s, [&] {
    return std::any_of(controller.capture().targets.begin(), controller.capture().targets.end(),
      [](const TargetEulerPayload & value) { return value.yaw == 100003.0F; });
  }), "latest pressure target was not received");
  controller.DrainFor(150ms);
  std::vector<float> burst;
  for (const auto & target : controller.capture().targets) {
    if (target.yaw >= 100001.0F) burst.push_back(target.yaw);
  }
  Check(!burst.empty() && burst.back() == 100003.0F, "latest target was not last");
  Check(burst.size() <= 2, "more than in-flight plus latest pending target survived");
  Check(std::none_of(controller.capture().fires.begin(), controller.capture().fires.end(),
          [](const FirePayload & value) { return value.isfire; }),
    "pressure case emitted true fire");
  SendSignal(child.control(), 'D');
  child.Wait(Clock::now() + 2s);
}

void RunCase(std::string_view scenario, const std::string & executable)
{
  if (scenario == "quaternion") {
    TestFragmentedAhrs(executable);
  } else if (scenario == "output") {
    TestTargetAndFailClosedFire(executable);
  } else if (scenario == "stale") {
    TestStaleRecovery(executable);
  } else if (scenario == "pressure") {
    TestLatestPendingUnderPtyPressure(executable);
  } else {
    Fail("unknown integration case");
  }
}

void RunCaseProcess(const std::string & executable, std::string_view scenario)
{
  const pid_t pid = fork();
  Check(pid >= 0, "case fork failed");
  if (pid == 0) {
    execl(executable.c_str(), executable.c_str(), "--case", std::string(scenario).c_str(),
      static_cast<char *>(nullptr));
    _exit(127);
  }

  const auto deadline = Clock::now() + 12s;
  for (;;) {
    int status = 0;
    const auto result = waitpid(pid, &status, WNOHANG);
    if (result == pid) {
      Check(WIFEXITED(status) && WEXITSTATUS(status) == 0,
        "integration case failed: " + std::string(scenario));
      return;
    }
    if (result < 0 && errno != EINTR) {
      static_cast<void>(KillAndReapWithin(pid, 500ms));
      Fail("case waitpid failed: " + std::string(scenario));
    }
    if (Clock::now() >= deadline) {
      const bool reaped = KillAndReapWithin(pid, 500ms);
      Fail("integration case timed out: " + std::string(scenario) +
        (reaped ? "" : " (SIGKILL reap also timed out)"));
    }
    poll(nullptr, 0, 5);
  }
}

std::string WriteConfig(const std::string & slave)
{
  const std::string path = "/tmp/libxr_pty_" + std::to_string(getpid()) + ".yaml";
  std::ofstream output(path);
  Check(output.good(), "cannot create child config");
  output << "com_port: \"" << slave << "\"\n"
         << "baudrate: 921600\n"
         << "default_mode: AUTO_AIM\n"
         << "default_bullet_speed: 23.0\n";
  output.close();
  return path;
}

bool Near(double actual, double expected, double tolerance)
{
  return std::abs(actual - expected) <= tolerance;
}

int ChildMain(std::string_view scenario, const std::string & slave, int control)
{
  try {
    const auto config = WriteConfig(slave);
    SendSignal(control, 'S');
    const auto before_ready = Clock::now();
    io::Gimbal gimbal(config);
    const auto after_ready = Clock::now();
    unlink(config.c_str());
    SendSignal(control, 'R');

    if (scenario == "quaternion") {
      Check(ReadSignal(control, Clock::now() + 2s) == 'Q', "bad quaternion command");
      const auto requested = before_ready + 250ms;
      const auto q = gimbal.q(requested).normalized();
      const double ready_uncertainty =
        std::chrono::duration<double>(after_ready - before_ready).count();
      Check(ready_uncertainty < 1.0, "unexpected readiness delay");
      Check(Near(q.norm(), 1.0, 1e-9), "interpolated quaternion not normalized");
      Check(Near(std::abs(q.w()), std::sqrt(0.5), 0.08) &&
          Near(std::abs(q.z()), std::sqrt(0.5), 0.08),
        "unexpected SLERP orientation");
      SendSignal(control, 'P');
    } else if (scenario == "output") {
      Check(ReadSignal(control, Clock::now() + 2s) == 'T', "bad target command");
      gimbal.send(true, false, 1, 2, 3, 4, 5, 6);
      SendSignal(control, 'A');
      Check(ReadSignal(control, Clock::now() + 2s) == 'F', "bad fire command");
      gimbal.send(false, true, 11, 12, 13, 14, 15, 16);
      SendSignal(control, 'A');
    } else if (scenario == "stale") {
      Check(ReadSignal(control, Clock::now() + 2s) == 'T', "bad stale command");
      gimbal.send(true, false, 77, 0, 0, 0, 0, 0);
      SendSignal(control, 'A');
      Check(ReadSignal(control, Clock::now() + 2s) == 'R', "bad recovery command");
      const auto requested = Clock::now() + 200ms;
      const auto q = gimbal.q(requested).normalized();
      Check(Near(q.norm(), 1.0, 1e-9), "recovered quaternion not normalized");
      Check(std::abs(q.w()) < 0.1 && Near(std::abs(q.x()), std::sqrt(0.5), 0.1) &&
          Near(std::abs(q.y()), std::sqrt(0.5), 0.1),
        "old pre-gap interpolation samples were retained");
      gimbal.send(true, false, 88, 0, 0, 0, 0, 0);
      SendSignal(control, 'P');
    } else if (scenario == "pressure") {
      Check(ReadSignal(control, Clock::now() + 2s) == 'B', "bad pressure command");
      bool saturated = false;
      for (int i = 0; i < 4000; ++i) {
        gimbal.send(true, false, static_cast<float>(i), 0, 0, 0, 0, 0);
        if (i % 32 == 31) SendSignal(control, 'G');
        pollfd descriptor{control, POLLIN, 0};
        const int result = poll(&descriptor, 1, 2);
        if (result > 0 && (descriptor.revents & POLLIN)) {
          Check(ReadSignal(control, Clock::now() + 1s) == 'M', "bad saturation command");
          saturated = true;
          break;
        }
      }
      Check(saturated, "PTY did not saturate within child bound");
      gimbal.send(true, false, 100001, 0, 0, 0, 0, 0);
      gimbal.send(true, false, 100002, 0, 0, 0, 0, 0);
      gimbal.send(true, false, 100003, 0, 0, 0, 0, 0);
      SendSignal(control, 'A');
    } else {
      Fail("unknown child scenario");
    }
    Check(ReadSignal(control, Clock::now() + 3s) == 'D', "bad completion command");
    close(control);
    _exit(0);
  } catch (...) {
    const char failure = 'X';
    const auto ignored = write(control, &failure, 1);
    static_cast<void>(ignored);
    _exit(2);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 5 && std::string_view(argv[1]) == "--child") {
    return ChildMain(argv[2], argv[3], std::stoi(argv[4]));
  }
  try {
    const auto executable = SelfPath();
    if (argc == 3 && std::string_view(argv[1]) == "--case") {
      RunCase(argv[2], executable);
      return 0;
    }
    Check(argc == 1, "invalid arguments");
    for (const std::string_view scenario : {"quaternion", "output", "stale", "pressure"}) {
      RunCaseProcess(executable, scenario);
    }
    return 0;
  } catch (const std::exception & error) {
    const std::string message = std::string("libxr_pty_test: ") + error.what() + "\n";
    const auto ignored = write(STDERR_FILENO, message.data(), message.size());
    static_cast<void>(ignored);
    return 1;
  }
}
