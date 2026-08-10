#pragma once

#include <Eigen/Geometry>

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>

#include "io/gimbal/ahrs_timeline.hpp"

namespace io
{
class GimbalRuntimeGate
{
public:
  using Clock = AhrsTimeline::Clock;
  using SafeFireLatch = std::function<void()>;
  using Submission = std::function<void()>;

  explicit GimbalRuntimeGate(
    SafeFireLatch safe_fire_latch,
    std::function<Clock::time_point()> now_provider = Clock::now);

  SampleResult OnSample(
    uint64_t remote_us, Eigen::Quaterniond quaternion, Clock::time_point local_time);
  bool MarkStale(Clock::time_point now);
  bool SubmitWhenFresh(Clock::time_point now, const Submission & submission);
  void SubmitPassiveFalseFire();
  void WaitReady();
  Eigen::Quaterniond WaitQuaternion(Clock::time_point requested);
  [[nodiscard]] bool HasFreshAhrs(Clock::time_point now) const;

private:
  bool MarkStaleLocked(Clock::time_point now);

  mutable std::mutex mutex_;
  std::condition_variable ready_changed_;
  AhrsTimeline timeline_;
  SafeFireLatch safe_fire_latch_;
  bool ready_ = false;
  bool submissions_enabled_ = false;
  bool safe_fire_latched_for_outage_ = false;
};
}  // namespace io
