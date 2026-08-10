#include "io/gimbal/gimbal_runtime_gate.hpp"

#include <utility>

namespace io
{
GimbalRuntimeGate::GimbalRuntimeGate(
  SafeFireLatch safe_fire_latch, std::function<Clock::time_point()> now_provider)
: timeline_(std::move(now_provider)), safe_fire_latch_(std::move(safe_fire_latch))
{
}

SampleResult GimbalRuntimeGate::OnSample(
  uint64_t remote_us, Eigen::Quaterniond quaternion, Clock::time_point local_time)
{
  std::lock_guard lock(mutex_);
  const auto result = timeline_.OnSample(remote_us, quaternion, local_time);
  if (result == SampleResult::REJECTED) {
    return result;
  }

  if (!ready_) {
    ready_ = true;
    submissions_enabled_ = true;
    ready_changed_.notify_all();
    return result;
  }

  if (result == SampleResult::RECOVERED) {
    submissions_enabled_ = false;
    if (!safe_fire_latched_for_outage_) {
      safe_fire_latch_();
    }
    safe_fire_latched_for_outage_ = false;
    submissions_enabled_ = true;
  }
  return result;
}

bool GimbalRuntimeGate::MarkStale(Clock::time_point now)
{
  std::lock_guard lock(mutex_);
  return MarkStaleLocked(now);
}

bool GimbalRuntimeGate::MarkStaleLocked(Clock::time_point now)
{
  if (!timeline_.MarkStale(now)) return false;
  submissions_enabled_ = false;
  safe_fire_latch_();
  safe_fire_latched_for_outage_ = true;
  return true;
}

bool GimbalRuntimeGate::SubmitWhenFresh(Clock::time_point now, const Submission & submission)
{
  std::lock_guard lock(mutex_);
  if (!submissions_enabled_ || !timeline_.Fresh(now)) {
    if (ready_ && submissions_enabled_) MarkStaleLocked(now);
    return false;
  }
  submission();
  return true;
}

void GimbalRuntimeGate::SubmitPassiveFalseFire()
{
  std::lock_guard lock(mutex_);
  safe_fire_latch_();
}

void GimbalRuntimeGate::WaitReady()
{
  std::unique_lock lock(mutex_);
  ready_changed_.wait(lock, [this] { return ready_; });
}

Eigen::Quaterniond GimbalRuntimeGate::WaitQuaternion(Clock::time_point requested)
{
  return timeline_.WaitAt(requested);
}

bool GimbalRuntimeGate::HasFreshAhrs(Clock::time_point now) const
{
  return timeline_.Fresh(now);
}
}  // namespace io
