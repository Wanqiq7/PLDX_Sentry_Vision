#include "io/gimbal/ahrs_timeline.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace io
{
AhrsTimeline::AhrsTimeline(std::function<Clock::time_point()> now_provider)
: now_provider_(std::move(now_provider))
{
}

SampleResult AhrsTimeline::OnSample(
  uint64_t remote_us, Eigen::Quaterniond quaternion, Clock::time_point local_time)
{
  const auto max_abs = std::max(
    {std::abs(quaternion.w()), std::abs(quaternion.x()), std::abs(quaternion.y()),
      std::abs(quaternion.z())});
  if (!std::isfinite(quaternion.w()) || !std::isfinite(quaternion.x()) ||
      !std::isfinite(quaternion.y()) || !std::isfinite(quaternion.z()) ||
      !std::isfinite(max_abs) || max_abs <= 0.0) {
    return SampleResult::REJECTED;
  }
  Eigen::Quaterniond scaled(
    quaternion.w() / max_abs, quaternion.x() / max_abs, quaternion.y() / max_abs,
    quaternion.z() / max_abs);
  const auto scaled_norm = scaled.norm();
  if (!std::isfinite(scaled_norm) || scaled_norm <= 1e-6) {
    return SampleResult::REJECTED;
  }
  const long double mathematical_norm = static_cast<long double>(max_abs) *
    static_cast<long double>(scaled_norm);
  if (mathematical_norm <= 1e-6L) {
    return SampleResult::REJECTED;
  }
  scaled.normalize();
  quaternion = scaled;

  std::lock_guard lock(mutex_);
  const bool expired = has_anchor_ && local_time - last_receive_ >= kFreshness;
  const bool recovered = stale_ || expired || (has_anchor_ && remote_us < last_remote_);
  const bool reset_mapping = !has_anchor_ || recovered || remote_us < last_remote_;
  if (reset_mapping) {
    samples_.clear();
    remote_anchor_ = remote_us;
    local_anchor_ = local_time;
    has_anchor_ = true;
    stale_ = false;
  }

  auto mapped_time = local_anchor_;
  if (remote_us >= remote_anchor_) {
    mapped_time += std::chrono::microseconds(remote_us - remote_anchor_);
  }
  samples_.push_back({mapped_time, quaternion});
  if (samples_.size() > kMaxSamples) {
    samples_.pop_front();
  }
  last_remote_ = remote_us;
  last_receive_ = local_time;
  changed_.notify_all();
  return recovered ? SampleResult::RECOVERED : SampleResult::ACCEPTED;
}

bool AhrsTimeline::MarkStale(Clock::time_point now)
{
  std::lock_guard lock(mutex_);
  if (!has_anchor_ || stale_ || now - last_receive_ < kFreshness) {
    return false;
  }
  stale_ = true;
  changed_.notify_all();
  return true;
}

Eigen::Quaterniond AhrsTimeline::WaitAt(Clock::time_point requested)
{
  std::unique_lock lock(mutex_);
  for (;;) {
    const auto now = now_provider_();
    const bool expired = has_anchor_ && now - last_receive_ >= kFreshness;
    const bool bracketed = !samples_.empty() && samples_.front().local_time <= requested &&
      samples_.back().local_time >= requested;
    const bool unavailable = !samples_.empty() && requested < samples_.front().local_time;
    if (!stale_ && !expired && unavailable) {
      throw AhrsSampleUnavailable();
    }
    if (!stale_ && !expired && bracketed) {
      break;
    }
    if (!stale_ && !expired && has_anchor_) {
      const auto remaining = kFreshness - (now - last_receive_);
      changed_.wait_for(lock, remaining);
    } else {
      changed_.wait(lock);
    }
  }

  auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), requested,
    [](const Sample & sample, Clock::time_point time) { return sample.local_time < time; });
  if (upper == samples_.begin()) {
    return upper->quaternion;
  }
  if (upper == samples_.end()) {
    return samples_.back().quaternion;
  }
  if (upper->local_time == requested) {
    return upper->quaternion;
  }
  const auto & lower = *(upper - 1);
  const auto span = upper->local_time - lower.local_time;
  const double alpha = std::chrono::duration<double>(requested - lower.local_time).count() /
    std::chrono::duration<double>(span).count();
  Eigen::Quaterniond result = lower.quaternion.slerp(alpha, upper->quaternion);
  result.normalize();
  return result;
}

bool AhrsTimeline::Fresh(Clock::time_point now) const
{
  std::lock_guard lock(mutex_);
  return has_anchor_ && !stale_ && now - last_receive_ < kFreshness;
}

void AhrsTimeline::Reset()
{
  std::lock_guard lock(mutex_);
  samples_.clear();
  has_anchor_ = false;
  stale_ = false;
  changed_.notify_all();
}
}  // namespace io
