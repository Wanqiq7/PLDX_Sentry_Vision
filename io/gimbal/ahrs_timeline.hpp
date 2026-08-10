#pragma once

#include <Eigen/Geometry>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <stdexcept>

namespace io
{
enum class SampleResult { REJECTED, ACCEPTED, RECOVERED };

class AhrsSampleUnavailable : public std::runtime_error
{
public:
  explicit AhrsSampleUnavailable(const char * message = "requested AHRS sample is unavailable")
  : std::runtime_error(message)
  {
  }
};

class AhrsTimeline
{
public:
  using Clock = std::chrono::steady_clock;

  explicit AhrsTimeline(std::function<Clock::time_point()> now_provider = Clock::now);

  SampleResult OnSample(uint64_t remote_us, Eigen::Quaterniond quaternion, Clock::time_point local_time);
  bool MarkStale(Clock::time_point now);
  Eigen::Quaterniond WaitAt(Clock::time_point requested);
  bool Fresh(Clock::time_point now) const;
  void Reset();

private:
  struct Sample
  {
    Clock::time_point local_time;
    Eigen::Quaterniond quaternion;
  };

  static constexpr auto kFreshness = std::chrono::milliseconds(150);
  static constexpr std::size_t kMaxSamples = 1000;

  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::deque<Sample> samples_;
  bool has_anchor_ = false;
  bool stale_ = false;
  uint64_t remote_anchor_ = 0;
  uint64_t last_remote_ = 0;
  Clock::time_point local_anchor_{};
  Clock::time_point last_receive_{};
  std::function<Clock::time_point()> now_provider_;
};
}  // namespace io
