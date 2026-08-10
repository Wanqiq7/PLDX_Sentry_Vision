#ifndef IO__ROS2__CAPTURE_TIMESTAMP_HPP
#define IO__ROS2__CAPTURE_TIMESTAMP_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>

namespace io
{

inline int64_t capture_timestamp_in_ros_nanoseconds(
  std::chrono::steady_clock::time_point captured_at,
  std::chrono::steady_clock::time_point steady_now, int64_t ros_now_nanoseconds)
{
  const auto age = std::max(
    std::chrono::steady_clock::duration::zero(), steady_now - captured_at);
  const auto age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(age).count();
  return std::max<int64_t>(0, ros_now_nanoseconds - age_ns);
}

}  // namespace io

#endif  // IO__ROS2__CAPTURE_TIMESTAMP_HPP
