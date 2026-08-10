#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <memory>
#include <string>

#include "latest_value_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#ifdef IO_HAS_PLDX_VISION_INTERFACES
#include "pldx_vision_interfaces/msg/target_observation.hpp"
#endif

namespace io
{
class Publish2Nav : public rclcpp::Node
{
public:
  struct Sample
  {
    Eigen::Vector4d target;
    std::chrono::steady_clock::time_point captured_at;
  };

  Publish2Nav();

  ~Publish2Nav();

  void start();

  bool send_data(
    const Eigen::Vector4d & data, std::chrono::steady_clock::time_point captured_at) noexcept;

private:
  void publish_sample(const Sample & sample);

  // ROS2 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
#ifdef IO_HAS_PLDX_VISION_INTERFACES
  rclcpp::Publisher<pldx_vision_interfaces::msg::TargetObservation>::SharedPtr observation_publisher_;
#endif
  std::unique_ptr<LatestValuePublisher<Sample>> latest_publisher_;
};

}  // namespace io

#endif  // Publish2Nav_HPP_
