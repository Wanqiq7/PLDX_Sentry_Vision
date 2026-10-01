#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <memory>
#include <string>

#include "latest_value_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#ifdef IO_HAS_ROS_INTERFACES
#include "ros_interfaces/msg/team_information.hpp"
#include "ros_interfaces/msg/game_info.hpp"
#include "ros_interfaces/msg/sentry_info_online.hpp"
#include "ros_interfaces/msg/sentry_info_offline.hpp"
#include "ros_interfaces/msg/radar_info.hpp"
#endif
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
  void publish_navigation_feedback();

  // ROS2 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
#ifdef IO_HAS_PLDX_VISION_INTERFACES
  rclcpp::Publisher<pldx_vision_interfaces::msg::TargetObservation>::SharedPtr observation_publisher_;
#endif
  std::unique_ptr<LatestValuePublisher<Sample>> latest_publisher_;
#ifdef IO_HAS_ROS_INTERFACES
  rclcpp::Publisher<ros_interfaces::msg::TeamInformation>::SharedPtr team_publisher_;
  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_publisher_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr online_publisher_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_publisher_;
  rclcpp::Publisher<ros_interfaces::msg::RadarInfo>::SharedPtr radar_publisher_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
#endif
};

}  // namespace io

#endif  // Publish2Nav_HPP_
