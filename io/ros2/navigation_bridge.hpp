#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/behavior.hpp>
#include <ros_interfaces/msg/game_info.hpp>
#include <ros_interfaces/msg/radar_info.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>
#include <ros_interfaces/msg/sentry_info_online.hpp>
#include <ros_interfaces/msg/team_information.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#include "io/gimbal/gimbal_runtime.hpp"
#include "io/gimbal/navigation_protocol.hpp"
namespace io {
class NavigationBridge : public rclcpp::Node {
 public:
  explicit NavigationBridge(GimbalRuntime& runtime,
                            bool telemetry_only = false);
  ~NavigationBridge() override;

 private:
  void Tick();
  void Receive(const std::string&, std::span<const uint8_t>);
  GimbalRuntime& runtime_;
  const bool telemetry_only_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  geometry_msgs::msg::Twist velocity_{};
  geometry_msgs::msg::WrenchStamped force_{};
  nav_msgs::msg::Odometry odometry_{};
  ros_interfaces::msg::Behavior behavior_{};
  std::chrono::steady_clock::time_point velocity_at_{}, behavior_at_{},
      odometry_at_{}, force_at_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<ros_interfaces::msg::Behavior>::SharedPtr behavior_sub_;
  rclcpp::Publisher<ros_interfaces::msg::TeamInformation>::SharedPtr team_pub_;
  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr
      online_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr
      offline_pub_;
  rclcpp::Publisher<ros_interfaces::msg::RadarInfo>::SharedPtr radar_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr
      diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace io
