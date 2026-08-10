#include "publish2nav.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "capture_timestamp.hpp"
#include "tools/logger.hpp"

namespace io
{

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("auto_aim_target_pos", 10);
#ifdef IO_HAS_PLDX_VISION_INTERFACES
  observation_publisher_ =
    this->create_publisher<pldx_vision_interfaces::msg::TargetObservation>(
      "vision/target_observation", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
#endif
  latest_publisher_ = std::make_unique<LatestValuePublisher<Sample>>(
    [this](const Sample & sample) { publish_sample(sample); });

  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node initialized.");
}

Publish2Nav::~Publish2Nav()
{
  latest_publisher_.reset();
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node shutting down.");
}

bool Publish2Nav::send_data(
  const Eigen::Vector4d & target_pos,
  std::chrono::steady_clock::time_point captured_at) noexcept
{
  return latest_publisher_->submit(Sample{target_pos, captured_at});
}

void Publish2Nav::publish_sample(const Sample & sample)
{
  const auto & target_pos = sample.target;
  // 创建消息
  auto message = std::make_shared<std_msgs::msg::String>();

  // 将 Eigen::Vector3d 数据转换为字符串并存储在消息中
  // Keep the legacy protocol unchanged: x,y,valid,target_id.
  const bool has_target = std::isfinite(target_pos[0]) && std::isfinite(target_pos[1]) &&
                          std::isfinite(target_pos[2]) && target_pos[3] > 0.0;
  message->data = std::to_string(target_pos[0]) + "," + std::to_string(target_pos[1]) + "," +
                  (has_target ? "1," : "0,") + std::to_string(target_pos[3]);

  // 发布消息
  publisher_->publish(*message);
#ifdef IO_HAS_PLDX_VISION_INTERFACES
  pldx_vision_interfaces::msg::TargetObservation observation;
  const auto ros_now = this->get_clock()->now();
  const auto stamp_ns = capture_timestamp_in_ros_nanoseconds(
    sample.captured_at, std::chrono::steady_clock::now(), ros_now.nanoseconds());
  observation.header.stamp = rclcpp::Time(stamp_ns, ros_now.get_clock_type());
  observation.header.frame_id = "vision_gimbal";
  observation.position.x = target_pos[0];
  observation.position.y = target_pos[1];
  observation.position.z = target_pos[2];
  // The tracker velocity is relative to PLDX's rotation-only world frame and
  // contains sentry ego-motion. Publish unknown velocity as zero instead of
  // presenting it as an absolute map-frame target velocity.
  observation.velocity.x = 0.0;
  observation.velocity.y = 0.0;
  observation.velocity.z = 0.0;
  observation.target_id = target_pos[3] > 0.0 ? static_cast<uint8_t>(target_pos[3]) : 0;
  observation.confidence = observation.target_id == 0 ? 0.0F : 1.0F;
  observation.valid = has_target && observation.target_id != 0;
  observation_publisher_->publish(observation);
#endif

  // RCLCPP_INFO(
  //   this->get_logger(), "auto_aim_target_pos_publisher node sent message: '%s'",
  //   message->data.c_str());
}

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
