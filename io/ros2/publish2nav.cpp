#include "publish2nav.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "capture_timestamp.hpp"
#include "tools/logger.hpp"
#include "io/gimbal/gimbal_runtime.hpp"

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
#ifdef IO_HAS_ROS_INTERFACES
  team_publisher_ = create_publisher<ros_interfaces::msg::TeamInformation>("/sentry/team_info", 1);
  game_publisher_ = create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 1);
  online_publisher_ = create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 1);
  offline_publisher_ = create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 1);
  radar_publisher_ = create_publisher<ros_interfaces::msg::RadarInfo>("/sentry/radar_info", 1);
  feedback_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { publish_navigation_feedback(); });
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

#ifdef IO_HAS_ROS_INTERFACES
void Publish2Nav::publish_navigation_feedback()
{
  auto * runtime = GimbalRuntime::Current();
  if (runtime == nullptr) return;
  const auto feedback = runtime->NavigationFeedback();
  if (!feedback.fresh) return;
  const auto stamp = now();
  ros_interfaces::msg::TeamInformation team; team.header.stamp = stamp;
  for (size_t i = 0; i < 4; ++i) {
    team.allies[i].robot_id = feedback.team.ally_status[i].robot_id;
    team.allies[i].robot_hp = feedback.team.ally_status[i].robot_hp;
    team.allies[i].position.position.x = feedback.team.ally_status[i].robot_pos_x;
    team.allies[i].position.position.y = feedback.team.ally_status[i].robot_pos_y;
  }
  team.outpost_hp = feedback.team.outpost_hp; team.base_hp = feedback.team.base_hp; team_publisher_->publish(team);
  ros_interfaces::msg::GameInfo game; game.header.stamp = stamp;
  game.game_time_remaining = feedback.game.game_time_remaining; game.coin_remaining = feedback.game.coin_remaining;
  game.event_code = feedback.game.event_code; game.game_status = feedback.game.game_status;
  game.manual_point_x = feedback.game.manual_point_x; game.manual_point_y = feedback.game.manual_point_y;
  game.manual_key = feedback.game.manual_key; game.enemy_outpost_hp = feedback.game.enemy_outpost_hp;
  game.enemy_base_hp = feedback.game.enemy_base_hp; game_publisher_->publish(game);
  ros_interfaces::msg::SentryInfoOnline online; online.header.stamp = stamp;
  online.self_health = feedback.online.self_health; online.bullets_remaining = feedback.online.bullets_remaining;
  online.cooling_value = feedback.online.cooling_value; online.heat_limit = feedback.online.heat_limit;
  online.current_heat = feedback.online.current_heat; online.sentry_pos.x = feedback.online.sentry_pos_x;
  online.sentry_pos.y = feedback.online.sentry_pos_y; online.speed_monitor_angle = feedback.online.speed_monitor_angle;
  online.sentry_info_1 = feedback.online.sentry_info_1; online.sentry_info_2 = feedback.online.sentry_info_2;
  online.sentry_info_3 = feedback.online.sentry_info_3; online.energy_ratio = feedback.online.energy_ratio; online_publisher_->publish(online);
  ros_interfaces::msg::SentryInfoOffline offline; offline.header.stamp = stamp;
  offline.yaw_camerainit_to_gimbal = feedback.offline.yaw_camerainit_to_gimbal; offline.lifter_current_pos = feedback.offline.lifter_current_pos;
  offline.is_transformable = feedback.offline.is_transformable; offline.transform_state = feedback.offline.transform_state;
  offline.capacitor_capacity = feedback.offline.capacitor_capacity; offline.chassis_imu_yaw = feedback.offline.chassis_imu_yaw;
  offline.tunnel_yaw_aligned = feedback.offline.tunnel_yaw_aligned; offline_publisher_->publish(offline);
  ros_interfaces::msg::RadarInfo radar; radar.header.stamp = stamp;
  for (size_t i = 0; i < 6; ++i) { radar.enemies[i].robot_id = feedback.radar.enemy_status[i].robot_id; radar.enemies[i].robot_hp = feedback.radar.enemy_status[i].robot_hp; radar.enemies[i].allowed_projectile = feedback.radar.enemy_status[i].allowed_projectile; radar.enemies[i].position.position.x = feedback.radar.enemy_status[i].robot_pos_x; radar.enemies[i].position.position.y = feedback.radar.enemy_status[i].robot_pos_y; }
  radar.enemy_coin_left = feedback.radar.enemy_coin_left; radar.enemy_coin_accumulated = feedback.radar.enemy_coin_accumulated; radar.is_enemy_outpost_sensed = feedback.radar.is_enemy_outpost_sensed; radar_publisher_->publish(radar);
}
#endif

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
