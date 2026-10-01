#include "subscribe2nav.hpp"

#include <chrono>

namespace io
{
namespace
{
template <typename Input>
std::vector<int8_t> as_target_ids(const Input & input)
{
  return {input.begin(), input.end()};
}
}  // namespace

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber"),
  directive_store_(
    std::chrono::milliseconds(
      declare_parameter<int>("target_directive_timeout_ms", 750)),
    std::chrono::seconds(1))
{
#ifdef IO_HAS_PLDX_VISION_INTERFACES
  target_directive_subscription_ =
    create_subscription<pldx_vision_interfaces::msg::VisionTargetDirective>(
      "vision/target_directive", 10,
      std::bind(&Subscribe2Nav::target_directive_callback, this, std::placeholders::_1));
#endif

  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_mpc", rclcpp::QoS(1),
    std::bind(&Subscribe2Nav::cmd_vel_callback, this, std::placeholders::_1));
#ifdef IO_HAS_ROS_INTERFACES
  behavior_subscription_ = create_subscription<ros_interfaces::msg::Behavior>(
    "/sentry/behavior", rclcpp::QoS(1),
    std::bind(&Subscribe2Nav::behavior_callback, this, std::placeholders::_1));
  legacy_behavior_subscription_ = create_subscription<ros_interfaces::msg::Behavior>(
    "/sentry/behaivor_send", rclcpp::QoS(1),
    std::bind(&Subscribe2Nav::behavior_callback, this, std::placeholders::_1));
#endif

#ifdef IO_HAS_SP_MSGS
  enemy_status_subscription_ = create_subscription<sp_msgs::msg::EnemyStatusMsg>(
    "enemy_status", 10,
    std::bind(&Subscribe2Nav::enemy_status_callback, this, std::placeholders::_1));
  autoaim_target_subscription_ = create_subscription<sp_msgs::msg::AutoaimTargetMsg>(
    "autoaim_target", 10,
    std::bind(&Subscribe2Nav::autoaim_target_callback, this, std::placeholders::_1));
#endif

  RCLCPP_INFO(get_logger(), "nav_subscriber node initialized.");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(get_logger(), "nav_subscriber node starting to spin...");
  rclcpp::spin(shared_from_this());
}

TargetDirective Subscribe2Nav::subscribe_target_directive() const
{
  return directive_store_.snapshot();
}

void Subscribe2Nav::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (auto * runtime = GimbalRuntime::Current()) {
    navigation_protocol::ChassisTarget value{};
    value.vx_mps = static_cast<float>(msg->linear.x);
    value.vy_mps = static_cast<float>(msg->linear.y);
    value.vw_rad_s = static_cast<float>(msg->angular.z);
    value.use_speed_control = true;
    runtime->SendNavData(value);
  }
}

#ifdef IO_HAS_ROS_INTERFACES
void Subscribe2Nav::behavior_callback(const ros_interfaces::msg::Behavior::SharedPtr msg)
{
  if (auto * runtime = GimbalRuntime::Current()) {
    navigation_protocol::BehaviorData value{};
    value.pitch_mode = msg->pitch_mode;
    value.desire_stance = msg->desired_stance;
    value.desire_lifter_pos = msg->desire_lifter_pos;
    value.scan_yaw_min_rad = msg->scan_yaw_min * 0.017453292519943295F;
    value.scan_yaw_max_rad = msg->scan_yaw_max * 0.017453292519943295F;
    value.ammo_purchase_request = msg->ammo_purchase_request;
    value.revive_request = msg->revive_request;
    value.remote_revive_request = msg->remote_revive_request;
    value.remote_ammo_request = msg->remote_ammo_request;
    value.remote_health_request = msg->remote_health_request;
    value.use_limited_scan = msg->use_limited_scan;
    value.not_aim_enemy = msg->not_aim_enemy;
    value.use_capacitor = msg->use_capacitor;
    value.tunnel_align_active = msg->tunnel_align_active;
    value.tunnel_align_angle_rad = msg->tunnel_align_angle_deg * 0.017453292519943295F;
    value.use_gyro_mode = msg->use_gyro_mode;
    runtime->SendBehaviorData(value);
  }
}
#endif

#ifdef IO_HAS_PLDX_VISION_INTERFACES
void Subscribe2Nav::target_directive_callback(
  const pldx_vision_interfaces::msg::VisionTargetDirective::SharedPtr msg)
{
  directive_store_.accept_typed(TargetDirective{
    msg->valid, as_target_ids(msg->excluded_target_ids), msg->restrict_to_preferred_targets,
    as_target_ids(msg->preferred_target_ids)});
}
#endif

#ifdef IO_HAS_SP_MSGS
void Subscribe2Nav::enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg)
{
  directive_store_.accept_legacy_exclusions(msg->invincible_enemy_ids);
}

void Subscribe2Nav::autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg)
{
  directive_store_.accept_legacy_preferences(msg->target_ids);
}

std::vector<int8_t> Subscribe2Nav::subscribe_enemy_status()
{
  return subscribe_target_directive().excluded_target_ids;
}

std::vector<int8_t> Subscribe2Nav::subscribe_autoaim_target()
{
  return subscribe_target_directive().preferred_target_ids;
}
#endif
}  // namespace io
