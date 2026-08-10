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
