#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

#include "target_directive.hpp"

#ifdef IO_HAS_PLDX_VISION_INTERFACES
#include "pldx_vision_interfaces/msg/vision_target_directive.hpp"
#endif

#ifdef IO_HAS_SP_MSGS
#include "sp_msgs/msg/autoaim_target_msg.hpp"
#include "sp_msgs/msg/enemy_status_msg.hpp"
#endif

namespace io
{
class Subscribe2Nav : public rclcpp::Node
{
public:
  Subscribe2Nav();
  ~Subscribe2Nav();

  void start();
  TargetDirective subscribe_target_directive() const;

#ifdef IO_HAS_SP_MSGS
  std::vector<int8_t> subscribe_enemy_status();
  std::vector<int8_t> subscribe_autoaim_target();
#endif

private:
#ifdef IO_HAS_PLDX_VISION_INTERFACES
  void target_directive_callback(
    const pldx_vision_interfaces::msg::VisionTargetDirective::SharedPtr msg);
  rclcpp::Subscription<pldx_vision_interfaces::msg::VisionTargetDirective>::SharedPtr
    target_directive_subscription_;
#endif

#ifdef IO_HAS_SP_MSGS
  void enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg);
  void autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg);
  rclcpp::Subscription<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_status_subscription_;
  rclcpp::Subscription<sp_msgs::msg::AutoaimTargetMsg>::SharedPtr autoaim_target_subscription_;
#endif

  TargetDirectiveStore directive_store_;
};
}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP
