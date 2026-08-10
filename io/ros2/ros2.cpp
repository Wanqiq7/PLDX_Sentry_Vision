#include "ros2.hpp"
namespace io
{
ROS2::ROS2(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  publish2nav_ = std::make_shared<Publish2Nav>();

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
}

ROS2::~ROS2()
{
  if (rclcpp::ok()) rclcpp::shutdown();
  if (publish_spin_thread_ && publish_spin_thread_->joinable()) publish_spin_thread_->join();
  if (subscribe_spin_thread_ && subscribe_spin_thread_->joinable()) subscribe_spin_thread_->join();
}

bool ROS2::publish(
  const Eigen::Vector4d & target_pos,
  std::chrono::steady_clock::time_point captured_at) noexcept
{
  return publish2nav_->send_data(target_pos, captured_at);
}

TargetDirective ROS2::subscribe_target_directive()
{
  return subscribe2nav_->subscribe_target_directive();
}

#ifdef IO_HAS_SP_MSGS
std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  return subscribe2nav_->subscribe_enemy_status();
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return subscribe2nav_->subscribe_autoaim_target();
}
#endif

}  // namespace io
