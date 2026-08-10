#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <string>

#include "io/command.hpp"
#include "io/gimbal/gimbal_runtime.hpp"
#include "io/gimbal/state_types.hpp"

namespace io
{
struct VisionToGimbal
{
  uint8_t mode = 0;  // 0: no control, 1: control without fire, 2: control and fire
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;
};

enum class GimbalMode
{
  IDLE,
  AUTO_AIM,
  SMALL_BUFF,
  BIG_BUFF
};

enum class GimbalControlMode : uint8_t
{
  RELAX,
  COMMON,
  AUTOPATROL,
  LOW_SENSITIVITY
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);
  ~Gimbal() = default;

  GimbalMode mode() const;
  GimbalControlMode control_mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point t) { return q(t); }
  io::Mode legacy_mode() const;
  double bullet_speed_value() const { return state().bullet_speed; }
  io::ShootMode shoot_mode_value() const;
  void send(const io::Command & command)
  {
    send(command.control, command.shoot, static_cast<float>(command.yaw), 0, 0,
      static_cast<float>(command.pitch), 0, 0);
  }
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);
  void send(io::VisionToGimbal command);

private:
  std::shared_ptr<GimbalRuntime> runtime_;
  GimbalMode default_mode_ = GimbalMode::AUTO_AIM;
  double default_bullet_speed_ = 23.0;
};
}  // namespace io

#endif  // IO__GIMBAL_HPP
