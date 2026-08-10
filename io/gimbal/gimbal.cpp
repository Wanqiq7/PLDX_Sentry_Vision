#include "gimbal.hpp"

#include "io/gimbal/gimbal_config.hpp"
#include "io/gimbal/gimbal_command.hpp"
#include "io/gimbal/libxr_protocol.hpp"
#include "tools/logger.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  const auto config = LoadGimbalConfig(config_path);
  runtime_ = std::shared_ptr<GimbalRuntime>(&GimbalRuntime::Instance(config), [](GimbalRuntime *) {});
  default_mode_ = static_cast<GimbalMode>(config.default_mode);
  default_bullet_speed_ = config.default_bullet_speed;
  tools::logger()->info("[Gimbal] LibXR SharedTopic runtime; waiting for live gimbal feedback.");
  runtime_->WaitReady();
}

GimbalMode Gimbal::mode() const
{
  return control_mode() == GimbalControlMode::COMMON ? GimbalMode::AUTO_AIM :
                                                       GimbalMode::IDLE;
}

GimbalControlMode Gimbal::control_mode() const
{
  const auto snapshot = runtime_->Snapshot();
  return snapshot.fresh_feedback &&
      (snapshot.feedback_flags & libxr_protocol::FEEDBACK_GIMBAL_MODE_VALID) != 0U ?
    static_cast<GimbalControlMode>(snapshot.gimbal_mode) : GimbalControlMode::RELAX;
}

GimbalState Gimbal::state() const
{
  const auto snapshot = runtime_->Snapshot();
  return {0.0F, 0.0F, 0.0F, 0.0F, snapshot.bullet_speed, snapshot.bullet_count};
}

io::ShootMode Gimbal::shoot_mode_value() const
{
  const auto snapshot = runtime_->Snapshot();
  return snapshot.fresh_feedback &&
      (snapshot.feedback_flags & libxr_protocol::FEEDBACK_SHOOT_MODE_VALID) != 0U ?
    static_cast<io::ShootMode>(snapshot.shoot_mode) : io::left_shoot;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE: return "IDLE";
    case GimbalMode::AUTO_AIM: return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF: return "BIG_BUFF";
    default: return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  return runtime_->WaitQuaternion(t);
}

io::Mode Gimbal::legacy_mode() const
{
  switch (mode()) {
    case GimbalMode::AUTO_AIM: return io::auto_aim;
    case GimbalMode::SMALL_BUFF: return io::small_buff;
    case GimbalMode::BIG_BUFF: return io::big_buff;
    default: return io::idle;
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  if (control) {
    runtime_->SendTarget(libxr_protocol::EncodeTarget(
      yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc));
    runtime_->SendFire(fire);
  } else {
    runtime_->SendPassiveFalseFire();
  }
}

void Gimbal::send(io::VisionToGimbal command)
{
  const auto mode = gimbal_detail::DecodeCommandMode(command.mode);
  if (!mode.valid) {
    tools::logger()->warn("[Gimbal] Invalid VisionToGimbal mode {}; failing closed.", command.mode);
  }
  send(mode.control, mode.fire, command.yaw, command.yaw_vel, command.yaw_acc, command.pitch,
    command.pitch_vel, command.pitch_acc);
}
}  // namespace io
