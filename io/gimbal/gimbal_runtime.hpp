#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <string>

#include "io/gimbal/ahrs_timeline.hpp"
#include "io/gimbal/libxr_protocol.hpp"
#include "io/ros2/navigation_protocol.hpp"

namespace io
{
struct RuntimeConfig
{
  std::string device;
  // When control_interface is set, LinuxUART discovers the CDC device by
  // VID/PID and the parent CDC control-interface string.  device remains for
  // PTY/unit-test fixtures and legacy non-USB transports.
  std::string transport_vid;
  std::string transport_pid;
  std::string transport_control_interface;
  uint32_t baudrate;
  int default_mode;
  double default_bullet_speed;
  bool transport_diagnostics_enabled = false;
};

struct RuntimeSnapshot
{
  int default_mode;
  double default_bullet_speed;
  bool fresh_ahrs;
  float yaw = 0.0F, yaw_velocity = 0.0F, pitch = 0.0F, pitch_velocity = 0.0F;
  float bullet_speed = 0.0F;
  uint16_t bullet_count = 0;
  uint8_t gimbal_mode = 0;
  uint8_t vision_task = 0;
  uint8_t feedback_flags = 0;
  bool fresh_feedback = false;
  uint64_t rx_topic_updates = 0, rx_bytes = 0, tx_publish_requests = 0;
  uint64_t tx_pack_failures = 0, rx_invalid_payload_failures = 0;
  uint64_t read_failures = 0, write_failures = 0;
};

class GimbalRuntime
{
public:
  // The first acquisition owns the process-lifetime runtime; later acquisitions throw.
  static GimbalRuntime & Instance(const RuntimeConfig & config);

  GimbalRuntime(const GimbalRuntime &) = delete;
  GimbalRuntime & operator=(const GimbalRuntime &) = delete;
  GimbalRuntime(GimbalRuntime &&) = delete;
  GimbalRuntime & operator=(GimbalRuntime &&) = delete;

  void SendTarget(const libxr_protocol::TargetEulerPayload & target);
  void SendFire(bool fire);
  void SendPassiveFalseFire();
  void SendNavData(const navigation_protocol::ChassisTarget & value);
  void SendBehaviorData(const navigation_protocol::BehaviorData & value);
  void WaitReady();
  Eigen::Quaterniond WaitQuaternion(AhrsTimeline::Clock::time_point requested);
  [[nodiscard]] RuntimeSnapshot Snapshot() const;
  [[nodiscard]] bool HasFreshAhrs() const;

private:
  class Impl;

  explicit GimbalRuntime(const RuntimeConfig & config);
  ~GimbalRuntime();

  std::unique_ptr<Impl> impl_;
};
}  // namespace io
