#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <string>

#include "io/gimbal/ahrs_timeline.hpp"
#include "io/gimbal/libxr_protocol.hpp"

namespace io
{
struct RuntimeConfig
{
  std::string device;
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
  float bullet_speed = 0.0F;
  uint16_t bullet_count = 0;
  uint8_t gimbal_mode = 0;
  uint8_t shoot_mode = 0;
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
