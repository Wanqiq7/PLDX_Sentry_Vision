#pragma once

#include <Eigen/Geometry>

#include <bit>
#include <cstdint>

namespace io::libxr_protocol
{
static_assert(std::endian::native == std::endian::little, "LibXR topic payloads require little-endian");

inline constexpr char TARGET_EULER_TOPIC[] = "target_euler";
inline constexpr char FIRE_NOTIFY_TOPIC[] = "fire_notify";
inline constexpr char AHRS_QUATERNION_TOPIC[] = "ahrs_quaternion";
inline constexpr char NAV_GIMBAL_FEEDBACK_TOPIC[] = "nav_gimbal_feedback";
inline constexpr uint8_t FEEDBACK_YAW_VALID = 1U << 0U;
inline constexpr uint8_t FEEDBACK_PITCH_VALID = 1U << 1U;
inline constexpr uint8_t FEEDBACK_ANGULAR_VELOCITY_VALID = 1U << 2U;
inline constexpr uint8_t FEEDBACK_BULLET_SPEED_VALID = 1U << 3U;
inline constexpr uint8_t FEEDBACK_BULLET_COUNT_VALID = 1U << 4U;
inline constexpr uint8_t FEEDBACK_GIMBAL_MODE_VALID = 1U << 5U;
inline constexpr uint8_t FEEDBACK_VISION_TASK_VALID = 1U << 6U;
inline constexpr uint8_t FEEDBACK_VALID_MASK = 0x7FU;
inline constexpr uint8_t GIMBAL_MODE_RELAX = 0U;
inline constexpr uint8_t GIMBAL_MODE_COMMON = 1U;
inline constexpr uint8_t GIMBAL_MODE_AUTOPATROL = 2U;
inline constexpr uint8_t GIMBAL_MODE_LOW_SENSITIVITY = 3U;

struct TargetEulerPayload
{
  float rol;
  float pit;
  float yaw;
  float rol_dot;
  float pit_dot;
  float yaw_dot;
  float rol_ddot;
  float pit_ddot;
  float yaw_ddot;
};

struct FirePayload
{
  bool isfire = false;
};

struct QuaternionPayload
{
  float x;
  float y;
  float z;
  float w;
};

// MCU -> vision feedback published by the gimbal over USB CDC.  All numeric
// fields use SI units: bullet_speed is m/s and angles (when added by a
// producer) are radians.  The topic name is part of the wire contract.
struct [[gnu::packed]] GimbalFeedbackPayload
{
  float yaw = 0.0F;
  float pitch = 0.0F;
  float yaw_velocity = 0.0F;
  float pitch_velocity = 0.0F;
  float bullet_speed = 0.0F;
  uint16_t bullet_count = 0;
  uint8_t gimbal_mode = 0;
  uint8_t vision_task = 0;
  uint8_t valid_flags = 0;
  uint8_t reserved[3] = {};
};

static_assert(sizeof(TargetEulerPayload) == 36);
static_assert(sizeof(FirePayload) == 1);
static_assert(sizeof(QuaternionPayload) == 16);
static_assert(sizeof(GimbalFeedbackPayload) == 28);

inline TargetEulerPayload EncodeTarget(
  float yaw, float yaw_dot, float yaw_ddot, float pit, float pit_dot, float pit_ddot)
{
  return {0.0F, pit, yaw, 0.0F, pit_dot, yaw_dot, 0.0F, pit_ddot, yaw_ddot};
}

inline Eigen::Quaterniond DecodeQuaternion(const QuaternionPayload & raw)
{
  return {raw.w, raw.x, raw.y, raw.z};
}
}  // namespace io::libxr_protocol
