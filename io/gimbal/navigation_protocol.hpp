#pragma once
#include <cstdint>
namespace io::navigation_protocol {
struct [[gnu::packed]] LinkDiagnostics {
  uint32_t nav_rx{}, behavior_rx{}, target_rx{}, fire_rx{};
};
static_assert(sizeof(LinkDiagnostics) == 16);
struct [[gnu::packed]] ChassisTarget {
  float vx_mps{};           /* target velocity, m/s */
  float vy_mps{};           /* target velocity, m/s */
  float vw_rad_s{};         /* target yaw rate, rad/s */
  float current_yaw{};      /* navigation yaw feedback, rad */
  float current_vx{};       /* measured velocity, m/s */
  float current_vy{};       /* measured velocity, m/s */
  float current_vw{};       /* measured yaw rate, rad/s */
  float fx_global{};        /* global-frame force, N */
  float fy_global{};        /* global-frame force, N */
  float fw_global{};        /* global-frame yaw torque, N*m */
  bool use_speed_control{}; /* true: speed path, false: direct force */
  float delta_yaw{};        /* navigation/IMU yaw delta, rad */
};

struct [[gnu::packed]] BehaviorData {
  uint8_t pitch_mode{};
  uint8_t desire_stance{};
  uint8_t desire_lifter_pos{};
  float scan_yaw_min_rad{}; /* rad */
  float scan_yaw_max_rad{}; /* rad */
  uint16_t ammo_purchase_request{};
  uint8_t revive_request{};
  uint8_t remote_revive_request{};
  uint8_t remote_ammo_request{};
  uint8_t remote_health_request{};
  bool use_limited_scan{};
  bool not_aim_enemy{};
  bool use_capacitor{};
  bool tunnel_align_active{};
  float tunnel_align_angle_rad{}; /* rad */
  bool use_gyro_mode{};
};

struct [[gnu::packed]] TeamInfo {
  struct [[gnu::packed]] AllyRobotStatus {
    uint8_t robot_id{};
    uint16_t robot_hp{};
    float robot_pos_x{};
    float robot_pos_y{};
  };
  AllyRobotStatus ally_status[4]{};
  uint16_t outpost_hp{};
  uint16_t base_hp{};
};

struct [[gnu::packed]] GameInfo {
  uint16_t game_time_remaining{};
  uint16_t coin_remaining{};
  uint32_t event_code{};
  uint8_t game_status{};
  float manual_point_x{};
  float manual_point_y{};
  uint8_t manual_key{};
  uint16_t enemy_outpost_hp{};
  uint16_t enemy_base_hp{};
};

struct [[gnu::packed]] SentryInfoOnline {
  uint16_t self_health{};
  uint16_t bullets_remaining{};
  uint16_t cooling_value{};
  uint16_t heat_limit{};
  uint16_t current_heat{};
  float sentry_pos_x{};
  float sentry_pos_y{};
  float speed_monitor_angle{};
  uint32_t sentry_info_1{};
  uint16_t sentry_info_2{};
  uint64_t sentry_info_3{};
  uint8_t energy_ratio{};
};

struct [[gnu::packed]] SentryInfoOffline {
  float yaw_camerainit_to_gimbal{}; /* deg, counterclockwise positive */
  uint8_t lifter_current_pos{};
  bool is_transformable{};
  float transform_state{};
  uint8_t capacitor_capacity{}; /* 0..100 percent; 255 means unavailable */
  float chassis_imu_yaw{};      /* rad */
  // True when the active tunnel-alignment request is within the firmware
  // alignment tolerance.
  bool tunnel_yaw_aligned{};
};

struct [[gnu::packed]] EnemyRobotStatus {
  uint8_t robot_id{};
  uint16_t robot_hp{};
  uint16_t allowed_projectile{};
  uint16_t robot_pos_x{};
  uint16_t robot_pos_y{};
};

struct [[gnu::packed]] RadarInfo {
  EnemyRobotStatus enemy_status[6]{};
  uint16_t enemy_coin_left{};
  uint16_t enemy_coin_accumulated{};
  bool is_enemy_outpost_sensed{};
};

struct [[gnu::packed]] GimbalFeedback {
  float yaw{};
  float pitch{};
  float yaw_velocity{};
  float pitch_velocity{};
  float bullet_speed{};
  uint16_t bullet_count{};
  uint8_t gimbal_mode{};
  uint8_t vision_task{};
  uint8_t valid_flags{};
  uint8_t reserved[3]{};
};

static_assert(sizeof(ChassisTarget) == 45);
static_assert(sizeof(BehaviorData) == 26);
static_assert(sizeof(SentryInfoOffline) == 16);
}  // namespace io::navigation_protocol
