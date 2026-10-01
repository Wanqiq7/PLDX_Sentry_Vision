#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace io::navigation_protocol
{
#pragma pack(push, 1)
struct ChassisTarget { float vx_mps{}, vy_mps{}, vw_rad_s{}, current_yaw{}, current_vx{}, current_vy{}, current_vw{}, fx_global{}, fy_global{}, fw_global{}; bool use_speed_control{}; float delta_yaw{}; };
struct BehaviorData { uint8_t pitch_mode{}, desire_stance{}, desire_lifter_pos{}; float scan_yaw_min_rad{}, scan_yaw_max_rad{}; uint16_t ammo_purchase_request{}; uint8_t revive_request{}, remote_revive_request{}, remote_ammo_request{}, remote_health_request{}; bool use_limited_scan{}, not_aim_enemy{}, use_capacitor{}, tunnel_align_active{}; float tunnel_align_angle_rad{}; bool use_gyro_mode{}; };
struct AllyRobotStatus { uint8_t robot_id{}; uint16_t robot_hp{}; float robot_pos_x{}, robot_pos_y{}; };
struct TeamInfo { AllyRobotStatus ally_status[4]{}; uint16_t outpost_hp{}, base_hp{}; };
struct GameInfo { uint16_t game_time_remaining{}, coin_remaining{}; uint32_t event_code{}; uint8_t game_status{}; float manual_point_x{}, manual_point_y{}; uint8_t manual_key{}; uint16_t enemy_outpost_hp{}, enemy_base_hp{}; };
struct SentryInfoOnline { uint16_t self_health{}, bullets_remaining{}, cooling_value{}, heat_limit{}, current_heat{}; float sentry_pos_x{}, sentry_pos_y{}, speed_monitor_angle{}; uint32_t sentry_info_1{}; uint16_t sentry_info_2{}; uint64_t sentry_info_3{}; uint8_t energy_ratio{}; };
struct SentryInfoOffline { float yaw_camerainit_to_gimbal{}; uint8_t lifter_current_pos{}; bool is_transformable{}; float transform_state{}; uint8_t capacitor_capacity{}; float chassis_imu_yaw{}; bool tunnel_yaw_aligned{}; };
struct EnemyRobotStatus { uint8_t robot_id{}; uint16_t robot_hp{}, allowed_projectile{}, robot_pos_x{}, robot_pos_y{}; };
struct RadarInfo { EnemyRobotStatus enemy_status[6]{}; uint16_t enemy_coin_left{}, enemy_coin_accumulated{}; bool is_enemy_outpost_sensed{}; };
#pragma pack(pop)

inline constexpr char NAV_DATA_TOPIC[] = "nav_data";
inline constexpr char BEHAVIOR_DATA_TOPIC[] = "behavior_data";
inline constexpr char TEAM_INFO_TOPIC[] = "team_info";
inline constexpr char GAME_INFO_TOPIC[] = "game_info";
inline constexpr char ONLINE_INFO_TOPIC[] = "online_info";
inline constexpr char OFFLINE_INFO_TOPIC[] = "offline_info";
inline constexpr char RADAR_INFO_TOPIC[] = "radar_info";

static_assert(sizeof(ChassisTarget) == 45 && sizeof(BehaviorData) == 26);
static_assert(sizeof(TeamInfo) == 48 && sizeof(GameInfo) == 22);
static_assert(sizeof(SentryInfoOnline) == 37 && sizeof(SentryInfoOffline) == 16);
static_assert(sizeof(RadarInfo) == 59 && sizeof(bool) == 1);
static_assert(offsetof(BehaviorData, use_gyro_mode) == 25);
static_assert(std::is_trivially_copyable_v<ChassisTarget> && std::is_trivially_copyable_v<BehaviorData>);
static_assert(std::is_trivially_copyable_v<TeamInfo> && std::is_trivially_copyable_v<GameInfo>);
static_assert(std::is_trivially_copyable_v<SentryInfoOnline> && std::is_trivially_copyable_v<SentryInfoOffline>);
static_assert(std::is_trivially_copyable_v<RadarInfo>);
}  // namespace io::navigation_protocol
