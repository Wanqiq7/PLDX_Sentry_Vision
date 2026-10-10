#include "navigation_bridge.hpp"

#include <cmath>
#include <cstring>
#include <numbers>
namespace io {
namespace {
template <class T>
std::span<const uint8_t> Bytes(const T& value) {
  return {reinterpret_cast<const uint8_t*>(&value), sizeof(value)};
}
template <class T>
T Decode(std::span<const uint8_t> bytes) {
  T value{};
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}
bool Finite(const geometry_msgs::msg::Twist& v) {
  return std::isfinite(v.linear.x) && std::isfinite(v.linear.y) &&
         std::isfinite(v.angular.z);
}
}  // namespace
NavigationBridge::NavigationBridge(GimbalRuntime& runtime, bool telemetry_only)
    : Node("navigation_usb_bridge"),
      runtime_(runtime),
      telemetry_only_(telemetry_only),
      tf_(get_clock()),
      listener_(tf_) {
  velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (Finite(*msg)) {
          velocity_ = *msg;
          velocity_at_ = std::chrono::steady_clock::now();
        }
      });
  force_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/cmd_force_mpc", 10,
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
        if (std::isfinite(msg->wrench.force.x) &&
            std::isfinite(msg->wrench.force.y) &&
            std::isfinite(msg->wrench.torque.z)) {
          force_ = *msg;
          force_at_ = std::chrono::steady_clock::now();
        }
      });
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (Finite(msg->twist.twist)) {
          odometry_ = *msg;
          odometry_at_ = std::chrono::steady_clock::now();
        }
      });
  behavior_sub_ = create_subscription<ros_interfaces::msg::Behavior>(
      "/sentry/behaivor_send", 10,
      [this](ros_interfaces::msg::Behavior::ConstSharedPtr msg) {
        behavior_ = *msg;
        behavior_at_ = std::chrono::steady_clock::now();
      });
  team_pub_ = create_publisher<ros_interfaces::msg::TeamInformation>(
      "/sentry/team_info", 10);
  game_pub_ =
      create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 10);
  online_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOnline>(
      "/sentry/online_info", 10);
  offline_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOffline>(
      "/sentry/offline_info", 10);
  radar_pub_ = create_publisher<ros_interfaces::msg::RadarInfo>(
      "/sentry/radar_info", 10);
  diagnostics_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>(
      "/sentry/link_diagnostics", 10);
  runtime_.SetNavigationReceiver(
      [this](const auto& name, auto bytes) { Receive(name, bytes); });
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { Tick(); });
}
NavigationBridge::~NavigationBridge() { runtime_.SetNavigationReceiver({}); }
void NavigationBridge::Tick() {
  if (telemetry_only_) return;
  using namespace navigation_protocol;
  const auto now = std::chrono::steady_clock::now();
  const bool behavior_fresh =
      now - behavior_at_ < std::chrono::milliseconds(500);
  ChassisTarget nav{};
  nav.use_speed_control = true;
  BehaviorData wire{};
  if (behavior_fresh) {
    const auto& b = behavior_;
    wire.pitch_mode = b.pitch_mode;
    wire.desire_stance = b.desired_stance;
    wire.desire_lifter_pos = b.desire_lifter_pos;
    wire.scan_yaw_min_rad = b.scan_yaw_min * std::numbers::pi_v<float> / 180.0F;
    wire.scan_yaw_max_rad = b.scan_yaw_max * std::numbers::pi_v<float> / 180.0F;
    wire.ammo_purchase_request = b.ammo_purchase_request;
    wire.revive_request = b.revive_request;
    wire.remote_revive_request = b.remote_revive_request;
    wire.remote_ammo_request = b.remote_ammo_request;
    wire.remote_health_request = b.remote_health_request;
    wire.use_limited_scan = b.use_limited_scan;
    wire.not_aim_enemy = b.not_aim_enemy;
    wire.use_capacitor = b.use_capacitor;
    wire.tunnel_align_active = b.tunnel_align_active;
    wire.tunnel_align_angle_rad =
        b.tunnel_align_angle_deg * std::numbers::pi_v<float> / 180.0F;
    wire.use_gyro_mode = b.use_gyro_mode;
    if (!std::isfinite(wire.scan_yaw_min_rad) ||
        !std::isfinite(wire.scan_yaw_max_rad) ||
        !std::isfinite(wire.tunnel_align_angle_rad))
      wire = {};
  }
  if (behavior_fresh &&
      !(behavior_.tunnel_prepare_active && !behavior_.tunnel_ready) &&
      now - velocity_at_ < std::chrono::milliseconds(500)) {
    auto v = velocity_;
    if (behavior_.tunnel_escape_active) {
      v.linear.x = behavior_.tunnel_escape_vx;
      v.linear.y = behavior_.tunnel_escape_vy;
      v.angular.z = behavior_.tunnel_escape_wz;
    }
    if (Finite(v)) {
      nav.vw_rad_s = v.angular.z;
      try {
        const auto transform =
            tf_.lookupTransform("camera_init", "base_link", tf2::TimePointZero);
        const auto& q = transform.transform.rotation;
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z));
        const auto age =
            (get_clock()->now() - rclcpp::Time(transform.header.stamp))
                .seconds();
        if (std::isfinite(yaw) && age >= -0.1 && age < 0.5) {
          nav.current_yaw = nav.delta_yaw = yaw;
          nav.vx_mps = v.linear.x * std::sin(yaw) - v.linear.y * std::cos(yaw);
          nav.vy_mps = v.linear.x * std::cos(yaw) + v.linear.y * std::sin(yaw);
          if (now - force_at_ < std::chrono::milliseconds(500)) {
            nav.fx_global = force_.wrench.force.x;
            nav.fy_global = force_.wrench.force.y;
            nav.fw_global = force_.wrench.torque.z;
          }
        }
      } catch (const tf2::TransformException&) {
      }
    }
  }
  if (now - odometry_at_ < std::chrono::milliseconds(500)) {
    nav.current_vx = odometry_.twist.twist.linear.x;
    nav.current_vy = odometry_.twist.twist.linear.y;
    nav.current_vw = odometry_.twist.twist.angular.z;
  }
  runtime_.SendNavigation("nav_data", Bytes(nav));
  runtime_.SendNavigation("behavior_data", Bytes(wire));
}
void NavigationBridge::Receive(const std::string& name,
                               std::span<const uint8_t> bytes) {
  using namespace navigation_protocol;
  if (name == "nav_link_diagnostics" &&
      bytes.size() == sizeof(LinkDiagnostics)) {
    const auto w = Decode<LinkDiagnostics>(bytes);
    std_msgs::msg::UInt32MultiArray m;
    m.data = {w.nav_rx, w.behavior_rx, w.target_rx, w.fire_rx};
    diagnostics_pub_->publish(m);
  } else if (name == "offline_info" &&
             bytes.size() == sizeof(SentryInfoOffline)) {
    const auto w = Decode<SentryInfoOffline>(bytes);
    ros_interfaces::msg::SentryInfoOffline m;
    m.header.stamp = get_clock()->now();
    m.yaw_camerainit_to_gimbal = w.yaw_camerainit_to_gimbal;
    m.lifter_current_pos = w.lifter_current_pos;
    m.is_transformable = w.is_transformable;
    m.transform_state = w.transform_state;
    m.capacitor_capacity = w.capacitor_capacity;
    m.chassis_imu_yaw = w.chassis_imu_yaw;
    m.tunnel_yaw_aligned = w.tunnel_yaw_aligned;
    offline_pub_->publish(m);
  } else if (name == "game_info" && bytes.size() == sizeof(GameInfo)) {
    const auto w = Decode<GameInfo>(bytes);
    ros_interfaces::msg::GameInfo m;
    m.header.stamp = get_clock()->now();
    m.game_time_remaining = w.game_time_remaining;
    m.coin_remaining = w.coin_remaining;
    m.event_code = w.event_code;
    m.game_status = w.game_status;
    m.manual_point_x = w.manual_point_x;
    m.manual_point_y = w.manual_point_y;
    m.manual_key = w.manual_key;
    m.enemy_outpost_hp = w.enemy_outpost_hp;
    m.enemy_base_hp = w.enemy_base_hp;
    game_pub_->publish(m);
  } else if (name == "online_info" &&
             bytes.size() == sizeof(SentryInfoOnline)) {
    const auto w = Decode<SentryInfoOnline>(bytes);
    ros_interfaces::msg::SentryInfoOnline m;
    m.header.stamp = get_clock()->now();
    m.self_health = w.self_health;
    m.bullets_remaining = w.bullets_remaining;
    m.cooling_value = w.cooling_value;
    m.heat_limit = w.heat_limit;
    m.current_heat = w.current_heat;
    m.sentry_pos.x = w.sentry_pos_x;
    m.sentry_pos.y = w.sentry_pos_y;
    m.speed_monitor_angle = w.speed_monitor_angle;
    m.sentry_info_1 = w.sentry_info_1;
    m.sentry_info_2 = w.sentry_info_2;
    m.sentry_info_3 = w.sentry_info_3;
    m.energy_ratio = w.energy_ratio;
    online_pub_->publish(m);
  } else if (name == "team_info" && bytes.size() == sizeof(TeamInfo)) {
    const auto w = Decode<TeamInfo>(bytes);
    ros_interfaces::msg::TeamInformation m;
    m.header.stamp = get_clock()->now();
    m.outpost_hp = w.outpost_hp;
    m.base_hp = w.base_hp;
    for (size_t i = 0; i < 4; ++i) {
      m.allies[i].robot_id = w.ally_status[i].robot_id;
      m.allies[i].robot_hp = w.ally_status[i].robot_hp;
      m.allies[i].position.position.x = w.ally_status[i].robot_pos_x;
      m.allies[i].position.position.y = w.ally_status[i].robot_pos_y;
    }
    team_pub_->publish(m);
  } else if (name == "radar_info" && bytes.size() == sizeof(RadarInfo)) {
    const auto w = Decode<RadarInfo>(bytes);
    ros_interfaces::msg::RadarInfo m;
    m.header.stamp = get_clock()->now();
    m.enemy_coin_left = w.enemy_coin_left;
    m.enemy_coin_accumulated = w.enemy_coin_accumulated;
    m.is_enemy_outpost_sensed = w.is_enemy_outpost_sensed;
    for (size_t i = 0; i < 6; ++i) {
      m.enemies[i].robot_id = w.enemy_status[i].robot_id;
      m.enemies[i].robot_hp = w.enemy_status[i].robot_hp;
      m.enemies[i].allowed_projectile = w.enemy_status[i].allowed_projectile;
      m.enemies[i].position.position.x = w.enemy_status[i].robot_pos_x;
      m.enemies[i].position.position.y = w.enemy_status[i].robot_pos_y;
    }
    radar_pub_->publish(m);
  }
}
}  // namespace io
