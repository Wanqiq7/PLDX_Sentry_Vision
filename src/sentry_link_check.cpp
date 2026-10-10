#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "io/gimbal/gimbal_config.hpp"
#include "io/ros2/navigation_bridge.hpp"
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr
        << "Usage: sentry_link_check CONFIG [SECONDS] [--telemetry-only]\n";
    return 2;
  }
  try {
    const auto config = io::LoadGimbalConfig(argv[1]);
    const int seconds = argc > 2 ? std::stoi(argv[2]) : 10;
    if (seconds < 1 || seconds > 300) return 2;
    rclcpp::init(argc, argv);
    auto& runtime = io::GimbalRuntime::Instance(config);
    const bool telemetry_only =
        argc > 3 && std::string(argv[3]) == "--telemetry-only";
    auto node = std::make_shared<io::NavigationBridge>(runtime, telemetry_only);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto end =
        std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      if (!telemetry_only) runtime.SendFire(false);
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto s = runtime.Snapshot();
    std::cout << "rx_bytes=" << s.rx_bytes
              << " rx_topics=" << s.rx_topic_updates
              << " fresh_ahrs=" << s.fresh_ahrs
              << " fresh_feedback=" << s.fresh_feedback
              << " invalid=" << s.rx_invalid_payload_failures
              << " read_failures=" << s.read_failures
              << " write_failures=" << s.write_failures << std::endl;
    rclcpp::shutdown();
    return s.rx_topic_updates > 0 && s.read_failures == 0 &&
                   s.write_failures == 0
               ? 0
               : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
