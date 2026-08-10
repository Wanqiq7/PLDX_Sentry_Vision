#include "io/gimbal/gimbal_config.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace io
{
RuntimeConfig LoadGimbalConfig(const std::string & path)
{
  try {
    const auto yaml = YAML::LoadFile(path);
    if (!yaml["com_port"] || !yaml["com_port"].IsScalar()) {
      throw std::invalid_argument("com_port is required");
    }
    const auto device = yaml["com_port"].as<std::string>();
    const auto non_space = std::find_if(device.begin(), device.end(), [](unsigned char value) {
      return std::isspace(value) == 0;
    });
    if (non_space == device.end()) {
      throw std::invalid_argument("com_port must not be empty");
    }
    RuntimeConfig config{device, 921600, 1, 23.0};
    if (yaml["baudrate"]) {
      const auto baudrate = yaml["baudrate"].as<long long>();
      if (baudrate < 1 || baudrate > 4000000) {
        throw std::invalid_argument("baudrate must be in 1..4000000");
      }
      config.baudrate = static_cast<uint32_t>(baudrate);
    }
    if (yaml["default_mode"]) {
      const auto mode = yaml["default_mode"].as<std::string>();
      if (mode == "IDLE") config.default_mode = 0;
      else if (mode == "AUTO_AIM") config.default_mode = 1;
      else if (mode == "SMALL_BUFF") config.default_mode = 2;
      else if (mode == "BIG_BUFF") config.default_mode = 3;
      else throw std::invalid_argument("unknown default_mode: " + mode);
    }
    if (yaml["default_bullet_speed"]) {
      config.default_bullet_speed = yaml["default_bullet_speed"].as<double>();
      if (!(config.default_bullet_speed > 0.0)) {
        throw std::invalid_argument("default_bullet_speed must be positive");
      }
    }
    if (yaml["transport_diagnostics_enabled"]) {
      if (!yaml["transport_diagnostics_enabled"].IsScalar()) {
        throw std::invalid_argument("transport_diagnostics_enabled must be boolean");
      }
      config.transport_diagnostics_enabled = yaml["transport_diagnostics_enabled"].as<bool>();
    }
    return config;
  } catch (const std::invalid_argument & e) {
    tools::logger()->error("[Gimbal] Invalid config {}: {}", path, e.what());
    throw;
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to parse config {}: {}", path, e.what());
    throw std::invalid_argument(e.what());
  }
}
}  // namespace io
