#include "io/gimbal/gimbal_config.hpp"

#include <cassert>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>

namespace
{
void write_fixture(const std::string & name, const std::string & body)
{
  std::ofstream file(name);
  file << body;
}

void rejects(const std::string & name, const std::string & body)
{
  write_fixture(name, body);
  bool rejected = false;
  try {
    (void)io::LoadGimbalConfig(name);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
}
}  // namespace

int main()
{
  write_fixture("gimbal_config_defaults.yaml", "com_port: /dev/gimbal\n");
  const auto defaults = io::LoadGimbalConfig("gimbal_config_defaults.yaml");
  assert(defaults.device == "/dev/gimbal");
  assert(defaults.baudrate == 921600);
  assert(defaults.default_mode == 1);
  assert(defaults.default_bullet_speed == 23.0);
  assert(!defaults.transport_diagnostics_enabled);

  write_fixture("gimbal_config_values.yaml",
    "com_port: /dev/test\nbaudrate: 115200\ndefault_mode: BIG_BUFF\ndefault_bullet_speed: 30.5\n");
  const auto values = io::LoadGimbalConfig("gimbal_config_values.yaml");
  assert(values.baudrate == 115200);
  assert(values.default_mode == 3);
  assert(values.default_bullet_speed == 30.5);

  write_fixture("gimbal_config_diagnostics.yaml",
    "com_port: /dev/test\ntransport_diagnostics_enabled: true\n");
  const auto diagnostics = io::LoadGimbalConfig("gimbal_config_diagnostics.yaml");
  assert(diagnostics.transport_diagnostics_enabled);

  rejects("gimbal_config_missing.yaml", "baudrate: 115200\n");
  rejects("gimbal_config_mode.yaml", "com_port: /dev/gimbal\ndefault_mode: BAD\n");
  rejects("gimbal_config_baud_low.yaml", "com_port: /dev/gimbal\nbaudrate: 0\n");
  rejects("gimbal_config_baud_high.yaml", "com_port: /dev/gimbal\nbaudrate: 4000001\n");
  rejects("gimbal_config_baud_text.yaml", "com_port: /dev/gimbal\nbaudrate: fast\n");
  rejects("gimbal_config_speed.yaml", "com_port: /dev/gimbal\ndefault_bullet_speed: 0\n");
  rejects("gimbal_config_speed_negative.yaml",
    "com_port: /dev/gimbal\ndefault_bullet_speed: -1\n");
  rejects("gimbal_config_speed_text.yaml",
    "com_port: /dev/gimbal\ndefault_bullet_speed: fast\n");
  rejects("gimbal_config_port_sequence.yaml", "com_port: [/dev/gimbal]\n");
  rejects("gimbal_config_port_empty.yaml", "com_port: \"\"\n");
  rejects("gimbal_config_port_whitespace.yaml", "com_port: \"   \\t  \"\n");
  return 0;
}
