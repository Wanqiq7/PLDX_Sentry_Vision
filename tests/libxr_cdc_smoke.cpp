#include "io/gimbal/gimbal.hpp"

#include <chrono>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  if (argc != 2) {
    tools::logger()->error("usage: libxr_cdc_smoke <config-path>");
    return 2;
  }

  tools::Exiter exiter;
  io::Gimbal gimbal(argv[1]);
  std::this_thread::sleep_for(100ms);

  while (!exiter.exit()) {
    const auto now = std::chrono::steady_clock::now();
    Eigen::Quaterniond quaternion;
    try {
      quaternion = gimbal.q(now - 50ms).normalized();
    } catch (const io::AhrsSampleUnavailable &) {
      continue;
    }
    tools::logger()->info(
      "[libxr_cdc_smoke] ahrs w={:.6f} x={:.6f} y={:.6f} z={:.6f}", quaternion.w(),
      quaternion.x(), quaternion.y(), quaternion.z());
    gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}
