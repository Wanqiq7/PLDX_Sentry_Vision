#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <vector>

namespace io
{
enum class TopicKind { TARGET, FIRE };

class OutgoingBridge
{
public:
  void Submit(TopicKind topic, std::span<const uint8_t> packet);
  [[nodiscard]] std::optional<std::span<const uint8_t>> BeginNext();
  void Complete(bool success);
  void ForceSafeFire(std::span<const uint8_t> packet);

private:
  std::mutex mutex_;
  std::optional<std::vector<uint8_t>> pending_target_;
  std::optional<std::vector<uint8_t>> pending_safe_fire_;
  std::optional<std::vector<uint8_t>> pending_fire_;
  std::optional<std::vector<uint8_t>> in_flight_;
};
}  // namespace io
