#include "io/gimbal/outgoing_bridge.hpp"

#include <utility>

namespace io
{
void OutgoingBridge::Submit(TopicKind topic, std::span<const uint8_t> packet)
{
  std::lock_guard lock(mutex_);
  std::vector<uint8_t> owned(packet.begin(), packet.end());
  switch (topic) {
    case TopicKind::TARGET:
      pending_target_ = std::move(owned);
      break;
    case TopicKind::FIRE:
      pending_fire_ = std::move(owned);
      break;
  }
}

std::optional<std::span<const uint8_t>> OutgoingBridge::BeginNext()
{
  std::lock_guard lock(mutex_);
  if (in_flight_) {
    return std::nullopt;
  }

  if (pending_safe_fire_) {
    in_flight_ = std::move(pending_safe_fire_);
    pending_safe_fire_.reset();
  } else if (pending_target_) {
    in_flight_ = std::move(pending_target_);
    pending_target_.reset();
  } else if (pending_fire_) {
    in_flight_ = std::move(pending_fire_);
    pending_fire_.reset();
  } else {
    return std::nullopt;
  }

  return std::span<const uint8_t>(*in_flight_);
}

void OutgoingBridge::Complete(bool success)
{
  static_cast<void>(success);
  std::lock_guard lock(mutex_);
  in_flight_.reset();
}

void OutgoingBridge::ForceSafeFire(std::span<const uint8_t> packet)
{
  std::lock_guard lock(mutex_);
  pending_target_.reset();
  pending_fire_.reset();
  pending_safe_fire_.emplace(packet.begin(), packet.end());
}
}  // namespace io
