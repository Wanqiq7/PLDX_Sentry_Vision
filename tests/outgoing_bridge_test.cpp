#include "io/gimbal/outgoing_bridge.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <span>

using io::OutgoingBridge;
using io::TopicKind;

namespace
{
template<std::size_t Size>
bool Equals(std::span<const uint8_t> actual, const std::array<uint8_t, Size> & expected)
{
  return actual.size() == expected.size() &&
    std::equal(actual.begin(), actual.end(), expected.begin());
}
}  // namespace

int main()
{
  const std::array<uint8_t, 2> target_1{0x10, 0x11};
  const std::array<uint8_t, 3> target_2{0x20, 0x21, 0x22};
  const std::array<uint8_t, 1> fire_true{0x31};
  const std::array<uint8_t, 1> fire_false{0x30};

  {
    OutgoingBridge bridge;
    bridge.Submit(TopicKind::TARGET, target_1);
    bridge.Submit(TopicKind::TARGET, target_2);

    const auto packet = bridge.BeginNext();
    assert(packet.has_value());
    assert(Equals(*packet, target_2));
  }

  {
    OutgoingBridge bridge;
    bridge.Submit(TopicKind::TARGET, target_1);
    const auto packet = bridge.BeginNext();
    assert(packet.has_value());
    assert(Equals(*packet, target_1));

    bridge.Submit(TopicKind::FIRE, fire_true);
    assert(!bridge.BeginNext().has_value());
    bridge.Complete(true);

    const auto fire = bridge.BeginNext();
    assert(fire.has_value());
    assert(Equals(*fire, fire_true));
  }

  {
    OutgoingBridge bridge;
    bridge.Submit(TopicKind::FIRE, fire_true);
    const auto failed = bridge.BeginNext();
    assert(failed.has_value());
    assert(Equals(*failed, fire_true));
    bridge.Complete(false);
    assert(!bridge.BeginNext().has_value());
  }

  {
    OutgoingBridge bridge;
    bridge.Submit(TopicKind::TARGET, target_1);
    const auto in_flight = bridge.BeginNext();
    assert(in_flight.has_value());
    bridge.Submit(TopicKind::TARGET, target_2);
    bridge.Complete(false);

    const auto replacement = bridge.BeginNext();
    assert(replacement.has_value());
    assert(Equals(*replacement, target_2));
  }

  {
    OutgoingBridge bridge;
    bridge.Submit(TopicKind::TARGET, target_1);
    bridge.Submit(TopicKind::FIRE, fire_true);
    bridge.ForceSafeFire(fire_false);

    const auto safe_fire = bridge.BeginNext();
    assert(safe_fire.has_value());
    assert(Equals(*safe_fire, fire_false));
    bridge.Complete(true);
    assert(!bridge.BeginNext().has_value());
  }

  {
    OutgoingBridge bridge;
    bridge.ForceSafeFire(fire_false);
    bridge.Submit(TopicKind::TARGET, target_2);

    const auto safe_fire = bridge.BeginNext();
    assert(safe_fire.has_value());
    assert(Equals(*safe_fire, fire_false));
    bridge.Complete(true);

    const auto target = bridge.BeginNext();
    assert(target.has_value());
    assert(Equals(*target, target_2));
    bridge.Complete(true);
    assert(!bridge.BeginNext().has_value());
  }

  {
    OutgoingBridge bridge;
    bridge.ForceSafeFire(fire_false);
    bridge.Submit(TopicKind::TARGET, target_2);
    bridge.Submit(TopicKind::FIRE, fire_true);

    const auto safe_fire = bridge.BeginNext();
    assert(safe_fire.has_value());
    assert(Equals(*safe_fire, fire_false));
    bridge.Complete(true);

    const auto target = bridge.BeginNext();
    assert(target.has_value());
    assert(Equals(*target, target_2));
    bridge.Complete(true);

    const auto ordinary_fire = bridge.BeginNext();
    assert(ordinary_fire.has_value());
    assert(Equals(*ordinary_fire, fire_true));
    bridge.Complete(true);
    assert(!bridge.BeginNext().has_value());
  }

  return 0;
}
