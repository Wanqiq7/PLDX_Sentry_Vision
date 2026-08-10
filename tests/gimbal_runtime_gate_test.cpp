#include "io/gimbal/gimbal_runtime_gate.hpp"
#include "io/gimbal/outgoing_bridge.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <span>

using namespace std::chrono_literals;

namespace
{
using io::GimbalRuntimeGate;
using io::OutgoingBridge;
using io::SampleResult;
using io::TopicKind;

template<std::size_t Size>
bool Equals(std::span<const uint8_t> actual, const std::array<uint8_t, Size> & expected)
{
  return actual.size() == expected.size() &&
    std::equal(actual.begin(), actual.end(), expected.begin());
}

void AssertSafeTargetFireOrder(OutgoingBridge & bridge)
{
  const std::array<uint8_t, 1> safe_fire{0x30};
  const std::array<uint8_t, 2> target{0x20, 0x21};
  const std::array<uint8_t, 1> true_fire{0x31};

  auto packet = bridge.BeginNext();
  assert(packet.has_value());
  assert(Equals(*packet, safe_fire));
  bridge.Complete(true);

  packet = bridge.BeginNext();
  assert(packet.has_value());
  assert(Equals(*packet, target));
  bridge.Complete(true);

  packet = bridge.BeginNext();
  assert(packet.has_value());
  assert(Equals(*packet, true_fire));
  bridge.Complete(true);
  assert(!bridge.BeginNext().has_value());
}

void RunRecoveryOrder(bool watchdog_first)
{
  using Clock = GimbalRuntimeGate::Clock;
  const auto t0 = Clock::time_point{};
  const std::array<uint8_t, 1> safe_fire{0x30};
  const std::array<uint8_t, 1> old_target{0x10};
  const std::array<uint8_t, 2> target{0x20, 0x21};
  const std::array<uint8_t, 1> true_fire{0x31};

  OutgoingBridge bridge;
  int safe_fire_latches = 0;
  GimbalRuntimeGate gate([&] {
    ++safe_fire_latches;
    bridge.ForceSafeFire(safe_fire);
  });

  assert(gate.OnSample(1000, Eigen::Quaterniond::Identity(), t0) == SampleResult::ACCEPTED);
  assert(safe_fire_latches == 0);
  assert(gate.SubmitWhenFresh(t0, [&] { bridge.Submit(TopicKind::TARGET, old_target); }));

  if (watchdog_first) {
    assert(gate.MarkStale(t0 + 150ms));
    assert(safe_fire_latches == 1);
    assert(!gate.SubmitWhenFresh(
      t0 + 150ms, [&] { bridge.Submit(TopicKind::TARGET, target); }));
    assert(
      gate.OnSample(2000, Eigen::Quaterniond::Identity(), t0 + 151ms) ==
      SampleResult::RECOVERED);
  } else {
    assert(
      gate.OnSample(2000, Eigen::Quaterniond::Identity(), t0 + 150ms) ==
      SampleResult::RECOVERED);
    assert(safe_fire_latches == 1);
  }

  assert(gate.SubmitWhenFresh(
    t0 + 151ms, [&] { bridge.Submit(TopicKind::TARGET, target); }));
  assert(gate.SubmitWhenFresh(
    t0 + 151ms, [&] { bridge.Submit(TopicKind::FIRE, true_fire); }));
  assert(safe_fire_latches == 1);
  AssertSafeTargetFireOrder(bridge);
}

void RunTimestampRegressionRecovery()
{
  using Clock = GimbalRuntimeGate::Clock;
  const auto t0 = Clock::time_point{};
  const std::array<uint8_t, 1> safe_fire{0x30};
  const std::array<uint8_t, 1> old_target{0x10};
  const std::array<uint8_t, 1> target{0x20};

  OutgoingBridge bridge;
  int safe_fire_latches = 0;
  GimbalRuntimeGate gate([&] {
    ++safe_fire_latches;
    bridge.ForceSafeFire(safe_fire);
  });
  assert(gate.OnSample(1000, Eigen::Quaterniond::Identity(), t0) == SampleResult::ACCEPTED);
  assert(gate.SubmitWhenFresh(t0, [&] { bridge.Submit(TopicKind::TARGET, old_target); }));
  assert(gate.OnSample(500, Eigen::Quaterniond::Identity(), t0 + 1ms) == SampleResult::RECOVERED);
  assert(safe_fire_latches == 1);
  assert(gate.SubmitWhenFresh(t0 + 1ms, [&] { bridge.Submit(TopicKind::TARGET, target); }));

  auto packet = bridge.BeginNext();
  assert(packet.has_value() && Equals(*packet, safe_fire));
  bridge.Complete(true);
  packet = bridge.BeginNext();
  assert(packet.has_value() && Equals(*packet, target));
}

void RunSubmissionDeadlineTransition()
{
  using Clock = GimbalRuntimeGate::Clock;
  const auto t0 = Clock::time_point{};
  const std::array<uint8_t, 1> safe_fire{0x30};
  const std::array<uint8_t, 1> pending_target{0x10};
  const std::array<uint8_t, 1> rejected_target{0x20};

  OutgoingBridge bridge;
  int safe_fire_latches = 0;
  GimbalRuntimeGate gate([&] {
    ++safe_fire_latches;
    bridge.ForceSafeFire(safe_fire);
  });
  assert(gate.OnSample(1000, Eigen::Quaterniond::Identity(), t0) == SampleResult::ACCEPTED);
  assert(gate.SubmitWhenFresh(
    t0, [&] { bridge.Submit(TopicKind::TARGET, pending_target); }));

  assert(!gate.SubmitWhenFresh(
    t0 + 150ms, [&] { bridge.Submit(TopicKind::TARGET, rejected_target); }));
  assert(safe_fire_latches == 1);
  assert(!gate.MarkStale(t0 + 150ms));
  assert(safe_fire_latches == 1);

  const auto packet = bridge.BeginNext();
  assert(packet.has_value() && Equals(*packet, safe_fire));
  bridge.Complete(true);
  assert(!bridge.BeginNext().has_value());
}

void RunPassiveFalseFireClearsTarget()
{
  using Clock = GimbalRuntimeGate::Clock;
  const auto t0 = Clock::time_point{};
  const std::array<uint8_t, 1> safe_fire{0x30};
  const std::array<uint8_t, 1> target{0x20};

  OutgoingBridge bridge;
  int safe_fire_latches = 0;
  GimbalRuntimeGate gate([&] {
    ++safe_fire_latches;
    bridge.ForceSafeFire(safe_fire);
  });
  assert(gate.OnSample(1000, Eigen::Quaterniond::Identity(), t0) == SampleResult::ACCEPTED);
  assert(gate.SubmitWhenFresh(t0, [&] { bridge.Submit(TopicKind::TARGET, target); }));
  gate.SubmitPassiveFalseFire();
  gate.SubmitPassiveFalseFire();
  assert(safe_fire_latches == 2);

  const auto packet = bridge.BeginNext();
  assert(packet.has_value() && Equals(*packet, safe_fire));
  bridge.Complete(true);
  assert(!bridge.BeginNext().has_value());
}
}  // namespace

int main()
{
  RunRecoveryOrder(true);
  RunRecoveryOrder(false);
  RunTimestampRegressionRecovery();
  RunSubmissionDeadlineTransition();
  RunPassiveFalseFireClearsTarget();
  return 0;
}
