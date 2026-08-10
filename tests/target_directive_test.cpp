#include <chrono>
#include <cstdlib>
#include <iostream>
#include <list>
#include <string>

#include "io/ros2/target_directive.hpp"
#include "tasks/omniperception/decider.hpp"

namespace
{
using namespace std::chrono_literals;

auto_aim::Armor armor(int id)
{
  auto result = auto_aim::Armor(
    0, id - 1, 1.0F, cv::Rect{},
    {{0.0F, 0.0F}, {2.0F, 0.0F}, {2.0F, 1.0F}, {0.0F, 1.0F}});
  result.name = static_cast<auto_aim::ArmorName>(id - 1);
  return result;
}

void expect_ids(const std::list<auto_aim::Armor> & armors, std::initializer_list<int> expected)
{
  auto actual = armors.begin();
  auto wanted = expected.begin();
  while (actual != armors.end() && wanted != expected.end()) {
    if (static_cast<int>(actual->name) + 1 != *wanted) std::abort();
    ++actual;
    ++wanted;
  }
  if (actual != armors.end() || wanted != expected.end()) std::abort();
}

void hard_exclusion_removes_targets()
{
  std::list<auto_aim::Armor> armors{armor(1), armor(2), armor(3)};
  io::TargetDirective directive{true, {2}, false, {}};
  omniperception::Decider::apply_target_directive(armors, directive);
  expect_ids(armors, {1, 3});
}

void strict_selection_keeps_only_preferred_targets()
{
  std::list<auto_aim::Armor> armors{armor(1), armor(2), armor(3)};
  io::TargetDirective directive{true, {}, true, {1, 3}};
  omniperception::Decider::apply_target_directive(armors, directive);
  expect_ids(armors, {1, 3});
}

void exclusion_wins_over_preference()
{
  std::list<auto_aim::Armor> armors{armor(1), armor(2), armor(3)};
  io::TargetDirective directive{true, {2}, true, {2, 3}};
  omniperception::Decider::apply_target_directive(armors, directive);
  expect_ids(armors, {3});
}

void invalid_directive_preserves_autonomous_candidates()
{
  std::list<auto_aim::Armor> armors{armor(1), armor(2), armor(3)};
  io::TargetDirective directive{false, {1, 2, 3}, true, {}};
  omniperception::Decider::apply_target_directive(armors, directive);
  expect_ids(armors, {1, 2, 3});
}

void queued_detection_removed_when_directive_filters_all_candidates()
{
  std::vector<omniperception::DetectionResult> detections{
    {{armor(1), armor(2)}, {}, 0.0, 0.0}};
  const io::TargetDirective directive{true, {}, true, {3}};

  for (auto & detection : detections) {
    omniperception::Decider::apply_target_directive(detection.armors, directive);
  }
  omniperception::Decider::remove_empty_detections(detections);

  if (!detections.empty()) std::abort();
}

void typed_directive_expires_at_configured_timeout()
{
  io::TargetDirectiveStore store(750ms, 1s);
  const auto now = io::TargetDirectiveStore::Clock::now();
  store.accept_typed(io::TargetDirective{true, {2}, false, {}}, now);
  if (!store.snapshot(now + 749ms).valid) std::abort();
  if (store.snapshot(now + 750ms).valid) std::abort();
}

void typed_directive_precedes_legacy_for_one_second()
{
  io::TargetDirectiveStore store(750ms, 1s);
  const auto now = io::TargetDirectiveStore::Clock::now();
  store.accept_legacy_exclusions({1}, now);
  store.accept_legacy_preferences({3}, now);
  store.accept_typed(io::TargetDirective{true, {2}, false, {}}, now);

  expect_ids(
    [&] {
      std::list<auto_aim::Armor> armors{armor(1), armor(2), armor(3)};
      omniperception::Decider::apply_target_directive(armors, store.snapshot(now + 800ms));
      return armors;
    }(),
    {1, 2, 3});

  const auto legacy = store.snapshot(now + 1s);
  if (!legacy.valid || legacy.excluded_target_ids != std::vector<int8_t>{1} ||
    !legacy.restrict_to_preferred_targets ||
    legacy.preferred_target_ids != std::vector<int8_t>{3})
    std::abort();
}

void ingress_normalizes_typed_and_legacy_updates_atomically()
{
  io::TargetDirectiveStore store(750ms, 1s);
  const auto now = io::TargetDirectiveStore::Clock::now();

  store.accept_typed(io::TargetDirective{true, {7, 2, 7}, true, {3, 1, 3}}, now);
  const auto typed = store.snapshot(now);
  if (typed.excluded_target_ids != std::vector<int8_t>{2, 7} ||
    typed.preferred_target_ids != std::vector<int8_t>{1, 3}) std::abort();

  store.accept_typed(io::TargetDirective{true, {2, 9}, true, {1}}, now + 1ms);
  if (store.snapshot(now + 1ms).excluded_target_ids != std::vector<int8_t>{2, 7}) std::abort();

  store.accept_typed(io::TargetDirective{false, {0, 9}, true, {9}}, now + 2ms);
  if (store.snapshot(now + 2ms).valid) std::abort();

  store.accept_legacy_exclusions({4, 1, 4}, now + 1003ms);
  store.accept_legacy_preferences({3, 2, 3}, now + 1003ms);
  auto legacy = store.snapshot(now + 1003ms);
  if (legacy.excluded_target_ids != std::vector<int8_t>{1, 4} ||
    legacy.preferred_target_ids != std::vector<int8_t>{2, 3}) std::abort();

  store.accept_legacy_exclusions({1, 0}, now + 1004ms);
  store.accept_legacy_preferences({3, 9}, now + 1004ms);
  legacy = store.snapshot(now + 1004ms);
  if (legacy.excluded_target_ids != std::vector<int8_t>{1, 4} ||
    legacy.preferred_target_ids != std::vector<int8_t>{2, 3}) std::abort();
}

void heartbeat_bridges_source_and_consumer_freshness_boundaries()
{
  io::TargetDirectiveStore store(750ms, 1s);
  const auto start = io::TargetDirectiveStore::Clock::now();
  const io::TargetDirective referee_and_navigation{true, {2}, true, {2, 3}};
  const io::TargetDirective referee_only{true, {2}, false, {}};

  for (auto elapsed = 0ms; elapsed <= 500ms; elapsed += 50ms) {
    store.accept_typed(referee_and_navigation, start + elapsed);
  }

  std::list<auto_aim::Armor> at_navigation_boundary{armor(1), armor(2), armor(3)};
  omniperception::Decider::apply_target_directive(
    at_navigation_boundary, store.snapshot(start + 500ms));
  expect_ids(at_navigation_boundary, {3});

  store.accept_typed(referee_only, start + 550ms);
  std::list<auto_aim::Armor> after_navigation_expiry{armor(1), armor(2), armor(3)};
  omniperception::Decider::apply_target_directive(
    after_navigation_expiry, store.snapshot(start + 550ms));
  expect_ids(after_navigation_expiry, {1, 3});

  for (auto elapsed = 600ms; elapsed <= 1000ms; elapsed += 50ms) {
    store.accept_typed(referee_only, start + elapsed);
  }
  std::list<auto_aim::Armor> at_referee_boundary{armor(1), armor(2), armor(3)};
  omniperception::Decider::apply_target_directive(
    at_referee_boundary, store.snapshot(start + 1000ms));
  expect_ids(at_referee_boundary, {1, 3});

  store.accept_typed(io::TargetDirective{false, {9}, true, {9}}, start + 1050ms);
  std::list<auto_aim::Armor> after_clear{armor(1), armor(2), armor(3)};
  omniperception::Decider::apply_target_directive(after_clear, store.snapshot(start + 1050ms));
  expect_ids(after_clear, {1, 2, 3});
}

}  // namespace

int main()
{
  std::cout << "hard exclusion\n" << std::flush;
  hard_exclusion_removes_targets();
  std::cout << "strict selection\n" << std::flush;
  strict_selection_keeps_only_preferred_targets();
  std::cout << "precedence inside directive\n" << std::flush;
  exclusion_wins_over_preference();
  std::cout << "invalid fallback\n" << std::flush;
  invalid_directive_preserves_autonomous_candidates();
  std::cout << "empty queued detection\n" << std::flush;
  queued_detection_removed_when_directive_filters_all_candidates();
  std::cout << "typed timeout\n" << std::flush;
  typed_directive_expires_at_configured_timeout();
  std::cout << "typed over legacy\n" << std::flush;
  typed_directive_precedes_legacy_for_one_second();
  std::cout << "normalized ingress\n" << std::flush;
  ingress_normalizes_typed_and_legacy_updates_atomically();
  std::cout << "cross-boundary heartbeat\n" << std::flush;
  heartbeat_bridges_source_and_consumer_freshness_boundaries();
  std::cout << "target_directive_test passed\n";
}
