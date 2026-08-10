#include "io/gimbal/ahrs_timeline.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;
using io::AhrsTimeline;
using io::SampleResult;

int main()
{
  using Clock = AhrsTimeline::Clock;
  const auto t0 = Clock::time_point{};
  auto now = t0;
  AhrsTimeline timeline([&] { return now; });

  assert(timeline.OnSample(1000, Eigen::Quaterniond::Identity(), t0) == SampleResult::ACCEPTED);
  assert(timeline.OnSample(
           2000, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())),
           t0 + 1ms) == SampleResult::ACCEPTED);
  const auto mid = timeline.WaitAt(t0 + 500us);
  assert(mid.angularDistance(
           Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()))) < 1e-9);

  // A remote clock regression starts a new mapping, discards old samples, and is recovery.
  assert(timeline.OnSample(500, Eigen::Quaterniond::Identity(), t0 + 2ms) == SampleResult::RECOVERED);
  assert(timeline.OnSample(600, Eigen::Quaterniond::Identity(), t0 + 3ms) == SampleResult::ACCEPTED);
  assert(timeline.WaitAt(t0 + 2ms).angularDistance(Eigen::Quaterniond::Identity()) < 1e-12);

  assert(timeline.Fresh(t0 + 3ms));
  assert(!timeline.Fresh(t0 + 153ms));
  assert(timeline.MarkStale(t0 + 153ms));
  assert(!timeline.Fresh(t0 + 153ms));
  assert(!timeline.MarkStale(t0 + 154ms));

  // Recovery remaps even without remote timestamp regression.
  assert(timeline.OnSample(700, Eigen::Quaterniond::Identity(), t0 + 160ms) == SampleResult::RECOVERED);
  assert(timeline.OnSample(
           800, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())),
           t0 + 161ms) == SampleResult::ACCEPTED);
  assert(timeline.WaitAt(t0 + 160050us).angularDistance(
           Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()))) < 1e-9);

  const auto nan = std::numeric_limits<double>::quiet_NaN();
  assert(timeline.OnSample(900, Eigen::Quaterniond(nan, 0, 0, 1), t0 + 162ms) == SampleResult::REJECTED);
  assert(timeline.OnSample(901, Eigen::Quaterniond(0, 0, 0, 0), t0 + 162ms) == SampleResult::REJECTED);
  Eigen::Quaterniond non_unit(2, 0, 0, 0);
  assert(timeline.OnSample(902, non_unit, t0 + 163ms) == SampleResult::ACCEPTED);
  assert(std::abs(timeline.WaitAt(t0 + 160202us).norm() - 1.0) < 1e-12);

  Eigen::Quaterniond huge(1e308, -1e308, 1e308, -1e308);
  assert(timeline.OnSample(903, huge, t0 + 164ms) == SampleResult::ACCEPTED);
  assert(std::abs(timeline.WaitAt(t0 + 160203us).norm() - 1.0) < 1e-12);
  Eigen::Quaterniond small(0.6e-6, 0.6e-6, 0.6e-6, 0.6e-6);
  assert(timeline.OnSample(904, small, t0 + 165ms) == SampleResult::ACCEPTED);
  assert(std::abs(timeline.WaitAt(t0 + 160204us).norm() - 1.0) < 1e-12);
  assert(timeline.OnSample(905, Eigen::Quaterniond(1e-6, 0, 0, 0), t0 + 166ms) == SampleResult::REJECTED);
  assert(timeline.OnSample(906, Eigen::Quaterniond(0.5e-6, 0.5e-6, 0.5e-6, 0.5e-6), t0 + 166ms) == SampleResult::REJECTED);

  // WaitAt detects the freshness deadline itself and resumes only after recovery brackets the request.
  auto waiter_now = t0;
  AhrsTimeline waiter_timeline([&] { return waiter_now; });
  waiter_timeline.OnSample(1, Eigen::Quaterniond::Identity(), t0);
  waiter_timeline.OnSample(2, Eigen::Quaterniond::Identity(), t0 + 1ms);
  waiter_now = t0 + 151ms;
  std::atomic<bool> returned{false};
  std::atomic<bool> unavailable{false};
  std::thread waiter([&] {
    try {
      (void)waiter_timeline.WaitAt(t0 + 200500us);
    } catch (const io::AhrsSampleUnavailable &) {
      unavailable = true;
    }
    returned = true;
  });
  std::this_thread::sleep_for(5ms);
  assert(!returned.load());
  assert(waiter_timeline.MarkStale(t0 + 151ms));
  assert(waiter_timeline.OnSample(300000, Eigen::Quaterniond::Identity(), t0 + 200ms) == SampleResult::RECOVERED);
  waiter_timeline.OnSample(301000, Eigen::Quaterniond::Identity(), t0 + 201ms);
  waiter.join();
  assert(returned.load());
  assert(!unavailable.load());

  // A request before a recovery anchor is permanently invalidated once the fresh recovered
  // deque establishes a later front timestamp. It must never receive an unrelated quaternion.
  auto fallback_now = t0;
  AhrsTimeline fallback_timeline([&] { return fallback_now; });
  const Eigen::Quaterniond old_generation = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond new_generation(
    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  fallback_timeline.OnSample(1000, old_generation, t0);
  fallback_timeline.OnSample(2000, old_generation, t0 + 1ms);
  fallback_now = t0 + 151ms;
  assert(fallback_timeline.MarkStale(fallback_now));

  std::atomic<bool> fallback_returned{false};
  std::atomic<bool> fallback_unavailable{false};
  std::thread fallback_waiter([&] {
    try {
      (void)fallback_timeline.WaitAt(t0 + 50ms);
    } catch (const io::AhrsSampleUnavailable &) {
      fallback_unavailable = true;
    }
    fallback_returned = true;
  });
  std::this_thread::sleep_for(5ms);
  assert(!fallback_returned.load());

  fallback_now = t0 + 200ms;
  assert(
    fallback_timeline.OnSample(300000, new_generation, fallback_now) ==
    SampleResult::RECOVERED);
  const auto deadline = std::chrono::steady_clock::now() + 100ms;
  while (!fallback_returned.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  const bool returned_after_recovery = fallback_returned.load();
  fallback_waiter.join();
  assert(returned_after_recovery);
  assert(fallback_unavailable.load());

  // A normal bracket remains valid, while a future request waits for a later sample.
  auto future_now = t0;
  AhrsTimeline future_timeline([&] { return future_now; });
  future_timeline.OnSample(1, old_generation, t0);
  std::atomic<bool> future_returned{false};
  std::thread future_waiter([&] {
    (void)future_timeline.WaitAt(t0 + 1ms);
    future_returned = true;
  });
  std::this_thread::sleep_for(5ms);
  assert(!future_returned.load());
  future_timeline.OnSample(1001, new_generation, t0 + 1ms);
  future_waiter.join();

  timeline.Reset();
  assert(!timeline.Fresh(t0 + 1s));
  return 0;
}
