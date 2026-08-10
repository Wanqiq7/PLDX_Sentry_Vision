#include "io/ros2/capture_timestamp.hpp"
#include "io/ros2/latest_value_publisher.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

int main()
{
  std::mutex mutex;
  std::condition_variable entered;
  std::condition_variable release;
  bool callback_entered = false;
  bool callback_released = false;
  std::atomic<int> last_published{-1};

  io::LatestValuePublisher<int> publisher([&](const int & value) {
    last_published = value;
    std::unique_lock lock(mutex);
    callback_entered = true;
    entered.notify_one();
    release.wait(lock, [&] { return callback_released; });
  });

  assert(publisher.submit(0));
  {
    std::unique_lock lock(mutex);
    assert(entered.wait_for(lock, 1s, [&] { return callback_entered; }));
  }

  const auto begin = std::chrono::steady_clock::now();
  for (int value = 1; value <= 1000; ++value) publisher.submit(value);
  const auto submission_time = std::chrono::steady_clock::now() - begin;
  assert(submission_time < 20ms);

  {
    std::lock_guard lock(mutex);
    callback_released = true;
  }
  release.notify_one();

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (last_published.load() != 1000 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  assert(last_published == 1000);

  const auto steady_now = std::chrono::steady_clock::time_point(10s);
  assert(io::capture_timestamp_in_ros_nanoseconds(steady_now - 25ms, steady_now, 2'000'000'000) ==
         1'975'000'000);
  assert(io::capture_timestamp_in_ros_nanoseconds(steady_now + 1ms, steady_now, 2'000'000'000) ==
         2'000'000'000);
  assert(io::capture_timestamp_in_ros_nanoseconds(steady_now - 3s, steady_now, 2'000'000'000) == 0);
}
