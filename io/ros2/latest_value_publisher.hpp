#ifndef IO__ROS2__LATEST_VALUE_PUBLISHER_HPP
#define IO__ROS2__LATEST_VALUE_PUBLISHER_HPP

#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace io
{

// A capacity-one handoff for latency-sensitive producers. submit() never waits:
// it replaces the pending value when the slot is available, or drops the new
// value during the very short slot swap performed by the consumer.
template<typename T>
class LatestValuePublisher
{
public:
  using PublishFunction = std::function<void(const T &)>;

  explicit LatestValuePublisher(PublishFunction publish)
  : publish_(std::move(publish)), worker_([this] { run(); })
  {
  }

  LatestValuePublisher(const LatestValuePublisher &) = delete;
  LatestValuePublisher & operator=(const LatestValuePublisher &) = delete;

  ~LatestValuePublisher() { stop(); }

  bool submit(T value) noexcept
  {
    std::unique_lock lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || stopping_) return false;
    pending_ = std::move(value);
    lock.unlock();
    ready_.notify_one();
    return true;
  }

  void stop() noexcept
  {
    {
      std::lock_guard lock(mutex_);
      stopping_ = true;
      pending_.reset();
    }
    ready_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

private:
  void run()
  {
    while (true) {
      std::optional<T> value;
      {
        std::unique_lock lock(mutex_);
        ready_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
        if (stopping_) return;
        value.swap(pending_);
      }
      publish_(*value);
    }
  }

  PublishFunction publish_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<T> pending_;
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace io

#endif  // IO__ROS2__LATEST_VALUE_PUBLISHER_HPP
