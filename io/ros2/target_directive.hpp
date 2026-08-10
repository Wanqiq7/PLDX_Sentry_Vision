#ifndef IO__ROS2__TARGET_DIRECTIVE_HPP
#define IO__ROS2__TARGET_DIRECTIVE_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

namespace io
{
struct TargetDirective
{
  bool valid{false};
  std::vector<int8_t> excluded_target_ids;
  bool restrict_to_preferred_targets{false};
  std::vector<int8_t> preferred_target_ids;
};

class TargetDirectiveStore
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  TargetDirectiveStore(
    std::chrono::milliseconds typed_timeout, std::chrono::milliseconds typed_precedence)
  : typed_timeout_(typed_timeout), typed_precedence_(typed_precedence)
  {
  }

  void accept_typed(const TargetDirective & directive, TimePoint now = Clock::now())
  {
    const auto normalized = normalize(directive);
    if (!normalized) return;
    std::lock_guard<std::mutex> lock(mutex_);
    typed_ = *normalized;
    typed_received_at_ = now;
  }

  void accept_legacy_exclusions(
    std::vector<int8_t> ids, TimePoint now = Clock::now())
  {
    auto normalized = normalize_ids(std::move(ids));
    if (!normalized) return;
    std::lock_guard<std::mutex> lock(mutex_);
    legacy_exclusions_ = std::move(*normalized);
    legacy_exclusions_received_at_ = now;
  }

  void accept_legacy_preferences(
    std::vector<int8_t> ids, TimePoint now = Clock::now())
  {
    auto normalized = normalize_ids(std::move(ids));
    if (!normalized) return;
    std::lock_guard<std::mutex> lock(mutex_);
    legacy_preferences_ = std::move(*normalized);
    legacy_preferences_received_at_ = now;
  }

  TargetDirective snapshot(TimePoint now = Clock::now()) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (typed_received_at_ && now - *typed_received_at_ < typed_precedence_) {
      if (now - *typed_received_at_ < typed_timeout_) return typed_;
      return {};
    }

    TargetDirective result;
    if (is_fresh(legacy_exclusions_received_at_, now)) {
      result.excluded_target_ids = legacy_exclusions_;
    }
    if (is_fresh(legacy_preferences_received_at_, now) && !legacy_preferences_.empty()) {
      result.restrict_to_preferred_targets = true;
      result.preferred_target_ids = legacy_preferences_;
    }
    result.valid = !result.excluded_target_ids.empty() || result.restrict_to_preferred_targets;
    return result;
  }

private:
  static std::optional<std::vector<int8_t>> normalize_ids(std::vector<int8_t> ids)
  {
    if (std::any_of(ids.begin(), ids.end(), [](int8_t id) { return id < 1 || id > 8; })) {
      return std::nullopt;
    }
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
  }

  static std::optional<TargetDirective> normalize(TargetDirective directive)
  {
    if (!directive.valid) return TargetDirective{};
    auto exclusions = normalize_ids(std::move(directive.excluded_target_ids));
    auto preferences = normalize_ids(std::move(directive.preferred_target_ids));
    if (!exclusions || !preferences) return std::nullopt;
    directive.excluded_target_ids = std::move(*exclusions);
    directive.preferred_target_ids = std::move(*preferences);
    return directive;
  }

  bool is_fresh(const std::optional<TimePoint> & received_at, TimePoint now) const
  {
    return received_at && now - *received_at < legacy_timeout_;
  }

  mutable std::mutex mutex_;
  const std::chrono::milliseconds typed_timeout_;
  const std::chrono::milliseconds typed_precedence_;
  const std::chrono::milliseconds legacy_timeout_{1500};
  TargetDirective typed_;
  std::optional<TimePoint> typed_received_at_;
  std::vector<int8_t> legacy_exclusions_;
  std::optional<TimePoint> legacy_exclusions_received_at_;
  std::vector<int8_t> legacy_preferences_;
  std::optional<TimePoint> legacy_preferences_received_at_;
};
}  // namespace io

#endif  // IO__ROS2__TARGET_DIRECTIVE_HPP
