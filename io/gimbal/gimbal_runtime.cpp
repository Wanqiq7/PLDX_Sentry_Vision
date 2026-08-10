#include "io/gimbal/gimbal_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "io/gimbal/gimbal_runtime_gate.hpp"
#include "io/gimbal/outgoing_bridge.hpp"
#include "linux_uart.hpp"
#include "message.hpp"
#include "libxr_system.hpp"
#include "tools/logger.hpp"

namespace io
{
using libxr_protocol::FirePayload;
using libxr_protocol::GimbalFeedbackPayload;
using libxr_protocol::QuaternionPayload;
using libxr_protocol::TargetEulerPayload;

class GimbalRuntime::Impl
{
public:
  explicit Impl(const RuntimeConfig & config)
  : config_(config),
    uart_(std::make_unique<LibXR::LinuxUART>(
      config.device.c_str(), config.baudrate, LibXR::UART::Parity::NO_PARITY, 8, 1, 1, 512)),
    target_topic_(LibXR::Topic::CreateTopic<TargetEulerPayload>(libxr_protocol::TARGET_EULER_TOPIC)),
    fire_topic_(LibXR::Topic::CreateTopic<FirePayload>(libxr_protocol::FIRE_NOTIFY_TOPIC)),
    ahrs_topic_(LibXR::Topic::CreateTopic<QuaternionPayload>(libxr_protocol::AHRS_QUATERNION_TOPIC)),
    feedback_topic_(LibXR::Topic::CreateTopic<GimbalFeedbackPayload>(
      libxr_protocol::NAV_GIMBAL_FEEDBACK_TOPIC)),
    server_(512),
    target_callback_(LibXR::Topic::Callback::Create(
      [](bool, Impl * self, const LibXR::Topic::RawMessageView & message) {
        self->QueuePacket(TopicKind::TARGET, self->target_topic_, message);
      },
      this)),
    fire_callback_(LibXR::Topic::Callback::Create(
      [](bool, Impl * self, const LibXR::Topic::RawMessageView & message) {
        self->QueuePacket(TopicKind::FIRE, self->fire_topic_, message);
      },
      this)),
    ahrs_callback_(LibXR::Topic::Callback::Create(
      [](bool, Impl * self, const LibXR::Topic::RawMessageView & message) {
        self->HandleAhrs(message);
      },
      this)),
    feedback_callback_(LibXR::Topic::Callback::Create(
      [](bool, Impl * self, const LibXR::Topic::RawMessageView & message) {
        self->HandleFeedback(message);
      },
      this)),
    gate_([this] { ForceSafeFire(); })
  {
    target_topic_.RegisterCallback(target_callback_);
    fire_topic_.RegisterCallback(fire_callback_);
    ahrs_topic_.RegisterCallback(ahrs_callback_);
    feedback_topic_.RegisterCallback(feedback_callback_);
    server_.Register(ahrs_topic_);
    server_.Register(feedback_topic_);

    rx_thread_ = std::thread(&Impl::ReceiveLoop, this);
    tx_thread_ = std::thread(&Impl::TransmitLoop, this);
    stale_thread_ = std::thread(&Impl::StaleLoop, this);
  }

  void SendTarget(const TargetEulerPayload & target)
  {
    gate_.SubmitWhenFresh(AhrsTimeline::Clock::now(), [&] {
      auto value = target;
      target_topic_.Publish(value);
      diagnostics_.tx_publish_requests.fetch_add(1);
    });
  }

  void SendFire(bool fire)
  {
    FirePayload payload{fire};
    if (!fire) { fire_topic_.Publish(payload); diagnostics_.tx_publish_requests.fetch_add(1); }
    else gate_.SubmitWhenFresh(
        AhrsTimeline::Clock::now(), [&] { fire_topic_.Publish(payload); diagnostics_.tx_publish_requests.fetch_add(1); });
  }

  void SendPassiveFalseFire() { gate_.SubmitPassiveFalseFire(); }

  void WaitReady() { gate_.WaitReady(); }

  Eigen::Quaterniond WaitQuaternion(AhrsTimeline::Clock::time_point requested)
  {
    return gate_.WaitQuaternion(requested);
  }

  RuntimeSnapshot Snapshot() const
  {
    std::lock_guard lock(feedback_mutex_);
    const auto fresh = feedback_received_ != AhrsTimeline::Clock::time_point{} &&
      AhrsTimeline::Clock::now() - feedback_received_ <= kFeedbackTimeout;
    const auto flags = fresh ? feedback_.valid_flags : static_cast<uint8_t>(0);
    return {config_.default_mode, config_.default_bullet_speed, HasFreshAhrs(),
      (flags & libxr_protocol::FEEDBACK_BULLET_SPEED_VALID) != 0U ?
        feedback_.bullet_speed : 0.0F,
      (flags & libxr_protocol::FEEDBACK_BULLET_COUNT_VALID) != 0U ?
        feedback_.bullet_count : static_cast<uint16_t>(0),
      (flags & libxr_protocol::FEEDBACK_GIMBAL_MODE_VALID) != 0U ?
        feedback_.gimbal_mode : static_cast<uint8_t>(0),
      (flags & libxr_protocol::FEEDBACK_SHOOT_MODE_VALID) != 0U ?
        feedback_.shoot_mode : static_cast<uint8_t>(0),
      flags, fresh, diagnostics_.rx_topic_updates.load(), diagnostics_.rx_bytes.load(),
      diagnostics_.tx_publish_requests.load(), diagnostics_.tx_pack_failures.load(),
      diagnostics_.rx_invalid_payload_failures.load(), diagnostics_.read_failures.load(),
      diagnostics_.write_failures.load()};
  }

  bool HasFreshAhrs() const { return gate_.HasFreshAhrs(AhrsTimeline::Clock::now()); }

private:
  void QueuePacket(
    TopicKind kind, LibXR::Topic::TopicHandle topic_handle,
    const LibXR::Topic::RawMessageView & message)
  {
    std::vector<uint8_t> packet(message.payload.size_ + LibXR::Topic::PACK_BASE_SIZE);
    const auto status = LibXR::Topic(topic_handle).PackRaw(
      message.payload, LibXR::RawData(packet.data(), packet.size()), message.timestamp);
    if (status == LibXR::ErrorCode::OK) outgoing_.Submit(kind, packet);
    else diagnostics_.tx_pack_failures.fetch_add(1);
  }

  void HandleAhrs(const LibXR::Topic::RawMessageView & message)
  {
    if (message.payload.size_ != sizeof(QuaternionPayload) || message.payload.addr_ == nullptr) {
      diagnostics_.rx_invalid_payload_failures.fetch_add(1); return;
    }

    QuaternionPayload raw{};
    std::memcpy(&raw, message.payload.addr_, sizeof(raw));
    const auto quaternion = libxr_protocol::DecodeQuaternion(raw);
    const auto now = AhrsTimeline::Clock::now();
    const auto raw_norm = std::sqrt(
      static_cast<long double>(raw.w) * raw.w + static_cast<long double>(raw.x) * raw.x +
      static_cast<long double>(raw.y) * raw.y + static_cast<long double>(raw.z) * raw.z);

    bool warn_norm = false;
    const auto result =
      gate_.OnSample(static_cast<uint64_t>(message.timestamp), quaternion, now);
    if (result == SampleResult::REJECTED) {
      diagnostics_.rx_invalid_payload_failures.fetch_add(1); return;
    }
    {
      std::lock_guard lock(warning_mutex_);
      if (std::abs(raw_norm - 1.0L) > 1e-3L &&
          (last_norm_warning_ == AhrsTimeline::Clock::time_point{} ||
           now - last_norm_warning_ >= std::chrono::seconds(1))) {
        last_norm_warning_ = now;
        warn_norm = true;
      }
    }

    if (warn_norm) {
      tools::logger()->warn("[GimbalRuntime] Normalized AHRS quaternion with norm {:.6f}",
        static_cast<double>(raw_norm));
    }
  }

  void HandleFeedback(const LibXR::Topic::RawMessageView & message)
  {
    if (message.payload.size_ != sizeof(GimbalFeedbackPayload) ||
      message.payload.addr_ == nullptr) {
      diagnostics_.rx_invalid_payload_failures.fetch_add(1);
      return;
    }
    GimbalFeedbackPayload sample{};
    std::memcpy(&sample, message.payload.addr_, sizeof(sample));
    const auto invalid_flags = static_cast<uint8_t>(
      sample.valid_flags & static_cast<uint8_t>(~libxr_protocol::FEEDBACK_VALID_MASK));
    const auto speed_valid =
      (sample.valid_flags & libxr_protocol::FEEDBACK_BULLET_SPEED_VALID) != 0U;
    const auto mode_valid =
      (sample.valid_flags & libxr_protocol::FEEDBACK_GIMBAL_MODE_VALID) != 0U;
    const auto shoot_mode_valid =
      (sample.valid_flags & libxr_protocol::FEEDBACK_SHOOT_MODE_VALID) != 0U;
    const auto reserved_zero =
      sample.reserved[0] == 0U && sample.reserved[1] == 0U && sample.reserved[2] == 0U;
    if (invalid_flags != 0U || !reserved_zero ||
      (speed_valid && (!std::isfinite(sample.bullet_speed) || sample.bullet_speed < 0.0F ||
        sample.bullet_speed > 100.0F)) ||
      (mode_valid && sample.gimbal_mode > 3U) ||
      (shoot_mode_valid && sample.shoot_mode > 2U)) {
      diagnostics_.rx_invalid_payload_failures.fetch_add(1);
      tools::logger()->warn("[GimbalRuntime] Rejected invalid gimbal feedback");
      return;
    }
    std::lock_guard lock(feedback_mutex_);
    feedback_ = sample;
    feedback_received_ = AhrsTimeline::Clock::now();
  }

  void ForceSafeFire()
  {
    FirePayload safe_fire{false};
    std::vector<uint8_t> packet(sizeof(safe_fire) + LibXR::Topic::PACK_BASE_SIZE);
    const auto status = fire_topic_.PackRaw(
      LibXR::ConstRawData(safe_fire), LibXR::RawData(packet.data(), packet.size()));
    if (status == LibXR::ErrorCode::OK) {
      outgoing_.ForceSafeFire(packet);
    } else {
      diagnostics_.tx_pack_failures.fetch_add(1);
    }
  }

  void ReceiveLoop()
  {
    LibXR::ReadOperation wait_op(rx_sem_);
    std::array<uint8_t, 512> buffer{};

    for (;;) {
      if (uart_->read_port_->Size() == 0) {
        auto readiness = LibXR::RawData(buffer.data(), 0);
        if ((*uart_->read_port_)(readiness, wait_op) != LibXR::ErrorCode::OK) {
          diagnostics_.read_failures.fetch_add(1);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
      }

      while (uart_->read_port_->Size() > 0) {
        const auto size = std::min(buffer.size(), uart_->read_port_->Size());
        auto bytes = LibXR::RawData(buffer.data(), size);
        if ((*uart_->read_port_)(bytes, wait_op) != LibXR::ErrorCode::OK) {
          diagnostics_.read_failures.fetch_add(1);
          break;
        }
        diagnostics_.rx_topic_updates.fetch_add(server_.ParseData(LibXR::ConstRawData(buffer.data(), size)));
        diagnostics_.rx_bytes.fetch_add(size);
      }
    }
  }

  void TransmitLoop()
  {
    LibXR::WriteOperation write_op(tx_sem_);
    for (;;) {
      auto packet = outgoing_.BeginNext();
      if (!packet) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      tx_packet_.assign(packet->begin(), packet->end());
      const auto status = (*uart_->write_port_)(
        LibXR::ConstRawData(tx_packet_.data(), tx_packet_.size()), write_op);
      if (status != LibXR::ErrorCode::OK) diagnostics_.write_failures.fetch_add(1);
      outgoing_.Complete(status == LibXR::ErrorCode::OK);
      tx_packet_.clear();
    }
  }

  void StaleLoop()
  {
    auto next_diagnostics = AhrsTimeline::Clock::now() + std::chrono::seconds(5);
    for (;;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      const auto now = AhrsTimeline::Clock::now();
      gate_.MarkStale(now);
      if (config_.transport_diagnostics_enabled && now >= next_diagnostics) {
        tools::logger()->info(
          "[GimbalRuntime] transport diagnostics: rx_topics={}, rx_bytes={}, tx_publish_requests={}, "
          "tx_pack_failures={}, rx_invalid_payloads={}, read_failures={}, write_failures={}",
          diagnostics_.rx_topic_updates.load(), diagnostics_.rx_bytes.load(),
          diagnostics_.tx_publish_requests.load(), diagnostics_.tx_pack_failures.load(),
          diagnostics_.rx_invalid_payload_failures.load(), diagnostics_.read_failures.load(),
          diagnostics_.write_failures.load());
        next_diagnostics = now + std::chrono::seconds(5);
      }
    }
  }

  const RuntimeConfig config_;
  std::unique_ptr<LibXR::LinuxUART> uart_;
  LibXR::Topic target_topic_;
  LibXR::Topic fire_topic_;
  LibXR::Topic ahrs_topic_;
  LibXR::Topic feedback_topic_;
  LibXR::Topic::Server server_;
  LibXR::Topic::Callback target_callback_;
  LibXR::Topic::Callback fire_callback_;
  LibXR::Topic::Callback ahrs_callback_;
  LibXR::Topic::Callback feedback_callback_;
  OutgoingBridge outgoing_;
  GimbalRuntimeGate gate_;
  LibXR::Semaphore rx_sem_;
  LibXR::Semaphore tx_sem_;
  std::mutex warning_mutex_;
  mutable std::mutex feedback_mutex_;
  GimbalFeedbackPayload feedback_{};
  AhrsTimeline::Clock::time_point feedback_received_{};
  static constexpr auto kFeedbackTimeout = std::chrono::milliseconds(250);
  AhrsTimeline::Clock::time_point last_norm_warning_{};
  std::vector<uint8_t> tx_packet_;
  std::thread rx_thread_;
  std::thread tx_thread_;
  std::thread stale_thread_;
  struct { std::atomic<uint64_t> rx_topic_updates{}, rx_bytes{}, tx_publish_requests{}, tx_pack_failures{}, rx_invalid_payload_failures{}, read_failures{}, write_failures{}; } diagnostics_;
};

GimbalRuntime & GimbalRuntime::Instance(const RuntimeConfig & config)
{
  static std::mutex instance_mutex;
  static GimbalRuntime * instance = nullptr;
  std::lock_guard lock(instance_mutex);
  if (instance != nullptr) {
    throw std::logic_error("GimbalRuntime may only be acquired once per process");
  }
  static std::once_flag platform_once;
  std::call_once(platform_once, [] { LibXR::PlatformInit(); });
  instance = new GimbalRuntime(config);
  return *instance;
}

GimbalRuntime::GimbalRuntime(const RuntimeConfig & config) : impl_(std::make_unique<Impl>(config)) {}

GimbalRuntime::~GimbalRuntime() = default;

void GimbalRuntime::SendTarget(const TargetEulerPayload & target) { impl_->SendTarget(target); }

void GimbalRuntime::SendFire(bool fire) { impl_->SendFire(fire); }

void GimbalRuntime::SendPassiveFalseFire() { impl_->SendPassiveFalseFire(); }

void GimbalRuntime::WaitReady() { impl_->WaitReady(); }

Eigen::Quaterniond GimbalRuntime::WaitQuaternion(AhrsTimeline::Clock::time_point requested)
{
  return impl_->WaitQuaternion(requested);
}

RuntimeSnapshot GimbalRuntime::Snapshot() const { return impl_->Snapshot(); }

bool GimbalRuntime::HasFreshAhrs() const { return impl_->HasFreshAhrs(); }
}  // namespace io
