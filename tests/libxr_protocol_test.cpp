#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "crc.hpp"
#include "io/gimbal/libxr_protocol.hpp"
#include "libxr.hpp"

namespace
{
using io::libxr_protocol::QuaternionPayload;
using io::libxr_protocol::GimbalFeedbackPayload;

void TestDtoLayoutAndMapping()
{
  static_assert(std::string_view(io::libxr_protocol::TARGET_EULER_TOPIC) == "target_euler");
  static_assert(std::string_view(io::libxr_protocol::FIRE_NOTIFY_TOPIC) == "fire_notify");
  static_assert(std::string_view(io::libxr_protocol::AHRS_QUATERNION_TOPIC) == "ahrs_quaternion");
  static_assert(
    std::string_view(io::libxr_protocol::NAV_GIMBAL_FEEDBACK_TOPIC) == "nav_gimbal_feedback_v1");
  static_assert(sizeof(io::libxr_protocol::TargetEulerPayload) == 36);
  static_assert(sizeof(io::libxr_protocol::FirePayload) == 1);
  static_assert(sizeof(QuaternionPayload) == 16);
  static_assert(sizeof(GimbalFeedbackPayload) == 12);
  static_assert(alignof(GimbalFeedbackPayload) == 1);
  static_assert(offsetof(GimbalFeedbackPayload, bullet_speed) == 0);
  static_assert(offsetof(GimbalFeedbackPayload, bullet_count) == 4);
  static_assert(offsetof(GimbalFeedbackPayload, gimbal_mode) == 6);
  static_assert(offsetof(GimbalFeedbackPayload, shoot_mode) == 7);
  static_assert(offsetof(GimbalFeedbackPayload, valid_flags) == 8);
  static_assert(offsetof(GimbalFeedbackPayload, reserved) == 9);

  GimbalFeedbackPayload feedback{
    18.5F, 42, 2, 1,
    static_cast<uint8_t>(io::libxr_protocol::FEEDBACK_VALID_MASK), {0, 0, 0}};
  assert(feedback.bullet_speed == 18.5F);
  assert(feedback.bullet_count == 42);
  assert(feedback.gimbal_mode == 2);
  assert(feedback.shoot_mode == 1);
  assert(feedback.valid_flags == 0x78);

  const auto target = io::libxr_protocol::EncodeTarget(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  assert(target.rol == 0.0F);
  assert(target.pit == 4.0F);
  assert(target.yaw == 1.0F);
  assert(target.rol_dot == 0.0F);
  assert(target.pit_dot == 5.0F);
  assert(target.yaw_dot == 2.0F);
  assert(target.rol_ddot == 0.0F);
  assert(target.pit_ddot == 6.0F);
  assert(target.yaw_ddot == 3.0F);

  assert(io::libxr_protocol::FirePayload{true}.isfire);

  const QuaternionPayload raw{2.0F, 3.0F, 4.0F, 1.0F};
  const auto q = io::libxr_protocol::DecodeQuaternion(raw);
  assert(q.w() == 1.0 && q.x() == 2.0 && q.y() == 3.0 && q.z() == 4.0);
}

void TestQuaternionPacketContract()
{
  constexpr const char * topic_name = io::libxr_protocol::AHRS_QUATERNION_TOPIC;
  constexpr uint64_t timestamp_us = 0x010203040506ULL;
  const QuaternionPayload payload{2.0F, 3.0F, 4.0F, 1.0F};
  auto topic = LibXR::Topic::CreateTopic<QuaternionPayload>(topic_name);
  LibXR::Topic::PackedData<QuaternionPayload> packet;

  assert(topic.PackData(payload, packet, LibXR::MicrosecondTimestamp(timestamp_us)) ==
         LibXR::ErrorCode::OK);
  static_assert(LibXR::Topic::PACK_BASE_SIZE == 17);
  static_assert(sizeof(packet) == sizeof(QuaternionPayload) + LibXR::Topic::PACK_BASE_SIZE);
  assert(packet.raw.header_.prefix == 0x5A);
  assert(packet.raw.header_.version == 1);
  assert(packet.raw.header_.GetDataLen() == sizeof(QuaternionPayload));
  assert(packet.raw.header_.topic_name_crc32 ==
         LibXR::CRC32::Calculate(topic_name, std::strlen(topic_name)));
  assert(static_cast<uint64_t>(packet.raw.header_.GetTimestamp()) == timestamp_us);
  assert(LibXR::CRC8::Verify(&packet.raw.header_, sizeof(packet.raw.header_)));
  assert(LibXR::CRC8::Verify(&packet, sizeof(packet)));
  assert(std::memcmp(packet.raw.data_, &payload, sizeof(payload)) == 0);
}

void TestServerAcceptsValidShortPayload()
{
  constexpr size_t short_payload_size = sizeof(float);
  auto topic = LibXR::Topic::CreateTopic<QuaternionPayload>("short_ahrs_quaternion");
  static bool callback_invoked = false;
  callback_invoked = false;
  auto callback = LibXR::Topic::Callback::Create(
    [](bool, void *, QuaternionPayload &) { callback_invoked = true; },
    reinterpret_cast<void *>(0));
  topic.RegisterCallback(callback);

  LibXR::Topic::PackedData<QuaternionPayload> packet;
  assert(topic.PackData(
           QuaternionPayload{2.0F, 3.0F, 4.0F, 1.0F}, packet,
           LibXR::MicrosecondTimestamp(123456)) == LibXR::ErrorCode::OK);

  auto * packet_bytes = reinterpret_cast<uint8_t *>(&packet);
  packet.raw.header_.SetDataLen(short_payload_size);
  packet.raw.header_.pack_header_crc8 = LibXR::CRC8::Calculate(
    &packet.raw.header_, sizeof(packet.raw.header_) - sizeof(uint8_t));
  const size_t packet_crc_offset = sizeof(packet.raw.header_) + short_payload_size;
  packet_bytes[packet_crc_offset] = LibXR::CRC8::Calculate(packet_bytes, packet_crc_offset);

  LibXR::Topic::Server server(512);
  server.Register(topic);
  assert(server.ParseData(LibXR::ConstRawData(
           packet_bytes, LibXR::Topic::PACK_BASE_SIZE + short_payload_size)) == 1);
  assert(callback_invoked);
}
}  // namespace

int main()
{
  TestDtoLayoutAndMapping();
  TestQuaternionPacketContract();
  TestServerAcceptsValidShortPayload();
  return 0;
}
