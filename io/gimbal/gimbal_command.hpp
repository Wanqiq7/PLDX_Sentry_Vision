#pragma once

#include <cstdint>

namespace io::gimbal_detail
{
struct CommandMode
{
  bool valid;
  bool control;
  bool fire;
};

constexpr CommandMode DecodeCommandMode(uint8_t mode)
{
  switch (mode) {
    case 0: return {true, false, false};
    case 1: return {true, true, false};
    case 2: return {true, true, true};
    default: return {false, false, false};
  }
}
}  // namespace io::gimbal_detail
