#include "io/gimbal/gimbal_command.hpp"

#include <cassert>

int main()
{
  const auto idle = io::gimbal_detail::DecodeCommandMode(0);
  assert(idle.valid && !idle.control && !idle.fire);

  const auto control = io::gimbal_detail::DecodeCommandMode(1);
  assert(control.valid && control.control && !control.fire);

  const auto fire = io::gimbal_detail::DecodeCommandMode(2);
  assert(fire.valid && fire.control && fire.fire);

  for (unsigned int value = 3; value <= 255; ++value) {
    const auto invalid = io::gimbal_detail::DecodeCommandMode(static_cast<uint8_t>(value));
    assert(!invalid.valid && !invalid.control && !invalid.fire);
  }
  return 0;
}
