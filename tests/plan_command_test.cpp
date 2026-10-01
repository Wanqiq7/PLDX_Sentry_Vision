#include "tasks/auto_aim/planner/planner.hpp"

#include <cassert>

int main()
{
  const io::Command legacy{true, false, 1.0, 2.0};
  assert(legacy.yaw_vel == 0);
  assert(legacy.yaw_acc == 0);
  assert(legacy.pitch_vel == 0);
  assert(legacy.pitch_acc == 0);

  const auto command = auto_aim::to_command(auto_aim::Plan{
    true, true, 0.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
  assert(command.control);
  assert(command.shoot);
  assert(command.yaw == 1.0);
  assert(command.yaw_vel == 2.0);
  assert(command.yaw_acc == 3.0);
  assert(command.pitch == 4.0);
  assert(command.pitch_vel == 5.0);
  assert(command.pitch_acc == 6.0);
  assert(command.horizon_distance == 0);
  return 0;
}
