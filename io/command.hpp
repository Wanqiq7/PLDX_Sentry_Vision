#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control = false;
  bool shoot = false;
  double yaw = 0;
  double pitch = 0;
  double horizon_distance = 0;  //无人机专有
  double yaw_vel = 0;
  double yaw_acc = 0;
  double pitch_vel = 0;
  double pitch_acc = 0;
};

}  // namespace io

#endif  // IO__COMMAND_HPP
