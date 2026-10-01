#pragma once

#include <string>
#include <vector>

namespace io
{
// Shared business enums used by the vision pipeline.
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};

inline const std::vector<std::string> MODES = {
  "idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// Kept for source compatibility with older vision modules.  Revision 2 does
// not carry a launcher shoot-mode field on the gimbal feedback topic.
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};

inline const std::vector<std::string> SHOOT_MODES = {
  "left_shoot", "right_shoot", "both_shoot"};

}  // namespace io
