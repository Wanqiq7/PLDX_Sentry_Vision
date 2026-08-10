# Target directive freshness

> 当前验证环境：Ubuntu 24.04、ROS 2 Jazzy、GCC 13、C++20。

Production sentry executables require `pldx_vision_interfaces`. `sp_msgs`
remains optional and only enables the legacy compatibility subscriptions.

`target_directive_timeout_ms` defaults to 750 ms.
The upstream fuser heartbeat period must be comfortably less than this consumer
timeout. The production fuser publishes valid state at 20 Hz (a 50 ms period),
while navigation and referee sources remain fresh for 500 ms and 1000 ms,
respectively.
