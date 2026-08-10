# CMake generated Testfile for 
# Source directory: /home/wanqiq/2026Sentry/PLDX_Sentry_Vision
# Build directory: /home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(target_directive_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/target_directive_test")
set_tests_properties(target_directive_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;114;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
add_test(libxr_protocol_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/libxr_protocol_test")
set_tests_properties(libxr_protocol_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;119;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
add_test(ahrs_timeline_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/ahrs_timeline_test")
set_tests_properties(ahrs_timeline_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;124;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
add_test(outgoing_bridge_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/outgoing_bridge_test")
set_tests_properties(outgoing_bridge_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;128;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
add_test(latest_value_publisher_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/latest_value_publisher_test")
set_tests_properties(latest_value_publisher_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;132;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
add_test(gimbal_runtime_gate_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/gimbal_runtime_gate_test")
set_tests_properties(gimbal_runtime_gate_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;141;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
add_test(gimbal_config_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/gimbal_config_test")
set_tests_properties(gimbal_config_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;146;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
add_test(gimbal_command_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/gimbal_command_test")
set_tests_properties(gimbal_command_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;150;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
add_test(libxr_pty_test "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/build-jazzy/libxr_pty_test")
set_tests_properties(libxr_pty_test PROPERTIES  _BACKTRACE_TRIPLES "/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;155;add_test;/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt;0;")
subdirs("tools")
subdirs("third_party/libxr")
subdirs("io")
subdirs("tasks/auto_aim")
subdirs("tasks/auto_buff")
subdirs("tasks/omniperception")
