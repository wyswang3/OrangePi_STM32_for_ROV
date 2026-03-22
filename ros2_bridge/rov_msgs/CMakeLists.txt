cmake_minimum_required(VERSION 3.8)
project(rov_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CommandResult.msg"
  "msg/ControlIntentState.msg"
  "msg/ControlState.msg"
  "msg/EventRecord.msg"
  "msg/HealthMonitorStatus.msg"
  "msg/HealthSummary.msg"
  "msg/MotorTestState.msg"
  "msg/NavState.msg"
  "msg/NavStateView.msg"
  "msg/SystemState.msg"
  "msg/TelemetryFrameV2.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
