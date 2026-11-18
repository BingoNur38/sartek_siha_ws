cmake_minimum_required(VERSION 3.8)
project(sartek_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TelemetryStatus.msg"
  "msg/SystemHealth.msg"
  "msg/QrMissionStatus.msg"
  "msg/DogfightStatus.msg"
  "msg/TargetInfo.msg"
  "msg/MissionCommand.msg"
  "msg/SafetyOverride.msg"
  "msg/NfzInfo.msg"
  "msg/GcsCommand.msg"
  "action/ExecuteMission.action"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
