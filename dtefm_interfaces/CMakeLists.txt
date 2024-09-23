cmake_minimum_required(VERSION 3.8)
project(dtefm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EAPCommand.msg"
  "msg/EFMCommand.msg"
  "msg/SRStateRobot.msg"
  "msg/SRRobotInfoMsg.msg"
  "msg/SRStatePA.msg"
  "msg/SRValues.msg"
  "msg/PetriNet.msg"
  "msg/PlaceMsg.msg"
  "msg/TransitionMsg.msg"
  "msg/ArcMsg.msg"
  "msg/GCPNStateMsg.msg"
  "srv/EAPCommandAnalysis.srv"
  "srv/EAPCommandState.srv"
  "srv/SRTcpCommunication.srv"
  "srv/SRState.srv"
  "srv/PNCommand.srv"
  "srv/EtherGateSrv.srv"
  "srv/SRRobotIKSrv.srv"
  "srv/SRRobotInfoSrv.srv"
  "srv/GCPNSrv.srv"
  "srv/IntensionControlSrv.srv"
  "action/SRRobotIKMove.action"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
