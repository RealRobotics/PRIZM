cmake_minimum_required(VERSION 3.8)
project(prizm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
    "msg/DynamixelPos.msg"
    "msg/DynamixelVel.msg"
    "msg/DynamixelPosArr.msg"
    "msg/DynamixelVelArr.msg"
)

set(srv_files
	"srv/DynamixelGetPos.srv"
	"srv/DynamixelGetVel.srv"
    "srv/DynamixelSetProfVel.srv"
    "srv/DynamixelGetStatus.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_package()
