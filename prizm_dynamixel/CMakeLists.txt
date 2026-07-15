cmake_minimum_required(VERSION 3.8)
project(prizm_dynamixel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(prizm_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(single_servo src/single_servo.cpp)
ament_target_dependencies(single_servo
  dynamixel_sdk
  prizm_interfaces
  rclcpp
)

add_executable(single_motor src/single_motor.cpp)
ament_target_dependencies(single_motor
  dynamixel_sdk
  prizm_interfaces
  rclcpp
)

install(TARGETS
  single_servo
  single_motor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Tell downstream packages where to find our headers
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  set(PROJECT_TEST ${PROJECT_NAME}_test)


  ament_add_gtest(${PROJECT_TEST}
    test/servo_driver_test.cpp
    src/dynamixelDriver.cpp
  )
  
  target_include_directories(${PROJECT_TEST} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_TEST}
    rclcpp
    prizm_interfaces
    dynamixel_sdk
  )


endif()


ament_package()