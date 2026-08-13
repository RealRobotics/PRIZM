cmake_minimum_required(VERSION 3.8)
project(prizm_dynamixel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(prizm_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
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

# add_executable(single_motor src/single_motor.cpp)
# ament_target_dependencies(single_motor
#   dynamixel_sdk
#   prizm_interfaces
#   rclcpp
# )

add_executable(driver
  src/driver.cpp
  src/dynamixelDriver.cpp
)
ament_target_dependencies(driver
  dynamixel_sdk
  std_srvs
  prizm_interfaces
  rclcpp
)

install(TARGETS
  single_servo
  # single_motor
  driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/publishPos.py
  scripts/publishVel.py
  scripts/getStatus.py
  scripts/calibrateServo.py
  DESTINATION lib/${PROJECT_NAME}
)


# Tell downstream packages where to find our headers
ament_export_include_directories(include)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  # set(PROJECT_TEST ${PROJECT_NAME}_test)


  # ament_add_gtest(${PROJECT_TEST}
  #   test/servo_driver_test.cpp
  #   src/dynamixelDriver.cpp
  # )


  # ament_target_dependencies(${PROJECT_TEST}
  #   rclcpp
  #   prizm_interfaces
  #   dynamixel_sdk
  # )

  set(TEST_DIR ${CMAKE_CURRENT_BINARY_DIR})
  set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  set(PKG_NAME ${PROJECT_NAME})

  # test/servo_driver_test.cpp
  set(TEST_FILES
    test/servo_driver_test.cpp
    src/dynamixelDriver.cpp
  )
  ament_add_gtest_executable(servo_driver_test ${TEST_FILES})

  target_include_directories(servo_driver_test PUBLIC
    $<BUILD_INTERFACE:${SRC_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(servo_driver_test
    dynamixel_sdk
    prizm_interfaces
    std_srvs
    rclcpp
  )


  add_ros_test(launch/driver_tests.py ARGS
    "TEST_DIR:=${TEST_DIR}"
    "SRC_DIR:=${SRC_DIR}"
    "PKG_NAME:=${PKG_NAME}"
  )

endif()


ament_package()
