cmake_minimum_required(VERSION 3.8)
project(hitech_bindings)

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

include_directories(include)

add_executable(hitech_bindings src/hitech_bindings.cpp)

ament_target_dependencies(hitech_bindings
  prizm_interfaces
  rclcpp
)

install(TARGETS
  hitech_bindings
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TEST_DIR ${CMAKE_CURRENT_BINARY_DIR})
  set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  set(PKG_NAME ${PROJECT_NAME})

  set(TEST_FILES
    test/bindings_test.cpp
  )
  ament_add_gtest_executable(bindings_test ${TEST_FILES})

  target_include_directories(bindings_test PUBLIC
    $<BUILD_INTERFACE:${SRC_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(bindings_test
    prizm_interfaces
    std_srvs
    rclcpp
  )


  add_ros_test(launch/binding_test.launch.py ARGS
    "TEST_DIR:=${TEST_DIR}"
    "SRC_DIR:=${SRC_DIR}"
    "PKG_NAME:=${PKG_NAME}"
  )

endif()

ament_package()
