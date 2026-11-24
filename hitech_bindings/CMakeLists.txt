cmake_minimum_required(VERSION 3.8)
project(hitech_bindings)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(prizm_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(hitech_driver
  src/hitech_driver.cpp
  src/hitech_bindings.cpp)

ament_target_dependencies(hitech_driver
  prizm_interfaces
  rclcpp
  std_msgs
)

install(TARGETS
  hitech_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/setPosition.py
  scripts/joyControl.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linters registered as tests from the <test_depend>s in package.xml.
  # Curated subset: copyright / cpplint / uncrustify are intentionally off.
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Vendor header from Hitec, not ours to reformat/check
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/hitech_bindings/hitec-can-servo-reg-public.h"
  )
  # ament_clang_format (Jazzy) ignores AMENT_LINT_AUTO_FILE_EXCLUDE, so
  # disable its auto hook and register it below with explicit paths instead.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_format)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_clang_format QUIET)
  if(ament_cmake_clang_format_FOUND)
    ament_clang_format(
      CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format"
      src
      test
      include/hitech_bindings/hitech_bindings.hpp
    )
  endif()

  set(TEST_FILES
    test/bindings_test.cpp
    src/hitech_bindings.cpp
  )
  ament_add_gtest_executable(bindings_test ${TEST_FILES})

  target_include_directories(bindings_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  # The test loads its node parameters from this file via NodeOptions
  target_compile_definitions(bindings_test PRIVATE
    TEST_PARAMS_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/test_driver.yaml"
  )
  ament_target_dependencies(bindings_test
    prizm_interfaces
    std_srvs
    std_msgs
    rclcpp
  )

  # Run the gtest directly (each TEST_F is reported individually in the
  # JUnit XML). This replaces the launch_testing wrapper, which raced: the
  # gtest finished before the active test attached, so it always failed.
  ament_add_gtest_test(bindings_test)

endif()

ament_package()
