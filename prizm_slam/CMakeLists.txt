cmake_minimum_required(VERSION 3.8)
project(prizm_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(prizm_interfaces REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(scan_to_pc src/LaserScanToPointCloud.cpp)
ament_target_dependencies(scan_to_pc
  sensor_msgs
  laser_geometry
  rclcpp
)

install(TARGETS
  scan_to_pc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/transformBroadcaster.py
  scripts/parsePointCloud.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
if(BUILD_TESTING)
  # Linters registered as tests from the <test_depend>s in package.xml.
  # Curated subset: copyright / cpplint / uncrustify are intentionally off.
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
