cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs   REQUIRED)

# ── C++ node ──────────────────────────────────────────────────────────────────
add_executable(robot_controller_node src/robot_controller_node.cpp)

target_include_directories(robot_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(robot_controller_node PUBLIC cxx_std_17)

ament_target_dependencies(robot_controller_node
  rclcpp
  geometry_msgs
  sensor_msgs)

# ── install ───────────────────────────────────────────────────────────────────
install(TARGETS robot_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/request_streams.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
