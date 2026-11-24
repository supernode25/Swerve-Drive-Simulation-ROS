cmake_minimum_required(VERSION 3.8)
project(midpro_nav2_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util
  nav_msgs geometry_msgs std_msgs pluginlib tf2 tf2_ros tf2_geometry_msgs)

add_library(${PROJECT_NAME} SHARED
  src/midpro_fsm_controller.cpp
  src/midpro_local_path.cpp
  src/midpro_control_loop.cpp
  src/path_conditioner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(midpro_straight_planner SHARED src/midpro_straight_planner.cpp)
ament_target_dependencies(midpro_straight_planner ${dependencies})
target_compile_definitions(midpro_straight_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core midpro_nav2_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME} midpro_straight_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(FILES midpro_nav2_controller_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} midpro_straight_planner)
ament_export_dependencies(${dependencies})
ament_package()
