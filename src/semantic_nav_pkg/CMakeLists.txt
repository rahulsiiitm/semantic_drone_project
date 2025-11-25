cmake_minimum_required(VERSION 3.8)
project(semantic_nav_pkg)

# 1. Find Dependencies
# We need standard C++ and the Nav2 Costmap libraries
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# 2. Define the Library (Your Plugin)
add_library(semantic_layer_lib src/semantic_layer.cpp)

# 3. Link Dependencies
# This tells the compiler: "This code needs Nav2 functions to work"
target_include_directories(semantic_layer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(semantic_layer_lib
  nav2_costmap_2d::nav2_costmap_2d
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  pluginlib::pluginlib)

# 4. Register the Plugin
# This allows ROS 2 to "discover" your plugin at runtime
pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_plugins.xml)

# 5. Install Rules (Where files go on Linux)
install(TARGETS semantic_layer_lib
  EXPORT export_semantic_layer_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/)

install(FILES nav2_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_libraries(semantic_layer_lib)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav2_costmap_2d)
ament_package()