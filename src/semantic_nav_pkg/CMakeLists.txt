cmake_minimum_required(VERSION 3.8)
project(semantic_nav_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_common REQUIRED)

# 2. Create the library
add_library(semantic_layer_lib src/semantic_layer.cpp)

# 3. Link libraries (THE FIX: Using ament_target_dependencies)
# This handles all the complex linking automatically
ament_target_dependencies(semantic_layer_lib
  rclcpp
  nav2_costmap_2d
  sensor_msgs
  pluginlib
  nav2_common
)

# 4. Include directories
target_include_directories(semantic_layer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 5. Export the plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_plugins.xml)

# 6. Install rules
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