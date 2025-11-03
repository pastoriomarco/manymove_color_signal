cmake_minimum_required(VERSION 3.8)
project(manymove_color_signal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(manymove_cpp_trees REQUIRED)
find_package(signal_column_msgs REQUIRED)

add_executable(color_signal_demo src/color_signal_demo.cpp)
ament_target_dependencies(color_signal_demo rclcpp signal_column_msgs)

install(TARGETS color_signal_demo
  DESTINATION lib/${PROJECT_NAME})

add_executable(bt_client_ur_color_signal
  src/bt_client_ur_color_signal.cpp
  src/bt_nodes.cpp)
target_include_directories(bt_client_ur_color_signal PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bt_client_ur_color_signal manymove_cpp_trees::manymove_cpp_trees_lib)
ament_target_dependencies(bt_client_ur_color_signal
  rclcpp rclcpp_action behaviortree_cpp_v3 manymove_cpp_trees signal_column_msgs)

install(TARGETS bt_client_ur_color_signal
  DESTINATION lib/${PROJECT_NAME})

# Install launch files so `ros2 launch` can find them
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
