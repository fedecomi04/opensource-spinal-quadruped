cmake_minimum_required(VERSION 3.8)
project(spine_roll_inclined_plane)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yesense_interface REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(spine_roll_inclined_plane_node src/spine_roll_inclined_plane_node.cpp)
target_include_directories(spine_roll_inclined_plane_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(spine_roll_inclined_plane_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(spine_roll_inclined_plane_node rclcpp yesense_interface std_msgs)

install(TARGETS spine_roll_inclined_plane_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
