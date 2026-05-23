cmake_minimum_required(VERSION 3.8)
project(mujoco_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

add_executable(mujoco_node
  src/mujoco_node.cpp
  src/sensor_ids.cpp        )
target_include_directories(mujoco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${GLFW_INCLUDE_DIRS}
  )
target_compile_features(mujoco_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mujoco_node
  "rclcpp"
  "std_msgs"
  "ament_index_cpp"
)

target_link_libraries(mujoco_node
  mujoco
  ${GLFW_LIBRARIES}
  ${CMAKE_DL_LIBS})

install(TARGETS mujoco_node
  DESTINATION lib/${PROJECT_NAME}
  )
install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

