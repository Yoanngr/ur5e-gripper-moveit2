cmake_minimum_required(VERSION 3.8)
project(ur5e_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(scene_setup src/scene_setup.cpp)
target_include_directories(scene_setup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(scene_setup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  scene_setup
  "rclcpp"
  "geometry_msgs"
  "shape_msgs"
  "moveit_msgs"
  "moveit_ros_planning_interface"
)

install(TARGETS scene_setup
  DESTINATION lib/${PROJECT_NAME})

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

add_executable(pick_place src/pick_place.cpp)
ament_target_dependencies(pick_place rclcpp geometry_msgs moveit_msgs moveit_ros_planning_interface)
install(TARGETS pick_place DESTINATION lib/${PROJECT_NAME})

ament_package()
