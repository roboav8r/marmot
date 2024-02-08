cmake_minimum_required(VERSION 3.8)
project(marmot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tracking_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install launch and config files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Add and install CPP targets
add_executable(depthai_preproc src/depthai_preproc.cpp)
ament_target_dependencies(depthai_preproc rclcpp vision_msgs geometry_msgs tracking_msgs tf2 tf2_ros tf2_geometry_msgs)
add_executable(nuscenes_preproc src/nuscenes_preproc.cpp)
ament_target_dependencies(nuscenes_preproc rclcpp vision_msgs geometry_msgs tracking_msgs foxglove_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS 
  depthai_preproc
  nuscenes_preproc
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/tbd_node.py
  scripts/nuscenes_exp_mgr.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_package()
