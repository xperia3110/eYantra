cmake_minimum_required(VERSION 3.8)
project(waypoint_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  # Add this line
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add action and service files
set(msg_files
  "action/NavToWaypoint.action"
  "srv/GetWaypoints.srv"
)

# Generate the custom action and service
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavToWaypoint.action"
  "srv/GetWaypoints.srv"
  DEPENDENCIES action_msgs geometry_msgs  # Add geometry_msgs here as well
)


# Install Python scripts
install(PROGRAMS
  src/pico_client.py
  src/pico_server.py
  src/waypoint_service.py
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

ament_package()
