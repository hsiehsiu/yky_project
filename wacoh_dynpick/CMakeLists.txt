cmake_minimum_required(VERSION 3.8)
project(wacoh_dynpick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Define the C++ executable target
# The name MUST match the executable name in the launch file (dynpick_driver_node)
add_executable(dynpick_driver_node src/dynpick_driver_node.cpp)

# Link the required libraries (rclcpp, messages, services)
ament_target_dependencies(dynpick_driver_node rclcpp geometry_msgs std_srvs)

# Install the executable to the lib directory
install(TARGETS dynpick_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
# --- 2. Install the Launch Files ---

# Install the entire 'launch' directory so ros2 launch can find the Python file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  # Ensures only the .py file is copied, ignoring hidden files
  PATTERN "*.py"
  PATTERN ".gitkeep" EXCLUDE
)

# --- 3. Export Package Information ---

# Export the build dependencies for other packages that might depend on this one
ament_export_dependencies(rclcpp geometry_msgs std_srvs)

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
