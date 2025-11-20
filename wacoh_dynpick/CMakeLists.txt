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
find_package(nlohmann_json REQUIRED)   # ★ 新增 JSON 套件

# -------------------------------
# 1. 原本的 dynpick_driver_node
# -------------------------------
add_executable(dynpick_driver_node src/dynpick_driver_node.cpp)
ament_target_dependencies(dynpick_driver_node 
  rclcpp 
  geometry_msgs 
  std_srvs
)

install(TARGETS dynpick_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# 2. 新增 force_listener_node
# -------------------------------
add_executable(force_listener_node src/force_listener.cpp)

ament_target_dependencies(force_listener_node
  rclcpp
  geometry_msgs
  nlohmann_json        # ★ 加入 JSON 套件
)

# 如果需要顯式 link，也可加上（可選）
target_link_libraries(force_listener_node
  nlohmann_json::nlohmann_json
)

install(TARGETS force_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# 3. Install launch files
# -------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py"
  PATTERN ".gitkeep" EXCLUDE
)

# -------------------------------
# 4. Export dependencies
# -------------------------------
ament_export_dependencies(rclcpp geometry_msgs std_srvs nlohmann_json)

# -------------------------------
# 5. Testing (unchanged)
# -------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
