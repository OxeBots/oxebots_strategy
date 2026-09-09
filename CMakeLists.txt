cmake_minimum_required(VERSION 3.8)
project(oxebots_strategy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(oxebots_interfaces REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(
  strategy_node
  src/strategy_node.cpp
  src/go_to_point_node.cpp
  src/kick_ball_node.cpp
  src/align_to_ball_node.cpp
  src/update_ball_position_node.cpp
  src/calculate_interception_node.cpp
  src/is_ball_close_condition.cpp
  src/goalkeeper_node.cpp
  src/precision_kick_node.cpp
  src/defender_nodes.cpp
  src/goal_line_defend_node.cpp
  src/is_ball_in_area_condition.cpp
  src/is_ball_in_our_half_condition.cpp
  src/lateral_clear_node.cpp
  src/position_bisector_node.cpp
  src/smother_save_node.cpp
  src/ai_parse_node.cpp
)

ament_target_dependencies(
  strategy_node
  rclcpp
  oxebots_interfaces
  ssl_league_msgs
  behaviortree_cpp
  ament_index_cpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

target_include_directories(strategy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include
)

target_link_directories(strategy_node PRIVATE /usr/local/lib)

target_link_libraries(strategy_node onnxruntime)

add_executable(
  movement_calculation_node
  src/movement_calculation.cpp
)

ament_target_dependencies(
  movement_calculation_node
  rclcpp
  oxebots_interfaces
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  tf2_ros
)

target_include_directories(movement_calculation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(
  d_star_planner_node
  src/d_star_planner.cpp
)

ament_target_dependencies(
  d_star_planner_node
  rclcpp
  oxebots_interfaces
  nav_msgs
  geometry_msgs
)

target_include_directories(d_star_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(
  role_assigner_node
  src/role_assigner_node.cpp
)

ament_target_dependencies(
  role_assigner_node
  rclcpp
  oxebots_interfaces
)

install(TARGETS
  strategy_node
  movement_calculation_node
  d_star_planner_node
  role_assigner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
)

ament_package()