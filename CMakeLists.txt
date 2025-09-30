cmake_minimum_required(VERSION 3.16)
project(oxebots_strategy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(include)

add_library(strategy_component SHARED
  src/strategy_node.cpp
)
target_include_directories(strategy_component PUBLIC include)
ament_target_dependencies(strategy_component
  rclcpp rclcpp_components behaviortree_cpp_v3
)
rclcpp_components_register_nodes(strategy_component "oxebots_strategy::StrategyNode")

add_executable(strategy_node src/strategy_node.cpp)
target_include_directories(strategy_node PUBLIC include)
ament_target_dependencies(strategy_node
  rclcpp behaviortree_cpp_v3
)
target_link_libraries(strategy_node strategy_component)

install(TARGETS strategy_component strategy_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_tree.xml")
  install(FILES test_tree.xml DESTINATION share/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
