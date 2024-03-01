cmake_minimum_required(VERSION 3.8)
project(gators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)
install(
  DIRECTORY include/gators
  DESTINATION include
)

# agents
add_library(agents STATIC src/agents.cpp)
ament_target_dependencies(agents
  sensor_msgs
  visualization_msgs
  std_srvs
  pcl_ros
  tf2_ros
)

# board
add_library(board STATIC src/board.cpp)
ament_target_dependencies(board
  sensor_msgs
  visualization_msgs
  std_srvs
  pcl_ros
  tf2_ros
)

# game_manager
add_library(game_manager STATIC src/game_manager.cpp)
ament_target_dependencies(game_manager
  sensor_msgs
  visualization_msgs
  std_srvs
  pcl_ros
  tf2_ros
)
target_link_libraries(game_manager board agents)

# mcts
add_library(mcts STATIC src/mcts.cpp)
ament_target_dependencies(mcts
  sensor_msgs
  visualization_msgs
  std_srvs
  pcl_ros
  tf2_ros
)
target_link_libraries(mcts game_manager board agents)

# visualizer
add_library(game_visualizer STATIC src/game_visualizer.cpp)
ament_target_dependencies(game_visualizer 
  rclcpp 
  sensor_msgs 
  visualization_msgs 
  pcl_ros
)

# executables
# play_game
add_executable(game_player src/game_player.cpp)
ament_target_dependencies(game_player 
  rclcpp
  std_srvs
)
target_link_libraries(game_player 
  agents
  board
  game_visualizer 
  game_manager
  mcts
)

install(
  TARGETS
    game_player
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    models
    rviz
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
