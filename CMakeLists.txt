cmake_minimum_required(VERSION 3.0.2)
project(trajectory_visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_service_files(
  FILES
  SaveTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp std_msgs tf2 tf2_ros visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hw_test_node src/hw_test_node.cpp)
target_link_libraries(hw_test_node ${catkin_LIBRARIES})
add_dependencies(hw_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_publisher_saver_node src/trajectory_publisher_saver_node.cpp)
target_link_libraries(trajectory_publisher_saver_node ${catkin_LIBRARIES})
add_dependencies(trajectory_publisher_saver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
