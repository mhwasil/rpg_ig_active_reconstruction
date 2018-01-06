cmake_minimum_required(VERSION 2.8.3)
project(ig_active_reconstruction_youbot)
set(CMAKE_BUILD_TYPE Release)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  urdf
  ig_active_reconstruction
  ig_active_reconstruction_msgs
  ig_active_reconstruction_ros
  ig_active_reconstruction_octomap
  movements
  tf
  gazebo_msgs
  geometry_msgs
  sensor_msgs
  moveit_ros_planning_interface
  message_generation
  mir_yb_action_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    urdf
    ig_active_reconstruction
    ig_active_reconstruction_msgs
    ig_active_reconstruction_octomap
    movements
    geometry_msgs
    sensor_msgs
    mir_yb_action_msgs
  DEPENDS
    Eigen
    Boost
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/codes/*.cpp"
)

add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
 ${catkin_EXPORTED_TARGETS}
)


# Add service file ....

add_service_files(
  FILES
  ybMoveArmTo.srv
  yb_move_arm_using_joints.srv
)

# Executables...........................................................

add_executable(yb_ros_server_node
  src/nodes/yb_ros_server_node.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(yb_ros_server_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(yb_ros_server_node
 ${catkin_EXPORTED_TARGETS}
)


add_executable(yb_joints_to_pose_converter
  src/nodes/yb_joints_to_pose_converter.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(yb_joints_to_pose_converter
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(yb_joints_to_pose_converter
 ${catkin_EXPORTED_TARGETS}
)

add_executable(yb_move_base_safe
  src/nodes/yb_move_base_safe.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(yb_move_base_safe
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(yb_move_base_safe
 ${catkin_EXPORTED_TARGETS}
)

add_executable(yb_trigger_robocup_topics
  src/nodes/yb_trigger_robocup_topics.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(yb_trigger_robocup_topics
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(yb_trigger_robocup_topics
 ${catkin_EXPORTED_TARGETS}
)

add_executable(yb_trigger_cloud_e_start_publish
  src/nodes/yb_trigger_cloud_e_start_publish.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(yb_trigger_cloud_e_start_publish
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(yb_trigger_cloud_e_start_publish
 ${catkin_EXPORTED_TARGETS}
)

add_executable(yb_trigger_cloud_e_add_cloud_start
  src/nodes/yb_trigger_cloud_e_add_cloud_start.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(yb_trigger_cloud_e_add_cloud_start
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(yb_trigger_cloud_e_add_cloud_start
 ${catkin_EXPORTED_TARGETS}
)

