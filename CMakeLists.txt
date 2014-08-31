cmake_minimum_required(VERSION 2.8.3)
project(ar_blocks)

find_package(catkin
  REQUIRED
  rospy 
  roscpp 
  roslib
  actionlib
	
	OpenCV

  message_generation
  sensor_msgs
  std_msgs
  geometry_msgs
	shape_msgs
  control_msgs
  baxter_core_msgs
  baxter_interface

	moveit_commander
	moveit_core
	# moveit_full
	moveit_msgs
	# moveit_planners
	moveit_planners_ompl
	# moveit_plugins
	# moveit_ros
	moveit_ros_benchmarks
	moveit_ros_benchmarks_gui
	moveit_ros_manipulation
	moveit_ros_move_group
	moveit_ros_perception
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_robot_interaction
	moveit_ros_visualization
	moveit_ros_warehouse
	moveit_setup_assistant
	moveit_simple_controller_manager

	ar_track_alvar
	moveit_simple_grasps
	baxter_pick_place

  tf
  dynamic_reconfigure
  cv_bridge
	image_transport
  
  actionlib
  actionlib_msgs
)
find_package(Boost REQUIRED)
find_package(Qt4 REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  Block.msg
  Layer.msg
  Structure.msg
)

add_action_files(
  DIRECTORY action FILES
  BuildStructure.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include
  CATKIN_DEPENDS
  rospy
  roscpp
  roslib
  actionlib

  sensor_msgs
  std_msgs
  geometry_msgs
	shape_msgs
  control_msgs
  baxter_core_msgs
  message_runtime
  baxter_interface

	moveit_commander
	moveit_core
	# moveit_full
	moveit_msgs
	# moveit_planners
	moveit_planners_ompl
	# moveit_plugins
	# moveit_ros
	moveit_ros_benchmarks
	moveit_ros_benchmarks_gui
	moveit_ros_manipulation
	moveit_ros_move_group
	moveit_ros_perception
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_robot_interaction
	moveit_ros_visualization
	moveit_ros_warehouse
	moveit_setup_assistant
	moveit_simple_controller_manager

	ar_track_alvar
	moveit_simple_grasps

  tf
  dynamic_reconfigure
  cv_bridge
	image_transport
)

set(ar_blocks_interface_HEADERS
  include/ar_blocks/ARBlocksInterface.h
  include/ar_blocks/Rectangle.h
  include/ar_blocks/Scene.h
)

QT4_WRAP_CPP(ar_blocks_interface_HEADERS_MOC ${ar_blocks_interface_HEADERS})

include(${QT_USE_FILE})

include_directories(
	include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
  ${QT_INCLUDES}
)

link_directories(
  ${QT_LINK_DIRS}
)

add_definitions(${QT_DEFINITIONS})

add_library(ar_blocks
	src/ARBlock.cpp
	src/ARWorldBuilder.cpp
)
target_link_libraries(ar_blocks 
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
  ${QT_LIBRARIES}
)
add_dependencies(ar_blocks ${catkin_EXPORTED_TARGETS} moveit_msgs_gencpp ar_blocks_msgs_generate_message_cpp)

add_executable(webcam_video_publisher nodes/webcam_video_publisher.cpp)
target_link_libraries(webcam_video_publisher  
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
) 
add_dependencies(webcam_video_publisher ${catkin_EXPORTED_TARGETS} ar_blocks_msgs_generate_message_cpp)

add_executable(ar_blocks_environment nodes/ar_blocks_environment.cpp)
target_link_libraries(ar_blocks_environment
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	pthread
	ar_blocks
)
add_dependencies(ar_blocks_environment ${catkin_EXPORTED_TARGETS} moveit_msgs_gencpp ar_blocks_msgs_generate_message_cpp)

add_executable(ar_blocks_server_test nodes/ar_blocks_server_test.cpp src/ARBlocksServerTest.cpp)
target_link_libraries(ar_blocks_server_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
  ar_blocks
)
add_dependencies(ar_blocks_server_test ${catkin_EXPORTED_TARGETS} moveit_msgs_gencpp ar_blocks_msgs_generate_message_cpp)

add_executable(ar_blocks_interface nodes/ar_blocks_interface.cpp 
  src/ARBlocksInterface.cpp
  src/Rectangle.cpp
  src/Scene.cpp
  ${ar_blocks_interface_HEADERS_MOC}
)
target_link_libraries(ar_blocks_interface
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
  ${QT_LIBRARIES}
)
