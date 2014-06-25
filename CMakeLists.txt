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
  trajectory_msgs
  baxter_core_msgs
  skeletonmsgs_nu
  
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

  tf
  dynamic_reconfigure
  cv_bridge
	image_transport
)

catkin_python_setup()

# generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs control_msgs trajectory_msgs baxter_core_msgs skeletonmsgs_nu)

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
  trajectory_msgs
  baxter_core_msgs
  skeletonmsgs_nu
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

  tf
  dynamic_reconfigure
  cv_bridge
	image_transport
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

add_library(ar_blocks
	src/ARBlock.cpp
	src/ARWorldBuilder.cpp
)
target_link_libraries(ar_blocks ${catkin_LIBRARIES})
add_dependencies(ar_blocks moveit_msgs_gencpp ar_blocks_msgs_generate_message_cpp)

add_executable(webcam_video_publisher nodes/webcam_video_publisher.cpp)
target_link_libraries(webcam_video_publisher  
	${catkin_LIBRARIES}

#	opencv_core
#	opencv_imgproc
#	opencv_highgui
#	opencv_ml
#	opencv_video
#	opencv_features2d
#	opencv_calib3d
#	opencv_objdetect
#	opencv_contrib
#	opencv_legacy
#	opencv_stitching

	pthread
 
) 
add_dependencies(webcam_video_publisher moveit_msgs_gencpp ar_blocks_msgs_generate_message_cpp)

add_executable(moveit_ar_environment nodes/moveit_ar_environment.cpp)
target_link_libraries(moveit_ar_environment
	ar_blocks
	${catkin_LIBRARIES}
	
	# opencv_core
	# opencv_imgproc
	# opencv_highgui
	# opencv_ml
	# opencv_video
	# opencv_features2d
	# opencv_calib3d
	# opencv_objdetect
	# opencv_contrib
	# opencv_legacy
	# opencv_stitching

	pthread

)
add_dependencies(moveit_ar_environment moveit_msgs_gencpp ar_blocks_msgs_generate_message_cpp)
