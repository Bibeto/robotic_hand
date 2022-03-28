cmake_minimum_required(VERSION 3.0.2)

project(robotic_hand)

find_package(roslaunch)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp 
  rospy 
  std_msgs
  gazebo_ros
  urdf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs urdf
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(robotic_hand_move_joint src/robotic_hand_move_joint.cpp)
target_link_libraries(robotic_hand_move_joint ${catkin_LIBRARIES})

add_executable(robotic_hand_move_world_coordinate src/robotic_hand_move_world_coordinate.cpp)
target_link_libraries(robotic_hand_move_world_coordinate ${catkin_LIBRARIES})


file(GLOB python_scripts_DIR
    "scripts/*.py"
)

catkin_install_python(PROGRAMS ${python_scripts_DIR}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)