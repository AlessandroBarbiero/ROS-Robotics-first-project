cmake_minimum_required(VERSION 3.0.2)

project(project_1)
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	geometry_msgs 
	nav_msgs
	message_generation
	message_filters
	)

add_message_files(
	FILES 
	CustomOdometry.msg
	)
#add this when implementing services
# add_service_files(FILES custom_service.srv)

generate_messages(
	DEPENDENCIES 
	nav_msgs
	std_msgs
	)

catkin_package(
	CATKIN_DEPENDS 
	message_runtime
	message_filters
	geometry_msgs
	)
#never change this line
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(AgileXCore src/AgileXCore.cpp)
target_link_libraries(AgileXCore ${catkin_LIBRARIES})

# This two lines are needed for every node
add_executable(lateralWheelSync src/LateralWheelSync.cpp)
target_link_libraries(lateralWheelSync ${catkin_LIBRARIES})

# Add this only for nodes that requires the specific custom message generated in the package
add_dependencies(lateralWheelSync robotics_hw1_generate_messages_cpp)
