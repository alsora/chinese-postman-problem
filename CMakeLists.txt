cmake_minimum_required(VERSION 3.0.0)
project(cpp_solver)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( OpenCV )
if (OpenCV_FOUND)
  add_definitions ( -DOPENCV_LIBRARY_FOUND=12 )
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()


include_directories(src/cpp_solver)


add_subdirectory(src/cpp_solver)
add_subdirectory(src/test)


if (ROS_CATKIN_MAKE)

	add_subdirectory(src/ros_exploration_plugin)

	## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  tf
	  pluginlib
	  geometry_msgs
	  nav_msgs
	  visualization_msgs
	  nav2d_navigator
	)

	###################################
	## catkin specific configuration ##
	###################################

	catkin_package(
	  LIBRARIES ChinesePostmanProblemPlugin
	  CATKIN_DEPENDS
	    	roscpp
		tf
		pluginlib
		geometry_msgs
		nav_msgs
		visualization_msgs
		nav2d_navigator
	)

endif()

