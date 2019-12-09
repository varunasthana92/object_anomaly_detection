cmake_minimum_required(VERSION 2.8.3)
project(voadrs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  trajectory_msgs
)

find_package( OpenCV REQUIRED )


catkin_package(
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/voadrs
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



add_executable(cvInspect src/cvInspect.cpp src/moveRobot.cpp src/imageConvert.cpp src/circle.cpp src/dataReader.cpp)


target_link_libraries(cvInspect
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )



#############
## Testing ##
#############


if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
  	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  catkin_add_gtest(level1Tests test/level1.test test/level1/main.cpp test/level1/dataReaderTest.cpp test/level1/circleTest.cpp
		 src/dataReader.cpp src/circle.cpp test/level1/moveRobotTest.cpp src/moveRobot.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(level1Tests
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
endif()
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_voadrs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
