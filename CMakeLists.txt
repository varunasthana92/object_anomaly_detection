cmake_minimum_required(VERSION 2.8.3)
project(object_anomaly_detection)

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

include_directories(
  include/object_anomaly_detection
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cvInspect src/cvInspect.cpp src/moveRobot.cpp src/imageConvert.cpp src/circle.cpp src/dataReader.cpp)

add_executable(cvTest test/level2/test.cpp src/imageConvert.cpp)

target_link_libraries(cvInspect
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

target_link_libraries(cvTest
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(level1Tests test/level1Tests.test test/level1/main.cpp test/level1/dataReaderTest.cpp test/level1/circleTest.cpp
		 src/dataReader.cpp src/circle.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(level1Tests
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
  find_package(rostest REQUIRED)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(level2Tests test/level2Tests.test test/level2/main.cpp test/level2/imageConvertTest.cpp
		 src/imageConvert.cpp )
  target_link_libraries(level2Tests
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
endif()

