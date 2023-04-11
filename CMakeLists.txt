cmake_minimum_required(VERSION 2.8.3)
project(robot_teleop)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
  joy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_teleop
#  CATKIN_DEPENDS roscpp rospy tf turtlesim
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ultrasonic_broadcaster src/ultrasonic_broadcaster.cpp)
target_link_libraries(ultrasonic_broadcaster ${catkin_LIBRARIES})

add_executable(myJoyTeleop src/myJoyTeleop.cpp)
target_link_libraries(myJoyTeleop ${catkin_LIBRARIES})