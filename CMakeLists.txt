cmake_minimum_required(VERSION 2.8.3)
project(ros_realtime_plotter)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  rospy
  nav_msgs
  geometry_msgs
)
catkin_package()

install(PROGRAMS
   scripts/real_time_plotter.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

