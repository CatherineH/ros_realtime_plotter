cmake_minimum_required(VERSION 2.8.3)
project(ros_realtime_plotter)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package()

install(PROGRAMS
   src/RealTimePlotter.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

