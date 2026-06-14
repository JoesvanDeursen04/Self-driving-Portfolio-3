cmake_minimum_required(VERSION 3.0.2)
project(maze_navigator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  duckietown_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/obstacle_detector.py
  src/maze_navigator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
