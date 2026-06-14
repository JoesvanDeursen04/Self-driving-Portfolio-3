cmake_minimum_required(VERSION 3.0.2)
project(maze_localizer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  duckietown_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/odometry_localizer.py
  src/apriltag_localizer.py
  src/maze_localizer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
