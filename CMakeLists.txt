cmake_minimum_required(VERSION 3.0.2)
project(r1_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
  tf
)

catkin_package()

install(DIRECTORY scripts/ config/ launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
