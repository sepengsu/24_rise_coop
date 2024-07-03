cmake_minimum_required(VERSION 3.5)
project(rtabmap_my)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/tf_broadcaster.py
  scripts/view_frames_fixed.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
