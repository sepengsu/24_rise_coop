cmake_minimum_required(VERSION 3.0.2)
project(racer_world)

# Compile as C++11
add_compile_options(-std=c++11)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS GAZEBO
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add any executable or libraries here
# Example: add_executable(racer_world_node src/racer_world_node.cpp)
# Example: target_link_libraries(racer_world_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Install any executables, libraries, or other required files
# Example: install(TARGETS racer_world_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Install any directories like launch, models, worlds, etc.
# Example: install(DIRECTORY launch models worlds
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
