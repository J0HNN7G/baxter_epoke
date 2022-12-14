cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find ROS
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build Cube plugin
if(${gazebo_VERSION_MAJOR} EQUAL 7)
  add_library(cube_plugin SHARED cube_plugin_gz7.cpp)
elseif(${gazebo_VERSION_MAJOR} EQUAL 11)
  add_library(cube_plugin SHARED cube_plugin_gz11.cpp)
endif()
target_link_libraries(cube_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
