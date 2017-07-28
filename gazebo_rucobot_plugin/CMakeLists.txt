cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS

  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(rucobot_plugin SHARED rucobot_plugin.cc)
target_link_libraries(rucobot_plugin ${GAZEBO_libraries})

# Build the stand-alone test program
add_executable(send_task_to_plugin send_task_to_plugin.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(send_task_to_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(send_task_to_plugin ${GAZEBO_LIBRARIES})
endif()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_INCLUDE_DIRS})
