cmake_minimum_required(VERSION 3.0.2)
project(canbus_record)

set(CMAKE_CXX_STANDARD 14)
# add_definitions("-Wall -g")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  can_msgs
)

add_message_files(
  FILES
  # Message2.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs  
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


link_directories(lib)
add_executable(canbus_record src/canbus_record.cpp)
target_link_libraries(canbus_record ${catkin_LIBRARIES} iTekon-usb)
add_dependencies(canbus_record ${catkin_EXPORTED_TARGETS})

