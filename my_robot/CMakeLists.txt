cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ball_chaser)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_service_files(
  FILES
  DriveToTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(drive_bot src/drive_bot.cpp)
add_executable(drive_bot src/process_image.cpp)

target_link_libraries(drive_bot ${catkin_LIBRARIES})
target_link_libraries(process_image ${catkin_LIBRARIES})
