cmake_minimum_required(VERSION 2.8.3)
project(iai_tf_log_checker)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED roscpp roslib kdl_parser tf2_kdl tf2_msgs message_runtime)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp kdl_parser tf2_kdl tf2_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/robot_state_publisher.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(tf_log_checker src/${PROJECT_NAME}/tf_log_checker.cpp)
# target_compile_features(tf_log_checker PRIVATE cxx_range_for)
add_dependencies(tf_log_checker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_log_checker ${catkin_LIBRARIES})
