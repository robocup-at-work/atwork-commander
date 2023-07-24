cmake_minimum_required(VERSION 2.8.3)
project(atwork_commander_core)

add_compile_options(-g -Wall -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  atwork_commander_msgs
  atwork_commander_gen
  roscpp
)

find_package(Boost 1.58.0 REQUIRED COMPONENTS filesystem program_options)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_control
  CATKIN_DEPENDS atwork_commander_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_control src/control.cpp)

add_executable(${PROJECT_NAME}_node src/atwork_refbox_ros_node.cpp)
add_executable(control src/control_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(control ${PROJECT_NAME}_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  include_directories(${catkin_INCLUDE_DIRS})
  add_executable(control_test tests/src/control.cpp)
  target_link_libraries(control_test ${PROJECT_NAME}_control ${catkin_LIBRARIES}  gtest gmock)
  add_rostest(tests/control.test)
endif()
