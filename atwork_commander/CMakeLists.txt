cmake_minimum_required(VERSION 2.8.3)
project(atwork_commander)

add_compile_options(-g -Wall -std=c++11)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS atwork_commander_core rostest)
  include_directories(${catkin_INCLUDE_DIRS})
  add_executable(basic_setup tests/src/basic_setup.cpp)
  target_link_libraries(basic_setup ${catkin_LIBRARIES})
  add_rostest(tests/integration_simple.test)
endif()
