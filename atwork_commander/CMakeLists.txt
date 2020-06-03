cmake_minimum_required(VERSION 2.8.3)
project(atwork_commander)


if(CATKIN_ENABLE_TESTING)
  find_package(catkin COMPONENTS rostest atwork_commander_core)
  catkin_package()
  include_directories(${catkin_INCLUDE_DIRS})
  add_executable(basic_setup tests/src/basic_setup.cpp)
  target_link_libraries(basic_setup ${catkin_LIBRARIES})
  add_rostest(tests/integration_simple.test)
endif()
