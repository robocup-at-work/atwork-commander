cmake_minimum_required(VERSION 2.8.3)
project(atwork_commander)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin COMPONENTS rostest)
  add_rostest(tests/integration_simple.test)
endif()
