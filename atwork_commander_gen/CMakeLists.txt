cmake_minimum_required(VERSION 3.0.0)
project(atwork_commander_gen)

add_compile_options(-std=c++14 -g)

find_package(catkin REQUIRED COMPONENTS
              atwork_commander_msgs
              roscpp
              pluginlib
             )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/TaskGenerator.cpp
  src/Definitions.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_support
  src/DefaultConfigParser.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_support ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_plugins
  src/simple/Generator.cpp
  src/jurek/Generator.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME}_plugin_support ${catkin_LIBRARIES})

add_executable(ExampleGenerator src/ExampleNode.cpp)
target_link_libraries(ExampleGenerator ${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


if(CATKIN_ENABLE_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIR})
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_executable(${PROJECT_NAME}_tests tests/tests.cpp)
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_rostest(tests/run.test)
endif()
