cmake_minimum_required(VERSION 2.8.3)
project(atwork_commander)

add_compile_options(-g -Wall -std=c++11)

find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(PROGRAMS
  scripts/default_bringup
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS atwork_commander_core rostest)
  include_directories(${catkin_INCLUDE_DIRS})
  add_executable(basic_setup tests/src/basic_setup.cpp)
  target_link_libraries(basic_setup ${catkin_LIBRARIES})
  add_rostest(tests/integration_simple.test)
endif()
