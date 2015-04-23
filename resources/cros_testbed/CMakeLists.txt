cmake_minimum_required(VERSION 2.8.3)
project(cros_testbed)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DoubleVector.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   add_two_ints.srv
 )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES cros_testbed
# CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(clock_listener src/clock_listener.cpp)
target_link_libraries(clock_listener ${catkin_LIBRARIES})

add_executable(counter_listener src/counter_listener.cpp)
target_link_libraries(counter_listener ${catkin_LIBRARIES})

add_executable(vector_listener src/vector_listener.cpp)
target_link_libraries(vector_listener ${catkin_LIBRARIES})

add_executable(vector_talker src/vector_talker.cpp)
target_link_libraries(vector_talker ${catkin_LIBRARIES})

add_executable(add_two_ints src/add_two_ints.cpp)
target_link_libraries(add_two_ints ${catkin_LIBRARIES})

add_executable(test_services_list src/test_services.cpp) 
target_link_libraries(test_services_list ${catkin_LIBRARIES})

add_executable(std_msgs_talker src/std_msgs_talker.cpp)
target_link_libraries(std_msgs_talker ${catkin_LIBRARIES})

add_executable(std_msgs_listener src/std_msgs_listener.cpp)
target_link_libraries(std_msgs_listener ${catkin_LIBRARIES})

## Declare a cpp library
# add_library(cros_testbed
#   src/${PROJECT_NAME}/cros_testbed.cpp
# )

## Declare a cpp executable
# add_executable(cros_testbed_node src/cros_testbed_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(clock_listener cros_testbed_generate_messages_cpp)
add_dependencies(counter_listener cros_testbed_generate_messages_cpp)
add_dependencies(vector_listener cros_testbed_generate_messages_cpp)
add_dependencies(vector_talker cros_testbed_generate_messages_cpp)
add_dependencies(add_two_ints cros_testbed_generate_messages_cpp)
add_dependencies(test_services_list cros_testbed_generate_messages_cpp)
add_dependencies(std_msgs_talker cros_testbed_generate_messages_cpp)
add_dependencies(std_msgs_listener cros_testbed_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(cros_testbed_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS cros_testbed cros_testbed_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cros_testbed.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
