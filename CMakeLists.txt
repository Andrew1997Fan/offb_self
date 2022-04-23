cmake_minimum_required(VERSION 2.8.3)
project(offb_self)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  )
find_package(Eigen3 REQUIRED)
find_package(message_generation)
#set(apriltags2_ros_DIR /home/ncrl/svo_install_overlay_ws/src)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb_self
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})
## Declare a C++ library
# add_library(offb_self
#   src/formation_four.cpp
# )
#add_library(offb_self include/hungarian.cpp include/Hungarian.cpp)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(offb_self ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(formation_four src/formation_four.cpp)

#add_executable(formation_vel src/formation_vel.cpp)
#add_executable(formation_three_interaction src/formation_three_interaction.cpp)
#add_executable(formation_four src/formation_four.cpp)
#add_executable(formation_two_interaction src/formation_three_interaction.cpp)
#add_executable(offb_main src/offb_main.cpp)

add_executable(follow_test2 src/follow_test2.cpp)
add_executable(velocity_control src/velocity_control.cpp)
add_executable(GPS_follow_test src/GPS_follow_test.cpp)
add_executable(offb_vel src/offb_vel.cpp)
add_executable(offb_vel_kb_test src/offb_vel_kb_test.cpp)
add_executable(pub_rc src/pub_rc.cpp)
add_executable(trajectory src/trajectory.cpp)
add_executable(GPS_setpoint src/GPS_setpoint.cpp)
add_executable(GPS_position_control src/GPS_position_control.cpp)

#add_executable(vision_down src/vision_down.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(formation_four ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(follow_test2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 #add_dependencies(vision_down ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(formation_vel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(formation_three_interaction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(formation_two_interaction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
# target_link_libraries(formation_four
#   ${catkin_LIBRARIES}
#   )

#target_link_libraries(offb_main
#  ${catkin_LIBRARIES}
#  )

target_link_libraries(follow_test2
  ${catkin_LIBRARIES}
  )
target_link_libraries(GPS_follow_test
  ${catkin_LIBRARIES}
  )
target_link_libraries(velocity_control
  ${catkin_LIBRARIES}
  )
target_link_libraries(offb_vel_kb_test
  ${catkin_LIBRARIES}
  )
target_link_libraries(offb_vel
  ${catkin_LIBRARIES}
  )
target_link_libraries(pub_rc
  ${catkin_LIBRARIES}
  )
target_link_libraries(trajectory
  ${catkin_LIBRARIES}
  )
target_link_libraries(GPS_setpoint
  ${catkin_LIBRARIES}
  )
target_link_libraries(GPS_position_control
  ${catkin_LIBRARIES}
  )

#===============================================#
#		GPS_mission			#
#===============================================#
set(INC
	./src
	./src/GPS_mission
	./src/GPS_mission/lib
	./src/GPS_mission/autopilot
	)
set(SRC
	./src/GPS_mission/GPS_mission.cpp
	./src/GPS_mission/gps_transform.cpp
	./src/GPS_mission/autopilot/autopilot.cpp
	)
include_directories(
	  ${catkin_INCLUDE_DIRS}
	  ${EIGEN3_INCLUDE_DIR}
	  ${INC}
	  )

add_executable(GPS_mission
	${SRC}
)
target_link_libraries(GPS_mission
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  )


#target_link_libraries(vision_down
#  ${catkin_LIBRARIES}
#  )
#target_link_libraries(formation_vel
#  ${catkin_LIBRARIES}
#  )
#target_link_libraries(formation_three_interaction
#  ${catkin_LIBRARIES}
#  offb_self
#  )
#target_link_libraries(formation_four
#  ${catkin_LIBRARIES}
#  )
#target_link_libraries(formation_two_interaction
#  ${catkin_LIBRARIES}
#  )
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_offb_self.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
