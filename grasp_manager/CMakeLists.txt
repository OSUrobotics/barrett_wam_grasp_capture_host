cmake_minimum_required(VERSION 2.8.3)
project(grasp_manager)

set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  tf
  image_transport
  pcl_ros
  cv_bridge
  geometry_msgs
  sensor_msgs
  std_msgs
  wam_msgs
  grasp_data_msgs
  std_srvs
  wam_srvs
  bag_tools
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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

## Generate added messages and services with any dependencies listed here


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
#  LIBRARIES grasp_manager
  CATKIN_DEPENDS bag_tools geometry_msgs rosbag roscpp rospy sensor_msgs std_msgs std_srvs tf wam_msgs wam_srvs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp library
# add_library(grasp_manager
#   src/${PROJECT_NAME}/grasp_manager.cpp
# )

## Declare a cpp executable
add_executable(depth_decompressor src/depth_decompressor.cpp)
target_link_libraries(depth_decompressor ${catkin_LIBRARIES})

add_executable(pointcloud_transformer src/pointcloud_transformer.cpp)
target_link_libraries(pointcloud_transformer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_transformer grasp_data_msgs_generate_messages_cpp)

add_executable(pointcloud_saver src/pointcloud_saver.cpp)
target_link_libraries(pointcloud_saver ${catkin_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(pointcloud_saver PROPERTIES COMPILE_FLAGS "-Wall -std=c++0x")

add_executable(polynomial_smoother src/polynomial_smoother.cpp src/pcl_to_ros.cpp)
target_link_libraries(polynomial_smoother ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ror_filter src/ror_filter.cpp src/pcl_to_ros.cpp)
target_link_libraries(ror_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(online_pointcloud_transformer src/online_pointcloud_transformer.cpp)
target_link_libraries(online_pointcloud_transformer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(grasp_manager_node grasp_manager_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(grasp_manager_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS grasp_manager grasp_manager_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grasp_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
