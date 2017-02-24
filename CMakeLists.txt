cmake_minimum_required(VERSION 2.8.3)
project(raspicam_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  compressed_image_transport
  roscpp
  std_msgs
  camera_info_manager
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm*) 
  FIND_LIBRARY(mmal_LIBS NAMES mmal PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalcore_LIBS NAMES mmal_core PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalutil_LIBS NAMES mmal_util PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalvcclient_LIBS NAMES mmal_vc_client PATHS /opt/vc/lib)
  IF( (NOT mmal_LIBS ) OR (NOT mmalutil_LIBS) OR (NOT mmalcore_LIBS) OR (NOT mmalvcclient_LIBS) )
      MESSAGE(FATAL_ERROR "Could not find mmal libraries")
  ENDIF()

  FIND_LIBRARY(vcos_LIBS NAMES vcos PATHS /opt/vc/lib)
  IF( (NOT vcos_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find vcos library")
  ENDIF()

  FIND_LIBRARY(bcmhost_LIBS NAMES bcm_host PATHS /opt/vc/lib)
  IF( (NOT bcmhost_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find bcm_host library")
  ENDIF()

  FIND_LIBRARY(vchostif_LIBS NAMES vchostif PATHS /opt/vc/lib)
  IF( (NOT vchostif_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find vchostif library")
  ENDIF()

  set(RPI_LIBRARIES ${RPI_LIBRARIES} 
  ${bcmhost_LIBS}
  ${vcos_LIBS} 
  ${mmal_LIBS} 
  ${mmalcore_LIBS} 
  ${mmalutil_LIBS}  
  ${mmalvcclient_LIBS} 
  ${vchostif_LIBS} 
  )
ENDIF()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/Camera.cfg
)


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
#  LIBRARIES raspicam
#  CATKIN_DEPENDS compressed_image_transport roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  /home/pi/userland
  /opt/vc/include
  /opt/vc/include/interface/vcos/pthreads
  /opt/vc/include/interface/vmcs_host/linux
)

## Declare a cpp library
add_library(raspicli STATIC
  src/RaspiCLI.cpp 
)

add_library(raspicamcontrol STATIC
  src/RaspiCamControl.cpp 
)

## Declare a cpp executable
add_executable(raspicam_node src/raspicam_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(raspicam_node raspicam_generate_messages_cpp)
add_dependencies(raspicam_node raspicam_node_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(raspicam_node
  ${catkin_LIBRARIES}
  raspicamcontrol raspicli
  ${RPI_LIBRARIES} 
)

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
install(TARGETS raspicam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  camera_info
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_raspicam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
