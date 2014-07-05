cmake_minimum_required(VERSION 2.8.3)
project(soccer)

###
# catkin_make -DCMAKE_INSTALL_PREFIX=/Users/kevin/catkin/install install
###

find_package(catkin REQUIRED COMPONENTS 
   roscpp 
   rospy 
   geometry_msgs 
   sensor_msgs 
   tf 
   std_msgs 
#   ip_serial 
   nav_msgs 
   message_generation
)

find_package(PkgConfig)
find_package(Boost REQUIRED COMPONENTS signals program_options)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

  # Declare the message files to be built
  add_message_files(FILES
    Battery.msg
  )

  # Actually generate the language-specific message and service files
  generate_messages(DEPENDENCIES geometry_msgs std_msgs sensor_msgs)

## Declare things to be passed to other projects
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs tf std_msgs nav_msgs
  DEPENDS Boost # other-3rd-party-libraries
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


################ EIGEN3 ##############################
pkg_check_modules(EIGEN3 REQUIRED eigen3)
if(EIGEN3_FOUND)
	message("======== FOUND Eigen ${EIGEN3_VERSION} =============")
	include_directories(${EIGEN3_INCLUDE_DIRS})
endif(EIGEN3_FOUND)

# Code -----------------
add_executable(soccer src/soccer.cpp)
target_link_libraries(soccer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(soccer soccer_gencpp) # this will build the msgs/srv


## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )