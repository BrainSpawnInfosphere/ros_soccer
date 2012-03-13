cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


pkg_check_modules(EIGEN3 REQUIRED eigen3)
if(EIGEN3_FOUND)
	message("======== FOUND Eigen ${EIGEN3_VERSION} =============")
	include_directories(${EIGEN3_INCLUDE_DIRS})
endif(EIGEN3_FOUND)


find_package(GLUT)
if(GLUT_FOUND)
	message("======== FOUND Glut ${GLUT_VERSION} =============")
endif(GLUT_FOUND)

find_package(OpenGL)
if(OpenGL_FOUND)
	message("======== FOUND OpenGL ${OpenGL_VERSION} =============")
endif(OpenGL_FOUND)


rosbuild_add_executable(soccer src/soccer.cpp src/MadgwickAHRS/MadgwickAHRS.c)
rosbuild_add_executable(soccer_sim src/soccer_sim.cpp)
rosbuild_add_executable(soccer_gui src/soccer_gui.cpp)
target_link_libraries(soccer_gui  ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
#set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")

