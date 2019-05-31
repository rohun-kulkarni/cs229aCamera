#  minimum required cmake version: 3.1.0
INCLUDE(FindPkgConfig)
cmake_minimum_required(VERSION 3.1.0)

project(TRACKER)
set(CMAKE_MODULE_PATH "./CMakeFiles/")
SET(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include")

set(CMAKE_PREFIX_PATH "/usr/local/include/")
SET(CMAKE_BUILD_TYPE "Release")

# - HIREDIS
find_library(HIREDIS_LIBRARY hiredis)

# - OPENCV
find_package(OpenCV  HINTS /usr/local/include/opencv4/opencv2 )
#set(OPENCV_INCLUDE_DIRS 

find_package(realsense2 HINTS /usr/local/include/librealsense2)

# set common dependencies
# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# # - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI2-SIMULATION
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-MODEL
FIND_PACKAGE(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-GRAPHICS
FIND_PACKAGE(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - SAI2-URDF
find_package(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})
add_definitions(${SAI2-URDF_DEFINITIONS})

# - SAI2-PRIMITIVES
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)


set(CS225A_COMMON_LIBRARIES
	${CHAI3D_LIBARIES}
	${SAI2-COMMON_LIBRARIES}
	${SAI2-PRIMITIVES_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	)



message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
set(OPENCV_LIBRARY ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})
#add_definitions(${OPENCV_DEFINITIONS})
set (TRACKER_BINARY_DIR                  ${PROJECT_SOURCE_DIR}/bin)
set(TRACKER_COMMON_LIBRARIES
	${OPENCV_LIBRARY}
	${HIREDIS_LIBRARY}
 	${realsense2_LIBRARY}
 	${CHAI3D_LIBARIES}
	${SAI2-COMMON_LIBRARIES}
	${SAI2-PRIMITIVES_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}

	)
add_subdirectory(src)

#find_package(hiredis REQUIRED)
#find_package(opencv REQUIRED)

#add_executable(tracker tracker.cpp)
#add_executable(pose-redis pose-redis.cpp)
#TARGET_LINK_LIBRARIES(tracker hiredis)
#TARGET_LINK_LIBRARIES(pose-redis hiredis)
#set_property(TARGET tracker PROPERTY CXX_STANDARD 11)
#set_property(TARGET pose-redis PROPERTY CXX_STANDARD 11)
#target_link_libraries(tracker ${DEPENDENCIES})
#target_link_libraries(pose-redis ${DEPENDENCIES})



 
#set_target_properties (tracker PROPERTIES
#	FOLDER "Examples/OpenCV"
#)

#install(
#	TARGETS

#	tracker
	#pose-redis

	#RUNTIME DESTINATION
	#${CMAKE_INSTALL_PREFIX}/bin
#)
