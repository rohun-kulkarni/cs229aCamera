#  minimum required cmake version: 3.1.0
INCLUDE(FindPkgConfig)
cmake_minimum_required(VERSION 3.1.0)

project(POSEREDIS)
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



message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
set(OPENCV_LIBRARY ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})
#add_definitions(${OPENCV_DEFINITIONS})
set (POSEREDIS_BINARY_DIR                  ${PROJECT_SOURCE_DIR}/bin)
set(POSEREDIS_COMMON_LIBRARIES
	${OPENCV_LIBRARY}
	${HIREDIS_LIBRARY}
 	${realsense2_LIBRARY}

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
