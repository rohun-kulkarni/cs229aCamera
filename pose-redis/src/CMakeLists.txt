# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${POSEREDIS_BINARY_DIR})
ADD_EXECUTABLE (pose-redis pose-redis.cpp ${POSEREDIS_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (pose-redis ${POSEREDIS_COMMON_LIBRARIES})

