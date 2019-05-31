# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TRACKER_BINARY_DIR})
ADD_EXECUTABLE (tracker tracker.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (tracker ${TRACKER_COMMON_LIBRARIES})

