cmake_minimum_required(VERSION 3.5)

set(PROJECT "scurve")
project(${PROJECT} VERSION 0 LANGUAGES C CXX)

# Set source and build directories.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Include paths.
include_directories(
    ${SRC_DIR}
)

# Create a shared library instead of an executable
add_library(${PROJECT} SHARED
    scurve.h
    scurve.c  
)

# Link to the math library
target_link_libraries(${PROJECT}
    -lm
)

# Install the shared library
install(TARGETS ${PROJECT}
    LIBRARY DESTINATION ${SRC_DIR}/../../../lib
    RUNTIME DESTINATION ${SRC_DIR}/../../../lib  # If you want to install binaries separately
)
