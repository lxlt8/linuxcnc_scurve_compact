cmake_minimum_required(VERSION 3.5)

project(test LANGUAGES C)

# Set source and build directories.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Include paths.
include_directories(
    ${SRC_DIR}
    ${SRC_DIR}/../src
    ${SRC_DIR}/../src/gcode
    ${SRC_DIR}/../src/math
    ${SRC_DIR}/../src/interpolate
    ${SRC_DIR}/../src/plot
    ${SRC_DIR}/../src/solver
)

add_executable(test
    main.c
)

# Set include paths for libraries to link.
add_library(clothoid SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(clothoid PROPERTIES
    IMPORTED_LOCATION ${SRC_DIR}/../build/src/libclothoid3d.so
    INTERFACE_INCLUDE_DIRECTORIES ${SRC_DIR}/../src
)

# Link to the math library
target_link_libraries(test
    clothoid -lceres -lm
)

install(TARGETS test
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

