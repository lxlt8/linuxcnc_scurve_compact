cmake_minimum_required(VERSION 3.5)

project(test_scurve LANGUAGES C)

# Set source and build directories.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Include paths.
include_directories(
    ${SRC_DIR}
    ${SRC_DIR}/../src
)

add_executable(test
    main.c
)

# Set include paths for libraries to link.
add_library(libscurve SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(libscurve PROPERTIES
    IMPORTED_LOCATION ${SRC_DIR}/../build/src/libscurve.so
    INTERFACE_INCLUDE_DIRECTORIES ${SRC_DIR}/../src
)

# Link to the math library
target_link_libraries(test
    libscurve -lm
)

install(TARGETS test
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

