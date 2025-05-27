cmake_minimum_required(VERSION 3.0)
set(PROJECT "tpmod_scurve") # Rename this after grotius is done with coding.
set(CMAKE_CXX_STANDARD 17)
project(${PROJECT} C CXX)

# Set source and build directories.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Include paths.
include_directories(
    ${SRC_DIR}
    ${SRC_DIR}/../../src/rtapi
    ${SRC_DIR}/../../src/hal
    ${SRC_DIR}/../../src/libnml/posemath
    ${SRC_DIR}/../../src/emc/tp
    ${SRC_DIR}/../../src/emc/motion
    ${SRC_DIR}/../../src/emc/kinematics
    ${SRC_DIR}/../../src/emc/nml_intf
    ${SRC_DIR}/../../include
    ${SRC_DIR}/../libclothoid3d/src
    ${SRC_DIR}/../libclothoid3d/src/gcode
    ${SRC_DIR}/../libscurve/src
    ${SRC_DIR}/optimizers
)

include_directories(/usr/include/tcl8.6)
include_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include/X11)

# Set path to eigen3, for install $ sudo apt-get install libeigen3-dev
include_directories("/usr/include/eigen3")
include_directories("/usr/include/eigen3/Eigen")

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# If Eigen3 is not found, print a message and suggest how to install it
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 not found. Please install Eigen by running: sudo apt-get install libeigen3-dev")
endif()

# Additional compiler definitions.
add_definitions(-DRTAPI -DUSPACE -D_GNU_SOURCE -D__MODULE__)

# Set prefix.
set(CMAKE_SHARED_LIBRARY_PREFIX "")

# Add the library and set sources.
add_library(${PROJECT} SHARED
    ${SRC_DIR}/../../src/emc/tp/tp_debug.h
    ${SRC_DIR}/../../src/emc/tp/tp.h
    ${SRC_DIR}/../../src/emc/tp/spherical_arc.c
    ${SRC_DIR}/../../src/libnml/posemath/sincos.c
    ${SRC_DIR}/../../src/libnml/posemath/sincos.h
    ${SRC_DIR}/../../src/libnml/posemath/posemath.h
    ${SRC_DIR}/../../src/libnml/posemath/_posemath.c
    ${SRC_DIR}/../libscurve/src/scurve.h
    ${SRC_DIR}/../libscurve/src/scurve.c
    ${SRC_DIR}/../libclothoid3d/src/math/geometry_math.h
    ${SRC_DIR}/../libclothoid3d/src/math/vector_math.h
    ${SRC_DIR}/feed.h
    ${SRC_DIR}/feed.c
    ${SRC_DIR}/arc.h
    ${SRC_DIR}/arc.c
    ${SRC_DIR}/interpolate.h
    ${SRC_DIR}/interpolate.c
    ${SRC_DIR}/halsection.h
    ${SRC_DIR}/module.h
    ${SRC_DIR}/common.h
    ${SRC_DIR}/curve.h
    ${SRC_DIR}/curve.c
    ${SRC_DIR}/emcmot_segment.h
    ${SRC_DIR}/sub_segment.h
    ${SRC_DIR}/tp_scurve.h
    ${SRC_DIR}/tp_scurve.c
    ${SRC_DIR}/vector.h
    ${SRC_DIR}/optimizers/path_standard.h
    ${SRC_DIR}/optimizers/path_standard_subseg.h
    ${SRC_DIR}/optimizers/path_standard_subseg_line_fillet.h
    ${SRC_DIR}/optimizers/path_clothoid.h
    ${SRC_DIR}/optimizers/path_clothoid_abc_uvw.h
    ${SRC_DIR}/look_ahead.h
    ${SRC_DIR}/keyboard.h
    ${SRC_DIR}/follower.h
)

# Set the RPATH for runtime linking
set_target_properties(${PROJECT} PROPERTIES
    INSTALL_RPATH "${SRC_DIR}/../../lib"  # Set the RPATH to your library directory
    BUILD_WITH_INSTALL_RPATH TRUE
)

# Set include paths for libraries to link.
add_library(linuxcnchal SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(linuxcnchal PROPERTIES
    IMPORTED_LOCATION ${SRC_DIR}/../../lib/liblinuxcnchal.so
    INTERFACE_INCLUDE_DIRECTORIES ${SRC_DIR}/../../include
)

# Set include paths for libraries to link.
add_library(clothoid3d SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(clothoid3d PROPERTIES
    IMPORTED_LOCATION ${SRC_DIR}/../../lib/libclothoid3d.so
    INTERFACE_INCLUDE_DIRECTORIES ${SRC_DIR}/../../include
)

# Set include paths for libraries to link.
# add_library(scurve SHARED IMPORTED) # or STATIC instead of SHARED
# set_target_properties(scurve PROPERTIES
#    IMPORTED_LOCATION ${SRC_DIR}/../../lib/libscurve.so
#    INTERFACE_INCLUDE_DIRECTORIES ${SRC_DIR}/../../include
#)

# Link dependencies.
target_link_libraries(${PROJECT} PRIVATE
    pthread rt linuxcnchal
    clothoid3d
    # scurve
    X11
)

# Installation configuration.
install(TARGETS ${PROJECT} DESTINATION ${SRC_DIR}/../../rtlib)

# Ensure README.md is part of the project
configure_file(${CMAKE_SOURCE_DIR}/../README.md ${CMAKE_BINARY_DIR}/../README.md COPYONLY)
