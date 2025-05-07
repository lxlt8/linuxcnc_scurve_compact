cmake_minimum_required(VERSION 3.0)
set(PROJECT "motmod")
project(${PROJECT} CXX C)

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Set includepaths.
include_directories(${SRC_DIR}/../../src)
include_directories(${SRC_DIR}/../../src/rtapi)
include_directories(${SRC_DIR}/../../src/hal)
include_directories(${SRC_DIR}/../../src/emc/motion)
include_directories(${SRC_DIR}/../../src/emc/nml_intf)
include_directories(${SRC_DIR}/../../src/emc/kinematics)
include_directories(${SRC_DIR}/../../src/emc/tp)
include_directories(${SRC_DIR}/../../src/libnml/posemath)
include_directories(/usr/include/tcl8.6)
include_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include/X11)

# Set prefix.
set(CMAKE_SHARED_LIBRARY_PREFIX "")

# Set compile flags.
SET(GCC_COVERAGE_COMPILE_FLAGS "
    -fno-fast-math -fno-unsafe-math-optimizations -DRTAPI -D_GNU_SOURCE
    -Drealtime -D__MODULE__ -DSIM -fPIC -Os")
add_definitions(-DRTAPI -D_GNU_SOURCE -D__MODULE__ )

ADD_LIBRARY(${PROJECT} SHARED
    ${SRC_DIR}/../../src/emc/kinematics/cubic.c
    ${SRC_DIR}/../../src/emc/motion/axis.c
    ${SRC_DIR}/../../src/emc/motion/motion.c
    ${SRC_DIR}/../../src/emc/motion/motion.h
    ${SRC_DIR}/../../src/emc/motion/command.c 
    ${SRC_DIR}/../../src/emc/motion/control.c
    ${SRC_DIR}/../../src/emc/motion/simple_tp.h
    ${SRC_DIR}/../../src/emc/motion/simple_tp.c
    ${SRC_DIR}/../../src/emc/motion/emcmotutil.c
    ${SRC_DIR}/../../src/emc/motion/stashf.c
    ${SRC_DIR}/../../src/emc/motion/dbuf.c
    ${SRC_DIR}/../../src/emc/tp/tc.h
    ${SRC_DIR}/../../src/emc/tp/tc.c
    ${SRC_DIR}/../../src/emc/tp/tcq.c
    ${SRC_DIR}/../../src/emc/tp/spherical_arc.c
    ${SRC_DIR}/../../src/emc/tp/blendmath.c
    ${SRC_DIR}/../../src/emc/nml_intf/emcpose.c
    ${SRC_DIR}/../../src/libnml/posemath/_posemath.c
    ${SRC_DIR}/../../src/libnml/posemath/sincos.c
)

# Set path to eigen3, for install $ sudo apt-get install libeigen3-dev
include_directories("/usr/include/eigen3")
include_directories("/usr/include/eigen3/Eigen")

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

# Link to libraries.
target_link_libraries(${PROJECT_NAME} PRIVATE
    linuxcnchal
    -ltcl8.6 -ltk8.6 -lX11
    -lm -ltirpc -pthread
)

# Set custom output name :
# set_property(TARGET  ${PROJECT} PROPERTY OUTPUT_NAME thename)

# Make install
install(TARGETS ${PROJECT} DESTINATION ${SRC_DIR}/../../rtlib)

# Notes.

