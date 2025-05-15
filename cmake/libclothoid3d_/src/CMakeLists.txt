cmake_minimum_required(VERSION 3.5)

set(PROJECT "clothoid3d")
project(${PROJECT} VERSION 0 LANGUAGES C CXX)

# Set source and build directories.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Include paths.
include_directories(
    ${SRC_DIR}
    ${SRC_DIR}/gcode
    ${SRC_DIR}/math
    ${SRC_DIR}/plot
    ${SRC_DIR}/solver
)

# Create a shared library instead of an executable
add_library(${PROJECT} SHARED
    eq14.h
    eq15.h
    eq17.h
    eq18.h
    eq19.h
    eq20.h
    eq21.h
    eq22.h
    eq23.h
    eq29.h
    eq30.h
    eq31.h
    eq32.h
    eq33.h
    eq35.h
    eq39.h
    eq40.h
    eq41.h
    eq42.h
    eq43.h
    eq44_G0.h
    eq44_G123.h
    eq45.h
    gauss_legendre.h
    algorithm1.h
    algorithm2.h
    plot/plot_c.h
    plot/plot_cpp.h
    solver/residual_clothoid_fit.h
    solver/residual_function_jacobian.h
    solver/residual_fillet_fit.h
    solver/ceres_capi.h
    solver/ceres_cppapi.h
    solver/ceres_intf.h
    solver/ceres_intf.cpp
    segment.h
    gcode/line3d.h
    gcode/arc3d.h
    gcode/clothoid3d.h
    gcode/fit.h
    math/vector_math.h
    math/geometry_math.h
    experimental/velocity.h
    experimental/ceres_motion_cppapi.h
    experimental/ceres_motion_intf.h
    experimental/ceres_motion_intf.cpp
    experimental/residual_clothoid_motion_fit.h
)

# Link to the math library
target_link_libraries(${PROJECT}
    -lceres
    -lm
)

# Install the shared library
# install(TARGETS  ${PROJECT}
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Make install
# Install both the shared library and its versioned counterpart
install(TARGETS ${PROJECT}
    LIBRARY DESTINATION ${SRC_DIR}/../../../lib
    RUNTIME DESTINATION ${SRC_DIR}/../../../lib  # If you want to install binaries separately
)
