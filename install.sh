#!/bin/bash

HOME_DIR=$(pwd)

cd cmake
./patch
./deps_lcnc
./build_lcnc
./build_cmake
