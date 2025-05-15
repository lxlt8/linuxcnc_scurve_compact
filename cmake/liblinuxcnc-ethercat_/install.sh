#!/bin/bash

HOME_DIR=$(pwd)

mkdir build
cd build
cmake ..
make install
