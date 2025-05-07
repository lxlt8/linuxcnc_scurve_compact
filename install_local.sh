#!/bin/bash

HOME_DIR=$(pwd)

# Copy from current directory's subfolders to system paths, where the .deb files are installed.
cp -rf "$HOME_DIR/rtlib/"* /usr/lib/linuxcnc/modules/
cp -rf "$HOME_DIR/lib/"* /usr/lib/
cp -rf "$HOME_DIR/include/"* /usr/include/linuxcnc/
