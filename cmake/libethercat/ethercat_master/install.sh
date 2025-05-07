#!/bin/bash

# Ethercat-master install script.
# make clean

HOME_DIR=$(pwd)

# Dependencies.
sudo apt-get install -y libtool autoconf pkg-config g++

#./bootstrap
#./configure
make all modules
sudo make modules_install install
sudo depmod # Update dependencies

# Copy config file.
cd /etc/
sudo mkdir -p sysconfig
cd $HOME_DIR/script/sysconfig
sudo cp ethercat /etc/sysconfig/
sudo chmod 777 /etc/sysconfig/ethercat

# Copy startup script and set as executable.
cd $HOME_DIR/script/init.d
sudo cp ethercat /etc/init.d/
sudo chmod +x /etc/init.d/ethercat
sudo chmod 777 /etc/init.d/ethercat

sudo chmod +x $HOME_DIR/script/set_ethercat_config.sh
sudo chmod 777 $HOME_DIR/script/set_ethercat_config.sh

cd $HOME_DIR/script/
sudo ./set_ethercat_config.sh

sudo echo "KERNEL=="\"EtherCAT[0-9]*\"", MODE="\"777\"", GROUP=""\"ethercat\"" > /etc/udev/rules.d/99-EtherCAT.rules
sudo chmod go+rwx /etc/udev/rules.d/99-EtherCAT.rules
