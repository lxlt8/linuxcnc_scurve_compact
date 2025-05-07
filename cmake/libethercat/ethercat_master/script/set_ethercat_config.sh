#!/bin/bash

# Get the MAC address of the first active network interface
MAC_ADDR=$(ip link show | awk '/ether/ {print $2; exit}')

# Define the file path
CONFIG_FILE="/etc/sysconfig/ethercat"

# Write the configuration
echo "MASTER0_DEVICE=$MAC_ADDR" > "$CONFIG_FILE"
echo "DEVICE_MODULES=generic" >> "$CONFIG_FILE"

# Ensure correct permissions
chmod 777 "$CONFIG_FILE"
