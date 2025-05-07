#!/bin/bash
# Purpose: Restart EtherCAT service (run with sudo)

SERVICE_NAME="ethercat"

# Restart logic
if systemctl is-active --quiet "$SERVICE_NAME"; then
    systemctl restart "$SERVICE_NAME" && echo "EtherCAT restarted." || echo "Failed!"
elif [ -f "/etc/init.d/$SERVICE_NAME" ]; then
    /etc/init.d/"$SERVICE_NAME" restart && echo "EtherCAT restarted." || echo "Failed!"
else
    echo "Error: EtherCAT service not found!"
    exit 1
fi
