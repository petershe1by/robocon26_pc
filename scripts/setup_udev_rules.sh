#!/bin/bash
# setup_udev_rules.sh — USB 设备权限规则

set -e

echo "=== 设置 USB CDC 设备权限 ==="
sudo bash -c 'cat > /etc/udev/rules.d/99-robocom.rules' <<LIMIT
# RoboCom USB CDC 设备
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666"
# D435
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666"
LIMIT

sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✓ udev 规则已更新"
echo "如需重新插拔 USB 设备" 
