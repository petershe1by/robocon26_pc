#!/bin/bash
# setup_autostart.sh — 配置开机自启动（任务 10）
# 注意: USB CDC 设备需要 udev 规则

set -e

SERVICE_NAME="robocom-autostart"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SCRIPT_PATH="$(cd "$(dirname "$0")/.." && pwd)/src/robocom_bringup/robocom_bringup/autostart.py"

# 创建 systemd 服务
sudo bash -c "cat > $SERVICE_FILE" <<EOF
[Unit]
Description=ROBOCON Upper Computer Autostart
After=network.target
After=dev-ttyACM0.device

[Service]
Type=simple
User=\$(whoami)
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
ExecStart=/usr/bin/python3 ${SCRIPT_PATH}
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "=== 设置 USB 权限（USB CDC）==="
sudo bash -c 'cat > /etc/udev/rules.d/99-robocom.rules' <<EOF
# RoboCom USB CDC 设备
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666"
# D435
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "=== 启用自启动服务 ==="
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}
sudo systemctl start ${SERVICE_NAME}

echo "服务状态:"
sudo systemctl status ${SERVICE_NAME} --no-pager
echo ""
echo "日志: journalctl -u ${SERVICE_NAME} -f"
echo "停止: sudo systemctl stop ${SERVICE_NAME}"
echo "禁用: sudo systemctl disable ${SERVICE_NAME}"
