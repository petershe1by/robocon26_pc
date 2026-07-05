#!/bin/bash
# =============================================================================
# ROBOCON 上位机 — Ubuntu 22.04 一键环境配置脚本
# 使用方式（全新系统）：
#   sudo apt update && sudo apt install -y git curl wget
#   bash <(curl -fsSL https://raw.githubusercontent.com/petershe1by/robocon26_pc/main/scripts/setup_all.sh)
# 或克隆后本地运行：
#   cd ~/ros2_ws/src/robocom_pc && bash scripts/setup_all.sh
# =============================================================================
set -euo pipefail

# ---- 颜色 ----
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
log()  { echo -e "${GREEN}[✓]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }
err()  { echo -e "${RED}[✗]${NC} $1"; exit 1; }
info() { echo -e "${CYAN}[→]${NC} $1"; }

# ---- 检测系统 ----
check_system() {
    info "检测系统版本..."
    if [ ! -f /etc/os-release ]; then
        err "无法识别系统 (/etc/os-release 不存在)"
    fi
    . /etc/os-release
    if [ "$ID" != "ubuntu" ] || [ "$VERSION_ID" != "22.04" ]; then
        warn "当前系统: $PRETTY_NAME，脚本针对 Ubuntu 22.04 编写"
        warn "继续运行可能失败，建议使用 Ubuntu 22.04 LTS"
        read -rp "是否继续？(y/N) " ans
        [[ "$ans" != "y" && "$ans" != "Y" ]] && exit 1
    fi
    log "系统: $PRETTY_NAME"
}

# ---- 1. 系统基础工具 ----
install_base() {
    info "安装系统基础工具..."
    sudo apt update
    sudo apt install -y \
        git curl wget vim htop cmake build-essential \
        python3-pip python3-venv python3-dev \
        software-properties-common locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    log "基础工具安装完成"
}

# ---- 2. ROS2 Humble ----
install_ros2() {
    if [ -d /opt/ros/humble ]; then
        warn "ROS2 Humble 已安装，跳过"
        return
    fi
    info "安装 ROS2 Humble..."

    # 添加 ROS2 源
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu jammy main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y \
        ros-humble-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-rosidl-default-generators

    # rosdep
    sudo rosdep init || true  # 可能已初始化
    rosdep update || true

    # 环境变量
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash

    log "ROS2 Humble 安装完成"
}

# ---- 3. Python 依赖 ----
install_python_deps() {
    info "安装 Python 依赖..."
    pip3 install --upgrade pip

    # 核心依赖（必须）
    pip3 install sympy pyserial PySide6 opencv-python numpy ultralytics pyrealsense2

    # 可选依赖（降级使用）
    warn "安装可选依赖：paddleocr / paddlepaddle / pyttsx3 / transforms3d..."
    warn "如果安装极慢或失败，可按 Ctrl+C 跳过，节点会自动降级为模拟模式"
    pip3 install paddleocr paddlepaddle pyttsx3 transforms3d || true

    log "Python 依赖安装完成"
}

# ---- 4. 创建工作空间并克隆代码 ----
setup_workspace() {
    local ws="$HOME/ros2_ws"

    if [ -d "$ws/src/robocom_pc/.git" ]; then
        warn "工作空间已存在，更新代码..."
        cd "$ws/src/robocom_pc"
        git pull origin main
        return
    fi

    info "创建工作空间..."
    mkdir -p "$ws/src"
    cd "$ws"

    # 克隆本仓库
    git clone https://github.com/petershe1by/robocon26_pc.git src/robocom_pc

    log "代码克隆完成"
}

# ---- 5. 编译 ----
build_ws() {
    local ws="$HOME/ros2_ws"
    cd "$ws"

    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash

    info "编译 robocom_interfaces（msg/srv）..."
    colcon build --symlink-install --packages-select robocom_interfaces

    info "编译所有包..."
    colcon build --symlink-install

    # 环境变量
    if ! grep -q "source $ws/install/setup.bash" ~/.bashrc; then
        echo "source $ws/install/setup.bash" >> ~/.bashrc
    fi
    # shellcheck source=/dev/null
    source "$ws/install/setup.bash"

    log "编译完成"
}

# ---- 6. udev 规则 ----
setup_udev() {
    if [ -f /etc/udev/rules.d/99-robocom.rules ]; then
        warn "udev 规则已存在"
        return
    fi
    info "配置 USB 设备权限..."
    sudo bash -c 'cat > /etc/udev/rules.d/99-robocom.rules' <<EOF
# STM32 USB CDC
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666"
# Intel RealSense D435
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    log "udev 规则已配置（重新插拔 USB 生效）"
}

# ---- 7. 模型文件提示 -#
notify_models() {
    local model_dir="$HOME/ros2_ws/src/robocom_pc/src/robocom_vision/models"
    if [ -f "$model_dir/block_detector.pt" ]; then
        log "YOLO 模型文件已存在"
    else
        warn "YOLO 模型文件未找到，需要手动下载："
        warn "  将 block_detector.pt 放入: $model_dir/"
        warn "  或通过软链接关联: ln -s /path/to/block_detector.pt $model_dir/"
    fi
}

# ---- 8. 验证 -#
verify() {
    info "验证安装..."
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
    source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true

    local ok=true
    python3 -c "from robocom_interfaces.msg import MotionCmd" 2>/dev/null && log "  robocom_interfaces  OK" || { warn "  robocom_interfaces  FAIL"; ok=false; }
    python3 -c "import serial" 2>/dev/null && log "  pyserial          OK" || { warn "  pyserial          FAIL"; ok=false; }
    python3 -c "import cv2" 2>/dev/null && log "  opencv-python     OK" || { warn "  opencv-python     FAIL"; ok=false; }
    python3 -c "import PySide6" 2>/dev/null && log "  PySide6           OK" || { warn "  PySide6           FAIL"; ok=false; }
    python3 -c "from ultralytics import YOLO" 2>/dev/null && log "  ultralytics       OK" || { warn "  ultralytics       FAIL"; ok=false; }

    if $ok; then
        log "核心依赖全部通过"
    else
        warn "部分依赖未安装，节点会自动降级运行"
    fi
}

# ---- 主流程 -#
main() {
    echo ""
    echo -e "${CYAN}==============================================${NC}"
    echo -e "${CYAN}  ROBOCON 上位机 — Ubuntu 22.04 一键配置${NC}"
    echo -e "${CYAN}==============================================${NC}"
    echo ""

    # 如果以 root 运行，降权到普通用户
    if [ "$EUID" -eq 0 ]; then
        warn "不要以 root 运行此脚本！"
        warn "请用普通用户执行: bash setup_all.sh"
        exit 1
    fi

    check_system
    install_base
    install_ros2
    install_python_deps
    setup_workspace
    build_ws
    setup_udev
    notify_models
    verify

    echo ""
    echo -e "${GREEN}==============================================${NC}"
    echo -e "${GREEN}  环境配置完成！${NC}"
    echo -e "${GREEN}==============================================${NC}"
    echo ""
    echo "下一步："
    echo "  1. 重新插拔 USB 设备使 udev 规则生效"
    echo "  2. 下载 YOLO 模型到 src/robocom_vision/models/"
    echo ""
    echo "  # 启动所有节点（无硬件模拟模式）"
    echo "  ros2 launch robocom_bringup all_start.launch.py"
    echo ""
    echo "  # 或者一键全程启动含雷达："
    echo "  ros2 run robocom_bringup autostart"
    echo ""
}

main "$@"
