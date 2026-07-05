#!/usr/bin/env bash
# =============================================================================
# ROBOCON 上位机 — Ubuntu 22.04 一键环境配置脚本
# 用法: chmod +x setup_all.sh && ./setup_all.sh
# 说明: 在全新 Ubuntu 22.04 上从零搭建完整开发/部署环境
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="${HOME}/ros2_ws"
REPO_URL="https://github.com/petershe1by/robocon26_pc.git"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'  # No Color

log_info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# ---------------------------------------------------------------------------
# 中断处理：被 Ctrl+C 中断时提示
# ---------------------------------------------------------------------------
trap 'log_warn "脚本被用户中断"; exit 1' INT TERM

# ---------------------------------------------------------------------------
# 检查系统
# ---------------------------------------------------------------------------
check_system() {
    log_info "检查系统版本..."

    if [ ! -f /etc/os-release ]; then
        log_error "无法识别操作系统（缺少 /etc/os-release）"
        exit 1
    fi

    source /etc/os-release
    if [ "$ID" != "ubuntu" ]; then
        log_error "仅支持 Ubuntu，当前系统: $ID"
        exit 1
    fi

    case "$VERSION_ID" in
        22.04|20.04)
            log_ok "Ubuntu $VERSION_ID (${VERSION_CODENAME})"
            ;;
        *)
            log_warn "未在 Ubuntu $VERSION_ID 上充分测试，推荐 22.04"
            ;;
    esac
}

# ---------------------------------------------------------------------------
# 步骤 1: 系统基础工具
# ---------------------------------------------------------------------------
step_system_tools() {
    log_info "===== 步骤 1/9: 安装系统基础工具 ====="

    sudo apt update
    sudo apt install -y --no-install-recommends \
        git \
        curl \
        wget \
        cmake \
        build-essential \
        python3-pip \
        python3-dev \
        python3-venv \
        software-properties-common \
        ca-certificates \
        gnupg \
        lsb-release \
        net-tools

    log_ok "系统基础工具已安装"
}

# ---------------------------------------------------------------------------
# 步骤 2: ROS2 Humble
# ---------------------------------------------------------------------------
step_ros2() {
    log_info "===== 步骤 2/9: 安装 ROS2 Humble ====="

    if command -v ros2 &>/dev/null; then
        log_ok "ROS2 已安装，跳过"
        return
    fi

    # 设置 locale
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    # 添加 ROS2 源
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y --no-install-recommends \
        ros-humble-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-ament-package \
        ros-humble-rosidl-default-generators \
        ros-humble-cv-bridge

    # 初始化 rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init || true
    fi
    rosdep update 2>/dev/null || true

    # 添加到 bashrc
    grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc 2>/dev/null || \
        echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

    export ROS_DOMAIN_ID=0
    grep -qxF 'export ROS_DOMAIN_ID=0' ~/.bashrc 2>/dev/null || \
        echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc

    log_ok "ROS2 Humble 已安装"
}

# ---------------------------------------------------------------------------
# 步骤 3: 克隆仓库
# ---------------------------------------------------------------------------
step_clone_repo() {
    log_info "===== 步骤 3/9: 克隆 robocon26_pc 仓库 ====="

    mkdir -p "${WORKSPACE_DIR}/src"

    if [ -d "${WORKSPACE_DIR}/src/robocom_pc/.git" ]; then
        log_info "仓库已存在，拉取最新代码"
        cd "${WORKSPACE_DIR}/src/robocom_pc"
        git pull origin main
    else
        git clone "${REPO_URL}" "${WORKSPACE_DIR}/src/robocom_pc"
        log_ok "仓库已克隆到 ${WORKSPACE_DIR}/src/robocom_pc"
    fi
}

# ---------------------------------------------------------------------------
# 步骤 4: 安装 Python 依赖
# ---------------------------------------------------------------------------
step_python_deps() {
    log_info "===== 步骤 4/9: 安装 Python 依赖 ====="

    pip3 install --upgrade pip setuptools wheel

    # 核心依赖（必需）
    pip3 install \
        sympy \
        pyserial \
        PySide6 \
        opencv-python \
        numpy \
        ultralytics \
        pyrealsense2

    # 可选依赖
    log_info "安装可选依赖（OCR / TTS / 坐标变换）..."
    pip3 install \
        paddlepaddle \
        paddleocr \
        pyttsx3 \
        transforms3d 2>/dev/null || log_warn "部分可选依赖安装失败，可忽略"

    log_ok "Python 依赖已安装"
}

# ---------------------------------------------------------------------------
# 步骤 5: Livox 雷达驱动
# ---------------------------------------------------------------------------
step_livox_driver() {
    log_info "===== 步骤 5/9: 编译 Livox 雷达驱动（Mid-360） ====="

    if [ -d "${HOME}/livox_ros_driver2" ]; then
        log_info "Livox driver 已存在，更新..."
        cd "${HOME}/livox_ros_driver2"
        git pull origin master
    else
        git clone https://github.com/Livox-SDK/livox_ros_driver2.git "${HOME}/livox_ros_driver2"
    fi

    cd "${HOME}/livox_ros_driver2"
    if ./build.sh humble 2>/dev/null; then
        log_ok "Livox 雷达驱动编译成功"
    else
        log_warn "Livox 驱动编译失败（可能缺少依赖或无雷达硬件），可跳过"
    fi
}

# ---------------------------------------------------------------------------
# 步骤 6: 编译工作空间
# ---------------------------------------------------------------------------
step_build_workspace() {
    log_info "===== 步骤 6/9: 编译 ROS2 工作空间 ====="

    source /opt/ros/humble/setup.bash

    cd "${WORKSPACE_DIR}"

    # 先编译 msg 接口
    colcon build --symlink-install --packages-select robocom_interfaces 2>&1 | tail -5
    # 再编译全部
    colcon build --symlink-install 2>&1 | tail -5

    # 添加 setup.bash 到 bashrc
    grep -qxF "source ${WORKSPACE_DIR}/install/setup.bash" ~/.bashrc 2>/dev/null || \
        echo "source ${WORKSPACE_DIR}/install/setup.bash" >> ~/.bashrc

    source "${WORKSPACE_DIR}/install/setup.bash"
    log_ok "ROS2 工作空间编译完成"
}

# ---------------------------------------------------------------------------
# 步骤 7: USB 设备权限
# ---------------------------------------------------------------------------
step_udev_rules() {
    log_info "===== 步骤 7/9: 配置 USB 设备 udev 规则 ====="

    if [ -f "${SCRIPT_DIR}/setup_udev_rules.sh" ]; then
        sudo bash "${SCRIPT_DIR}/setup_udev_rules.sh"
        log_ok "udev 规则已配置"
    else
        log_info "写入默认 udev 规则..."
        sudo tee /etc/udev/rules.d/99-robocom.rules > /dev/null <<'RULES'
# STM32 USB CDC (SBUS)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666"
# Intel RealSense D435
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666"
RULES
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        log_ok "udev 规则已配置（重新插拔 USB 后生效）"
    fi
}

# ---------------------------------------------------------------------------
# 步骤 8: 验证安装
# ---------------------------------------------------------------------------
step_verify() {
    log_info "===== 步骤 8/9: 验证安装 ====="

    local errors=0

    # 验证 ROS2
    if command -v ros2 &>/dev/null; then
        log_ok "ros2 CLI: $(ros2 --version 2>/dev/null | head -1)"
    else
        log_error "ros2 CLI 不可用"
        errors=$((errors + 1))
    fi

    # 验证 Python 包
    source "${WORKSPACE_DIR}/install/setup.bash" 2>/dev/null || true

    local py_pkgs=(
        "sympy:sympy"
        "serial:serial"
        "PySide6.QtWidgets:PySide6"
        "cv2:opencv-python"
        "numpy:numpy"
        "rclpy:rclpy"
    )

    for entry in "${py_pkgs[@]}"; do
        local mod="${entry%%:*}"
        local name="${entry##*:}"
        if python3 -c "import ${mod}" 2>/dev/null; then
            log_ok "Python: ${name}"
        else
            log_warn "Python: ${name} 未安装（节点会降级运行）"
        fi
    done

    # 验证接口
    if python3 -c "from robocom_interfaces.msg import MotionCmd; print('robocom_interfaces OK')" 2>/dev/null; then
        log_ok "ROS2 自定义接口已编译"
    else
        log_error "ROS2 自定义接口编译失败"
        errors=$((errors + 1))
    fi

    if [ $errors -eq 0 ]; then
        log_ok "全部检查通过"
    else
        log_warn "有 ${errors} 项检查未通过，请查看上方日志"
    fi
}

# ---------------------------------------------------------------------------
# 步骤 9: 模型文件提示
# ---------------------------------------------------------------------------
step_model_hint() {
    log_info "===== 步骤 9/9: 模型文件提示 ====="

    local model_dir="${WORKSPACE_DIR}/src/robocom_pc/src/robocom_vision/models"
    local model_file="${model_dir}/block_detector.pt"

    if [ -f "${model_file}" ]; then
        log_ok "YOLO 模型文件已存在: ${model_file}"
    else
        log_warn "YOLO 模型文件不存在，请手动放置:"
        echo -e "  ${CYAN}mkdir -p ${model_dir}"
        echo -e " 将 block_detector.pt 复制到 ${model_dir}/${NC}"
    fi
}

# ---------------------------------------------------------------------------
# 完成
# ---------------------------------------------------------------------------
print_summary() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  ROBOCON 上位机环境配置完成！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "  运行方式:"
    echo "    source ~/.bashrc"
    echo "    ros2 launch robocom_bringup all_start.launch.py"
    echo ""
    echo "  单独启动 UI:"
    echo "    ros2 run robocom_ui robocom_ui"
    echo ""
    echo "  调试命令:"
    echo "    ros2 topic list"
    echo "    ros2 topic echo /robot_state"
    echo "    ros2 service call /start_mission ..."
    echo ""
    echo "  开机自启（部署后执行）:"
    echo "    sudo bash ${SCRIPT_DIR}/setup_autostart.sh"
    echo ""
    echo "  模型文件:"
    echo "    将 block_detector.pt 放入 ${WORKSPACE_DIR}/src/robocom_pc/src/robocom_vision/models/"
    echo "    然后重新编译: colcon build --symlink-install"
    echo ""
}

# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
main() {
    echo ""
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  ROBOCON 上位机 — Ubuntu 22.04 一键配置${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    check_system
    step_system_tools
    step_ros2
    step_clone_repo
    step_python_deps
    step_livox_driver
    step_build_workspace
    step_udev_rules
    step_verify
    step_model_hint
    print_summary
}

main "$@"
