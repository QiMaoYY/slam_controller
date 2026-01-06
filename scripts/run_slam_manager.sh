#!/bin/bash
# Kuavo SLAM Manager 启动脚本
# 功能：在正确的conda环境中启动slam_manager节点
# 用法：./run_slam_manager.sh

# 注释掉set -e，让脚本在出错时可以显示错误信息而不是立即退出
# set -e

# ============== 配置 ==============
CONDA_ENV="demo"
SLAM_WS="/media/data/slam_ws"

# ============== 颜色输出 ==============
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ============== 主流程 ==============
info "Kuavo SLAM Manager 启动脚本"
info "========================================="

# 注意：此脚本假设在交互式shell中运行（已自动加载.bashrc）
# 如果从GUI启动，terminator会使用 bash -i 启动交互式shell

# 1. 检查conda环境
info "检查conda环境..."
if [ -n "${CONDA_DEFAULT_ENV}" ]; then
    success "已在conda环境: ${CONDA_DEFAULT_ENV}"
else
    info "尝试激活conda环境: ${CONDA_ENV}"
    if [ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]; then
        source "${HOME}/miniconda3/etc/profile.d/conda.sh"
    elif [ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]; then
        source "${HOME}/anaconda3/etc/profile.d/conda.sh"
    fi
    
    if [ -n "$(command -v conda)" ]; then
        conda activate ${CONDA_ENV} 2>/dev/null
        if [ $? -eq 0 ]; then
            success "conda环境 ${CONDA_ENV} 已激活"
        else
            error "激活conda环境失败，使用当前环境"
        fi
    else
        error "未找到conda，使用系统Python"
    fi
fi

info "Python版本: $(python3 --version)"
info "Python路径: $(which python3)"

# 2. 检查ROS环境（bashrc可能已经配置）
info "检查ROS环境..."
if [ -n "${ROS_VERSION}" ]; then
    success "ROS环境已配置 (ROS_VERSION=${ROS_VERSION})"
    info "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"
else
    info "ROS环境未配置，尝试加载..."
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
        success "ROS Noetic环境已加载"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash
        success "ROS Melodic环境已加载"
    else
        error "未找到ROS安装"
    fi
fi

# 3. 检查工作空间（bashrc可能已经配置）
info "检查SLAM工作空间..."
if echo "${ROS_PACKAGE_PATH}" | grep -q "${SLAM_WS}"; then
    success "工作空间已在ROS_PACKAGE_PATH中"
else
    info "加载SLAM工作空间..."
    if [ -f "${SLAM_WS}/devel/setup.bash" ]; then
        source "${SLAM_WS}/devel/setup.bash"
        success "工作空间已加载"
    else
        error "未找到SLAM工作空间: ${SLAM_WS}/devel/setup.bash"
    fi
fi

# 4. 检查roscore
info "检查ROS Master..."
if ! rostopic list &>/dev/null; then
    error "ROS Master未运行"
    error "slam_manager需要roscore运行，请先启动: roscore"
    error "尝试继续运行..."
else
    success "ROS Master已运行"
fi

# 5. 启动slam_manager节点
info "启动SLAM Manager节点..."
info "========================================="
echo ""

python3 ${SLAM_WS}/src/kuavo_slam/scripts/slam_manager.py

# 检查退出状态
EXIT_CODE=$?
if [ ${EXIT_CODE} -ne 0 ]; then
    error "slam_manager异常退出 (退出码: ${EXIT_CODE})"
    echo ""
    echo "按Enter键关闭窗口..."
    read
fi

