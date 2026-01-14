#!/bin/bash
# Kuavo SLAM Manager 启动脚本
# 功能：在正确的conda环境中启动slam_manager节点
# 用法：./run_slam_manager.sh
# 注意：此脚本假设在交互式shell中运行（已自动加载.bashrc）

CONDA_ENV="demo"
SLAM_WS="/media/data/slam_ws"

source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate ${CONDA_ENV}
source ${SLAM_WS}/devel/setup.bash
python3 ${SLAM_WS}/src/slam_controller/scripts/slam_manager.py
