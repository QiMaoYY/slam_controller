#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam_controller 工具函数与通用类型
"""

import glob
import os
import re
import shutil
from enum import Enum

import rospy
import yaml


class SlamState(Enum):
    """SLAM系统状态枚举"""
    IDLE = "idle"                    # 空闲状态
    MAPPING = "mapping"              # 建图中
    LOCALIZING = "localizing"        # 定位中
    NAVIGATING = "navigating"        # 导航中
    PROCESSING = "processing"        # 地图处理中
    ERROR = "error"                  # 错误状态


def bool_to_sh(v: bool) -> str:
    """将bool转换为shell脚本期望的 true/false 字符串。"""
    return 'true' if bool(v) else 'false'


def strip_ansi(text: str) -> str:
    """移除ANSI转义序列（颜色代码等）"""
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    return ansi_escape.sub('', text)


def cfg_get(cfg: dict, key: str, default=None):
    """从cfg中读取点分key（如 paths.slam_ws），并做路径展开。"""
    cur = cfg or {}
    for part in key.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    if isinstance(cur, str):
        return os.path.expandvars(os.path.expanduser(cur))
    return cur


def load_global_config() -> dict:
    """
    读取 slam_controller 全局配置文件。

    默认路径：<slam_controller>/config/slam_controller.yaml
    可用参数覆盖：~global_config
    """
    cfg_path = rospy.get_param('~global_config', '')
    if not cfg_path:
        try:
            import rospkg
            pkg_path = rospkg.RosPack().get_path('slam_controller')
            cfg_path = os.path.join(pkg_path, 'config', 'slam_controller.yaml')
        except Exception:
            cfg_path = ''

    if cfg_path and os.path.isfile(cfg_path):
        try:
            import yaml
            with open(cfg_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            if isinstance(data, dict):
                rospy.loginfo(f"已加载全局配置: {cfg_path}")
                return data
            rospy.logwarn(f"全局配置格式不为dict: {cfg_path}，将使用空配置")
        except Exception as e:
            rospy.logwarn(f"加载全局配置失败({cfg_path}): {e}，将使用空配置")
    else:
        rospy.logwarn("未找到全局配置文件，将使用空配置")

    return {}


def validate_map_name(map_name: str) -> tuple:
    """
    验证地图名称的合法性

    Args:
        map_name: 地图名称

    Returns:
        (bool, str): (是否合法, 错误信息)
    """
    if not map_name:
        return False, "地图名称不能为空"

    if not re.match(r'^[a-zA-Z0-9_\-]+$', map_name):
        return False, "地图名称只能包含字母、数字、下划线和横线"

    if len(map_name) > 50:
        return False, "地图名称过长（最多50个字符）"

    return True, ""


def validate_nav_map_ready(map_root: str,
                           map_name: str,
                           map2d_yaml_name: str,
                           map2d_pgm_name: str,
                           nav_pointcloud_name: str) -> tuple:
    """
    校验指定地图是否满足导航启动要求：
    - 目录存在
    - 必须包含 map2d.yaml / map2d.pgm / pointcloud.pcd
    """
    map_dir = os.path.join(map_root, map_name)
    if not os.path.isdir(map_dir):
        return False, f"地图目录不存在: {map_dir}"

    required = [
        os.path.join(map_dir, map2d_yaml_name),
        os.path.join(map_dir, map2d_pgm_name),
        os.path.join(map_dir, nav_pointcloud_name),
    ]
    missing = [p for p in required if not os.path.isfile(p)]
    if missing:
        return False, f"地图未准备好用于导航，缺少文件: {', '.join(missing)}"
    return True, ""


def save_map(map_name: str,
             kuavo_slam_path: str,
             faster_lio_pcd_dir: str,
             ori_pointcloud_name: str) -> tuple:
    """
    保存地图文件

    Args:
        map_name: 地图名称
        kuavo_slam_path: kuavo_slam 包路径
        faster_lio_pcd_dir: faster-lio 的PCD目录
        ori_pointcloud_name: 原始点云保存文件名

    Returns:
        (bool, str): (是否成功, 消息)
    """
    try:
        valid, error_msg = validate_map_name(map_name)
        if not valid:
            return False, error_msg

        maps_dir = os.path.join(kuavo_slam_path, 'maps')
        map_dir = os.path.join(maps_dir, map_name)

        if os.path.exists(map_dir):
            return False, f"地图 '{map_name}' 已存在"

        if not os.path.exists(faster_lio_pcd_dir):
            return False, f"未找到faster-lio PCD目录: {faster_lio_pcd_dir}"

        pcd_files = glob.glob(os.path.join(faster_lio_pcd_dir, '*.pcd'))
        if not pcd_files:
            return False, "未找到PCD地图文件，可能建图时间太短"

        os.makedirs(map_dir, exist_ok=True)
        rospy.loginfo(f"创建地图目录: {map_dir}")

        saved_files = []
        for pcd_file in pcd_files:
            src_filename = os.path.basename(pcd_file)
            dst_filename = ori_pointcloud_name
            dst_path = os.path.join(map_dir, dst_filename)
            shutil.move(pcd_file, dst_path)
            saved_files.append(dst_filename)
            rospy.loginfo(f"已保存: {src_filename} -> {dst_path}")

        msg = f"地图已保存到: {map_dir}/ (文件: {', '.join(saved_files)})"
        rospy.loginfo(f"{msg}")
        return True, msg

    except Exception as e:
        error_msg = f"保存地图时发生错误: {str(e)}"
        rospy.logerr(f"{error_msg}")
        return False, error_msg


NAV_TASKS_FILENAME = "nav_tasks.yaml"


def nav_tasks_path(map_root: str, map_name: str) -> str:
    """获取导航任务文件路径"""
    return os.path.join(map_root, map_name, NAV_TASKS_FILENAME)


def build_empty_nav_tasks(map_name: str) -> dict:
    """构建空任务文件结构"""
    return {
        "version": "1.0",
        "map_name": map_name,
        "task_groups": [],
    }


def parse_nav_tasks_yaml(text: str) -> tuple:
    """
    解析任务 YAML 字符串

    Returns:
        (bool, dict|None, str): (是否成功, 数据, 错误信息)
    """
    try:
        data = yaml.safe_load(text)
    except Exception as e:
        return False, None, f"YAML解析失败: {e}"

    if data is None:
        data = {}

    if not isinstance(data, dict):
        return False, None, "任务文件顶层必须是字典"

    return True, data, ""


def normalize_nav_tasks(map_name: str, data: dict) -> tuple:
    """
    规范化任务结构（补齐必要字段）

    Returns:
        (bool, dict|None, str): (是否成功, 规范化数据, 错误信息)
    """
    if not isinstance(data, dict):
        return False, None, "任务数据必须是字典"

    if data.get("map_name") and data.get("map_name") != map_name:
        return False, None, f"map_name不一致：{data.get('map_name')} != {map_name}"

    data["map_name"] = map_name
    if not data.get("version"):
        data["version"] = "1.0"

    if "task_groups" not in data or data["task_groups"] is None:
        data["task_groups"] = []
    if not isinstance(data["task_groups"], list):
        return False, None, "task_groups必须为list"

    return True, data, ""


def dump_nav_tasks_yaml(data: dict) -> str:
    """将任务结构序列化为 YAML 字符串"""
    return yaml.safe_dump(
        data,
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
    )

