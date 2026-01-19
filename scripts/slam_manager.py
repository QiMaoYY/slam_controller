#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kuavo SLAM 总控节点
功能：管理SLAM系统状态，提供ROS Service接口控制建图、定位、导航等功能

作者：Kuavo Team
日期：2026-01-05

使用方法：
    1. 启动节点：roslaunch kuavo_slam slam_manager.launch
    2. 或直接运行：python3 slam_manager.py

服务接口：
    - /slam_manager/start_mapping  开始建图
    - /slam_manager/stop_mapping   停止建图
    - /slam_manager/start_navigation 启动导航
    - /slam_manager/stop_navigation  停止导航
    - /slam_manager/get_status     获取系统状态
"""

import os
import signal
import subprocess
import threading
import time
from typing import Optional

import rospy

# Service消息类型（编译后自动生成）
from slam_controller.srv import (
    StartMapping,
    StopMapping,
    StartNavigation,
    StopNavigation,
    GetSlamStatus,
    ListMaps,
    ProcessMap,
)

from srv_handlers import SlamServiceHandlers
from utils import SlamState, cfg_get, load_global_config, strip_ansi

class SlamManager(SlamServiceHandlers):
    """
    SLAM系统总控管理器
    
    负责：
    1. 维护系统状态
    2. 管理子进程（建图、导航脚本）
    3. 提供ROS Service接口
    """
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('slam_manager', anonymous=False)

        # 全局配置（从配置文件读取，可被ROS参数覆盖）
        self.cfg = load_global_config()

        # 路径：优先配置文件，其次ROS参数兜底
        self.slam_ws = cfg_get(self.cfg, 'paths.slam_ws', '/media/data/slam_ws')
        self.kuavo_slam_path = rospy.get_param(
            '~kuavo_slam_path',
            cfg_get(self.cfg, 'paths.kuavo_slam', '/media/data/slam_ws/src/kuavo_slam')
        )
        self.livox_ws = cfg_get(self.cfg, 'paths.livox_ws', '/media/data/livox_ws')
        self.faster_lio_pcd_dir = cfg_get(self.cfg, 'paths.faster_lio_pcd_dir', '/media/data/slam_ws/src/faster-lio/PCD')

        # 脚本路径
        self.mapping_script = cfg_get(self.cfg, 'scripts.start_mapping', os.path.join(self.kuavo_slam_path, 'scripts', 'start_mapping.sh'))
        self.nav_script = cfg_get(self.cfg, 'scripts.nav_run', os.path.join(self.kuavo_slam_path, 'scripts', 'nav_run.sh'))
        self.map_process_script = cfg_get(self.cfg, 'scripts.map_processing', '/media/data/slam_ws/src/pcd_cleaner/scripts/run_all.sh')

        # 地图目录与文件名约定
        self.map_root = cfg_get(self.cfg, 'maps.root', os.path.join(self.kuavo_slam_path, 'maps'))
        self.ori_pointcloud_name = cfg_get(self.cfg, 'maps.ori_pointcloud_name', 'pointcloud_original.pcd')
        self.nav_pointcloud_name = cfg_get(self.cfg, 'maps.nav_pointcloud_name', 'pointcloud.pcd')
        self.map2d_yaml_name = cfg_get(self.cfg, 'maps.map2d_yaml_name', 'map2d.yaml')
        self.map2d_pgm_name = cfg_get(self.cfg, 'maps.map2d_pgm_name', 'map2d.pgm')

        # 地图处理默认参数
        self.ceiling_threshold = str(cfg_get(self.cfg, 'map_processing.ceiling_threshold', 0.8))
        self.default_process_mode = int(cfg_get(self.cfg, 'map_processing.default_mode', 0))

        # 超时参数
        self.stop_mapping_wait_sec = int(cfg_get(self.cfg, 'timeouts.stop_mapping_wait_sec', 60))
        # 导航停止等待时间（默认复用 stop_mapping_wait_sec）
        self.stop_nav_wait_sec = int(cfg_get(self.cfg, 'timeouts.stop_nav_wait_sec', self.stop_mapping_wait_sec))
        self.process_map_wait_sec = int(cfg_get(self.cfg, 'timeouts.process_map_wait_sec', 0))
        
        # 系统状态
        self._state = SlamState.IDLE
        self._state_lock = threading.Lock()
        self._task_start_time: Optional[float] = None
        self._status_message = "系统就绪"
        
        # 子进程管理
        self._current_process: Optional[subprocess.Popen] = None
        self._process_lock = threading.Lock()
        self._output_thread: Optional[threading.Thread] = None

        # 地图处理子进程管理
        self._map_process: Optional[subprocess.Popen] = None
        self._map_process_lock = threading.Lock()

        # 导航子进程管理（与建图分开，便于独立停止/回收）
        self._nav_process: Optional[subprocess.Popen] = None
        self._nav_process_lock = threading.Lock()
        
        # 注册ROS Service
        self._setup_services()
        
        # 注册节点关闭回调
        rospy.on_shutdown(self._on_shutdown)
        
        rospy.loginfo(f"建图脚本路径: {self.mapping_script}")
        rospy.loginfo(f"导航脚本路径: {self.nav_script}")
        rospy.loginfo(f"地图处理脚本路径: {self.map_process_script}")
        rospy.loginfo(f"地图根目录(map_root): {self.map_root}")
        rospy.loginfo(f"屋顶阈值(ceiling_threshold): {self.ceiling_threshold}")
        rospy.loginfo(f"faster-lio PCD目录: {self.faster_lio_pcd_dir}")
        rospy.loginfo("初始化完成，等待服务调用...")
    
    def _setup_services(self):
        """注册ROS Service服务"""
        # 建图相关服务
        self._srv_start_mapping = rospy.Service(
            '~start_mapping', 
            StartMapping, 
            self._handle_start_mapping
        )
        self._srv_stop_mapping = rospy.Service(
            '~stop_mapping', 
            StopMapping, 
            self._handle_stop_mapping
        )

        # 导航相关服务
        self._srv_start_navigation = rospy.Service(
            '~start_navigation',
            StartNavigation,
            self._handle_start_navigation
        )
        self._srv_stop_navigation = rospy.Service(
            '~stop_navigation',
            StopNavigation,
            self._handle_stop_navigation
        )
        
        # 状态查询服务
        self._srv_get_status = rospy.Service(
            '~get_status', 
            GetSlamStatus, 
            self._handle_get_status
        )

        # 地图列表查询服务
        self._srv_list_maps = rospy.Service(
            '~list_maps',
            ListMaps,
            self._handle_list_maps
        )

        # 地图处理服务
        self._srv_process_map = rospy.Service(
            '~process_map',
            ProcessMap,
            self._handle_process_map
        )
        
        rospy.loginfo("服务已注册")

    @property
    def state(self) -> SlamState:
        """获取当前状态（线程安全）"""
        with self._state_lock:
            return self._state
    
    @state.setter
    def state(self, new_state: SlamState):
        """设置当前状态（线程安全）"""
        with self._state_lock:
            old_state = self._state
            self._state = new_state
            if old_state != new_state:
                rospy.loginfo(f"状态变更: {old_state.value} -> {new_state.value}")
    
    def _get_uptime(self) -> int:
        """获取当前任务运行时间（秒）"""
        if self._task_start_time is None:
            return 0
        return int(time.time() - self._task_start_time)

    # Service 回调实现已拆分到 srv_handlers.SlamServiceHandlers
    
    def _monitor_process_output(self, process: subprocess.Popen, task_name: str):
        """
        监控子进程输出并记录到ROS日志
        
        Args:
            process: 子进程对象
            task_name: 任务名称（用于日志前缀）
        """
        try:
            for line in iter(process.stdout.readline, ''):
                if line:
                    # 移除ANSI颜色代码以便日志更清晰
                    clean_line = strip_ansi(line.rstrip())
                    if clean_line:
                        rospy.loginfo(f"[{task_name}] {clean_line}")
                        
                # 检查进程是否已结束
                if process.poll() is not None:
                    break
                    
        except Exception as e:
            rospy.logwarn(f"监控{task_name}输出时发生错误: {e}")
        
        finally:
            # 进程结束，检查是否需要更新状态
            rospy.loginfo(f"{task_name}进程输出监控结束")
            
            # 如果进程意外结束（不是通过stop命令），更新状态
            with self._process_lock:
                if self._current_process == process:
                    ret = process.poll()
                    if ret is not None:
                        self._current_process = None
                        if self.state == SlamState.MAPPING:
                            self.state = SlamState.IDLE
                            self._task_start_time = None
                            if ret == 0:
                                self._status_message = f"{task_name}正常完成"
                            else:
                                self._status_message = f"{task_name}异常退出 (code={ret})"
                                rospy.logwarn(f"{self._status_message}")

            # 导航进程退出后的状态恢复
            with self._nav_process_lock:
                if self._nav_process == process:
                    ret = process.poll()
                    if ret is not None:
                        self._nav_process = None
                        if self.state == SlamState.NAVIGATING:
                            self.state = SlamState.IDLE
                            self._task_start_time = None
                            if ret == 0:
                                self._status_message = f"{task_name}正常完成"
                            else:
                                self._status_message = f"{task_name}异常退出 (code={ret})"
                                rospy.logwarn(f"{self._status_message}")
    
    def _on_shutdown(self):
        """节点关闭时的清理操作"""
        rospy.loginfo("节点正在关闭，清理资源...")
        
        # 如果有正在运行的进程，发送终止信号
        with self._process_lock:
            if self._current_process is not None:
                try:
                    pgid = os.getpgid(self._current_process.pid)
                    rospy.loginfo(f"终止运行中的进程组 {pgid}...")
                    os.killpg(pgid, signal.SIGINT)
                    
                    # 等待一段时间让进程完成清理
                    time.sleep(3)
                    
                    # 如果还没退出，强制终止
                    if self._current_process.poll() is None:
                        os.killpg(pgid, signal.SIGKILL)
                        
                except ProcessLookupError:
                    pass
                except Exception as e:
                    rospy.logwarn(f"清理进程时发生错误: {e}")

        # 清理导航进程
        with self._nav_process_lock:
            if self._nav_process is not None:
                try:
                    pgid = os.getpgid(self._nav_process.pid)
                    rospy.loginfo(f"终止运行中的导航进程组 {pgid}...")
                    os.killpg(pgid, signal.SIGINT)
                    time.sleep(3)
                    if self._nav_process.poll() is None:
                        os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except Exception as e:
                    rospy.logwarn(f"清理导航进程时发生错误: {e}")

        # 清理地图处理进程
        with self._map_process_lock:
            if self._map_process is not None:
                try:
                    pgid = os.getpgid(self._map_process.pid)
                    rospy.loginfo(f"终止运行中的地图处理进程组 {pgid}...")
                    os.killpg(pgid, signal.SIGINT)
                    time.sleep(3)
                    if self._map_process.poll() is None:
                        os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except Exception as e:
                    rospy.logwarn(f"清理地图处理进程时发生错误: {e}")
        
        rospy.loginfo("清理完成")
    
    def spin(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            # 检查子进程是否意外退出
            with self._process_lock:
                if self._current_process is not None:
                    ret = self._current_process.poll()
                    if ret is not None and self.state == SlamState.MAPPING:
                        rospy.logwarn(f"检测到建图进程已退出 (code={ret})")
                        self._current_process = None
                        self.state = SlamState.IDLE
                        self._task_start_time = None

            # 检查导航进程是否意外退出
            with self._nav_process_lock:
                if self._nav_process is not None:
                    ret = self._nav_process.poll()
                    if ret is not None and self.state == SlamState.NAVIGATING:
                        rospy.logwarn(f"检测到导航进程已退出 (code={ret})")
                        self._nav_process = None
                        self.state = SlamState.IDLE
                        self._task_start_time = None
            
            rate.sleep()


def main():
    """主函数入口"""
    try:
        manager = SlamManager()
        manager.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生未处理的异常: {e}")
        raise


if __name__ == '__main__':
    main()
