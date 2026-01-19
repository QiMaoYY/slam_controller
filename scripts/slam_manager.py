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
from datetime import datetime
from enum import Enum
from typing import Optional

import rospy

# Service消息类型（编译后自动生成）
from slam_controller.srv import (
    StartMapping, StartMappingResponse,
    StopMapping, StopMappingResponse,
    StartNavigation, StartNavigationResponse,
    StopNavigation, StopNavigationResponse,
    GetSlamStatus, GetSlamStatusResponse,
    ListMaps, ListMapsResponse,
    ProcessMap, ProcessMapResponse,
)


class SlamState(Enum):
    """SLAM系统状态枚举"""
    IDLE = "idle"                    # 空闲状态
    MAPPING = "mapping"              # 建图中
    LOCALIZING = "localizing"        # 定位中
    NAVIGATING = "navigating"        # 导航中
    PROCESSING = "processing"        # 地图处理中
    ERROR = "error"                  # 错误状态


class SlamManager:
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
        self.cfg = self._load_global_config()

        # 路径：优先配置文件，其次ROS参数兜底
        self.slam_ws = self._cfg_get('paths.slam_ws', '/media/data/slam_ws')
        self.kuavo_slam_path = rospy.get_param('~kuavo_slam_path', self._cfg_get('paths.kuavo_slam', '/media/data/slam_ws/src/kuavo_slam'))
        self.livox_ws = self._cfg_get('paths.livox_ws', '/media/data/livox_ws')
        self.faster_lio_pcd_dir = self._cfg_get('paths.faster_lio_pcd_dir', '/media/data/slam_ws/src/faster-lio/PCD')

        # 脚本路径
        self.mapping_script = self._cfg_get('scripts.start_mapping', os.path.join(self.kuavo_slam_path, 'scripts', 'start_mapping.sh'))
        self.nav_script = self._cfg_get('scripts.nav_run', os.path.join(self.kuavo_slam_path, 'scripts', 'nav_run.sh'))
        self.map_process_script = self._cfg_get('scripts.map_processing', '/media/data/slam_ws/src/pcd_cleaner/scripts/run_all.sh')

        # 地图目录与文件名约定
        self.map_root = self._cfg_get('maps.root', os.path.join(self.kuavo_slam_path, 'maps'))
        self.ori_pointcloud_name = self._cfg_get('maps.ori_pointcloud_name', 'pointcloud_original.pcd')
        self.nav_pointcloud_name = self._cfg_get('maps.nav_pointcloud_name', 'pointcloud.pcd')
        self.map2d_yaml_name = self._cfg_get('maps.map2d_yaml_name', 'map2d.yaml')
        self.map2d_pgm_name = self._cfg_get('maps.map2d_pgm_name', 'map2d.pgm')

        # 地图处理默认参数
        self.ceiling_threshold = str(self._cfg_get('map_processing.ceiling_threshold', 0.8))
        self.default_process_mode = int(self._cfg_get('map_processing.default_mode', 0))

        # 超时参数
        self.stop_mapping_wait_sec = int(self._cfg_get('timeouts.stop_mapping_wait_sec', 60))
        # 导航停止等待时间（默认复用 stop_mapping_wait_sec）
        self.stop_nav_wait_sec = int(self._cfg_get('timeouts.stop_nav_wait_sec', self.stop_mapping_wait_sec))
        self.process_map_wait_sec = int(self._cfg_get('timeouts.process_map_wait_sec', 0))
        
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

    @staticmethod
    def _bool_to_sh(v: bool) -> str:
        """将bool转换为shell脚本期望的 true/false 字符串。"""
        return 'true' if bool(v) else 'false'

    def _validate_nav_map_ready(self, map_name: str) -> tuple:
        """
        校验指定地图是否满足导航启动要求：
        - 目录存在
        - 必须包含 map2d.yaml / map2d.pgm / pointcloud.pcd
        """
        map_dir = os.path.join(self.map_root, map_name)
        if not os.path.isdir(map_dir):
            return False, f"地图目录不存在: {map_dir}"

        required = [
            os.path.join(map_dir, self.map2d_yaml_name),
            os.path.join(map_dir, self.map2d_pgm_name),
            os.path.join(map_dir, self.nav_pointcloud_name),
        ]
        missing = [p for p in required if not os.path.isfile(p)]
        if missing:
            return False, f"地图未准备好用于导航，缺少文件: {', '.join(missing)}"
        return True, ""

    def _cfg_get(self, key: str, default=None):
        """从self.cfg中读取点分key（如 paths.slam_ws），并做路径展开。"""
        cur = self.cfg or {}
        for part in key.split('.'):
            if not isinstance(cur, dict) or part not in cur:
                return default
            cur = cur[part]
        # 对字符串做环境变量/用户目录展开
        if isinstance(cur, str):
            return os.path.expandvars(os.path.expanduser(cur))
        return cur

    def _load_global_config(self) -> dict:
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
    
    def _handle_start_mapping(self, req: StartMapping) -> StartMappingResponse:
        """
        处理开始建图请求
        
        Args:
            req: StartMapping请求，包含need_calibration参数
            
        Returns:
            StartMappingResponse: 操作结果
        """
        rospy.loginfo(f"收到开始建图请求 (校准={req.need_calibration})")
        
        # 检查当前状态
        if self.state != SlamState.IDLE:
            msg = f"无法开始建图：当前状态为 {self.state.value}"
            rospy.logwarn(f"{msg}")
            return StartMappingResponse(success=False, message=msg)
        # 检查脚本是否存在
        if not os.path.exists(self.mapping_script):
            msg = f"建图脚本不存在: {self.mapping_script}"
            rospy.logerr(f"{msg}")
            return StartMappingResponse(success=False, message=msg)
        
        # 构建命令
        cmd = ['/bin/bash', self.mapping_script]
        if not req.need_calibration:
            cmd.append('--no-calib')
        
        try:
            # 启动建图脚本
            # 使用start_new_session=True创建新的进程组，方便后续发送信号
            with self._process_lock:
                self._current_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,  # 创建新的会话和进程组
                    universal_newlines=True,
                    bufsize=1,  # 行缓冲
                )
            
            # 启动输出监控线程
            self._output_thread = threading.Thread(
                target=self._monitor_process_output,
                args=(self._current_process, "建图"),
                daemon=True
            )
            self._output_thread.start()
            
            # 更新状态
            self.state = SlamState.MAPPING
            self._task_start_time = time.time()
            self._status_message = "建图进行中"
            
            calib_str = "含校准" if req.need_calibration else "跳过校准"
            msg = f"建图已启动 ({calib_str})"
            rospy.loginfo(f"{msg}")
            return StartMappingResponse(success=True, message=msg)
            
        except Exception as e:
            msg = f"启动建图脚本失败: {str(e)}"
            rospy.logerr(f"{msg}")
            self.state = SlamState.ERROR
            self._status_message = msg
            return StartMappingResponse(success=False, message=msg)

    def _handle_start_navigation(self, req: StartNavigation) -> StartNavigationResponse:
        """
        处理启动导航请求

        Args:
            req: StartNavigation请求，包含 map_name / enable_rviz / need_calibration
        """
        map_name = (req.map_name or '').strip()
        rospy.loginfo(
            f"收到启动导航请求 (map={map_name}, rviz={req.enable_rviz}, 校准={req.need_calibration})"
        )

        # 检查当前状态
        if self.state != SlamState.IDLE:
            msg = f"无法启动导航：当前状态为 {self.state.value}"
            rospy.logwarn(f"{msg}")
            return StartNavigationResponse(success=False, message=msg)

        # 地图名合法性校验
        valid, err = self._validate_map_name(map_name)
        if not valid:
            return StartNavigationResponse(success=False, message=f"地图名非法: {err}")

        # 地图是否可用于导航
        ok, msg = self._validate_nav_map_ready(map_name)
        if not ok:
            rospy.logwarn(msg)
            return StartNavigationResponse(success=False, message=msg)

        # 检查脚本是否存在
        if not os.path.isfile(self.nav_script):
            msg = f"导航脚本不存在: {self.nav_script}"
            rospy.logerr(f"{msg}")
            return StartNavigationResponse(success=False, message=msg)

        # 构建命令：nav_run.sh <MAP_NAME> <MAP_ROOT> <RVIZ> <CALIB>
        cmd = [
            '/bin/bash',
            self.nav_script,
            map_name,
            self.map_root,
            self._bool_to_sh(req.enable_rviz),
            self._bool_to_sh(req.need_calibration),
        ]

        try:
            with self._nav_process_lock:
                if self._nav_process is not None and self._nav_process.poll() is None:
                    return StartNavigationResponse(success=False, message="已有导航任务在运行中")

                self._nav_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    universal_newlines=True,
                    bufsize=1,
                )

            # 启动输出监控线程
            threading.Thread(
                target=self._monitor_process_output,
                args=(self._nav_process, f"导航:{map_name}"),
                daemon=True
            ).start()

            # 更新状态
            self.state = SlamState.NAVIGATING
            self._task_start_time = time.time()
            self._status_message = f"导航进行中: {map_name} (rviz={req.enable_rviz}, calib={req.need_calibration})"

            msg = f"导航已启动: {map_name}"
            rospy.loginfo(f"{msg}")
            return StartNavigationResponse(success=True, message=msg)

        except Exception as e:
            with self._nav_process_lock:
                self._nav_process = None
            msg = f"启动导航脚本失败: {str(e)}"
            rospy.logerr(f"{msg}")
            self.state = SlamState.ERROR
            self._status_message = msg
            return StartNavigationResponse(success=False, message=msg)
    
    def _validate_map_name(self, map_name: str) -> tuple:
        """
        验证地图名称的合法性
        
        Args:
            map_name: 地图名称
            
        Returns:
            (bool, str): (是否合法, 错误信息)
        """
        if not map_name:
            return False, "地图名称不能为空"
        
        # 检查非法字符
        import re
        if not re.match(r'^[a-zA-Z0-9_\-]+$', map_name):
            return False, "地图名称只能包含字母、数字、下划线和横线"
        
        # 检查长度
        if len(map_name) > 50:
            return False, "地图名称过长（最多50个字符）"
        
        return True, ""
    
    def _save_map(self, map_name: str) -> tuple:
        """
        保存地图文件
        
        Args:
            map_name: 地图名称
            
        Returns:
            (bool, str): (是否成功, 消息)
        """
        try:
            # 验证地图名
            valid, error_msg = self._validate_map_name(map_name)
            if not valid:
                return False, error_msg
            
            # 路径定义
            maps_dir = os.path.join(self.kuavo_slam_path, 'maps')
            map_dir = os.path.join(maps_dir, map_name)
            
            # 保存地图必须确保目录不存在（避免覆盖）
            if os.path.exists(map_dir):
                return False, f"地图 '{map_name}' 已存在"
            faster_lio_pcd_dir = self.faster_lio_pcd_dir
            
            # 检查faster-lio PCD目录是否存在
            if not os.path.exists(faster_lio_pcd_dir):
                return False, f"未找到faster-lio PCD目录: {faster_lio_pcd_dir}"
            
            # 查找PCD文件
            import glob
            pcd_files = glob.glob(os.path.join(faster_lio_pcd_dir, '*.pcd'))
            
            if not pcd_files:
                return False, "未找到PCD地图文件，可能建图时间太短"
            
            # 创建地图目录
            os.makedirs(map_dir, exist_ok=True)
            rospy.loginfo(f"创建地图目录: {map_dir}")
            
            # 移动并重命名PCD文件
            saved_files = []
            for pcd_file in pcd_files:
                src_filename = os.path.basename(pcd_file)
                # 重命名为 pointcloud_original.pcd
                dst_filename = self.ori_pointcloud_name
                dst_path = os.path.join(map_dir, dst_filename)
                
                # 移动文件
                import shutil
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
    
    def _handle_stop_mapping(self, req: StopMapping) -> StopMappingResponse:
        """
        处理停止建图请求
        
        向建图脚本发送SIGINT信号，触发脚本的trap handler执行清理操作
        如果需要保存地图，则在脚本退出后执行地图保存
        
        Args:
            req: StopMapping请求，包含save_map和map_name参数
            
        Returns:
            StopMappingResponse: 操作结果
        """
        map_name = (req.map_name or '').strip()
        rospy.loginfo(f"收到停止建图请求 (保存={req.save_map}, 地图名={map_name})")

        # 如果需要保存地图，先做“合法性 + 重名”预检查
        # 重要：若不通过，必须直接返回失败且不得停止建图进程（避免丢图）
        if req.save_map:
            valid, error_msg = self._validate_map_name(map_name)
            if not valid:
                rospy.logwarn(f"地图名验证失败: {error_msg}，将保持建图继续运行")
                return StopMappingResponse(success=False, message=f"地图名非法: {error_msg}")

            # 重名检查：maps.root/<map_name> 已存在则拒绝
            map_dir = os.path.join(self.map_root, map_name)
            if os.path.exists(map_dir):
                msg = f"地图名重复：'{map_name}' 已存在，将保持建图继续运行"
                rospy.logwarn(msg)
                return StopMappingResponse(success=False, message=msg)
        
        # 检查当前状态
        if self.state != SlamState.MAPPING:
            msg = f"无法停止建图：当前状态为 {self.state.value}，不是建图状态"
            rospy.logwarn(f"{msg}")
            return StopMappingResponse(success=False, message=msg)
        
        try:
            with self._process_lock:
                if self._current_process is None:
                    msg = "建图进程不存在"
                    rospy.logwarn(f"{msg}")
                    self.state = SlamState.IDLE
                    return StopMappingResponse(success=False, message=msg)
                
                # 获取进程组ID并发送SIGINT信号
                # 这等同于在终端中按Ctrl+C
                try:
                    pgid = os.getpgid(self._current_process.pid)
                    rospy.loginfo(f"向进程组 {pgid} 发送SIGINT信号（触发脚本清理和保存地图）...")
                    os.killpg(pgid, signal.SIGINT)
                except ProcessLookupError:
                    rospy.logwarn("进程已不存在")
                    self._current_process = None
                    self.state = SlamState.IDLE
                    return StopMappingResponse(success=True, message="建图进程已结束")
            
            # 启动异步等待线程，避免阻塞服务响应
            def wait_for_process():
                """异步等待进程退出并处理地图保存"""
                timeout = 60
                start_wait = time.time()
                
                # 等待进程退出
                while True:
                    with self._process_lock:
                        if self._current_process is None:
                            rospy.loginfo("建图进程已被清理")
                            break
                        ret = self._current_process.poll()
                        if ret is not None:
                            rospy.loginfo(f"建图脚本已退出，返回码: {ret}")
                            self._current_process = None
                            break
                    
                    if time.time() - start_wait > self.stop_mapping_wait_sec:
                        rospy.logwarn("等待建图脚本退出超时，强制终止...")
                        with self._process_lock:
                            if self._current_process:
                                try:
                                    pgid = os.getpgid(self._current_process.pid)
                                    os.killpg(pgid, signal.SIGKILL)
                                    rospy.loginfo(f"已强制终止进程组 {pgid}")
                                except:
                                    pass
                                self._current_process = None
                        break
                    
                    time.sleep(0.5)
                
                
                # 处理地图保存
                if req.save_map:
                    rospy.loginfo(f"开始保存地图: {map_name}")
                    success, msg = self._save_map(map_name)
                    if success:
                        self._status_message = f"建图完成，地图已保存: {map_name}"
                    else:
                        self._status_message = f"建图完成，但保存地图失败: {msg}"
                        rospy.logerr(f"{self._status_message}")
                else:
                    rospy.loginfo("不保存地图，跳过")
                    self._status_message = "建图完成（未保存地图）"
                
                # 更新状态
                self.state = SlamState.IDLE
                self._task_start_time = None
            
            # 启动后台等待线程
            wait_thread = threading.Thread(target=wait_for_process, daemon=True)
            wait_thread.start()
            
            # 立即返回响应，不等待进程退出
            if req.save_map:
                msg = f"停止建图指令已发送，将保存地图到: {map_name}"
            else:
                msg = "停止建图指令已发送（不保存地图）"
            rospy.loginfo(f"{msg}")
            return StopMappingResponse(success=True, message=msg)
            
        except Exception as e:
            msg = f"停止建图时发生错误: {str(e)}"
            rospy.logerr(f"{msg}")
            # 确保状态被重置
            with self._process_lock:
                self._current_process = None
            self.state = SlamState.ERROR
            self._status_message = msg
            return StopMappingResponse(success=False, message=msg)
        
    def _handle_stop_navigation(self, req: StopNavigation) -> StopNavigationResponse:
        """
        处理停止导航请求：向导航脚本所在进程组发送 SIGINT（等同Ctrl+C）
        """
        rospy.loginfo("收到停止导航请求")

        if self.state != SlamState.NAVIGATING:
            msg = f"无法停止导航：当前状态为 {self.state.value}，不是导航状态"
            rospy.logwarn(f"{msg}")
            return StopNavigationResponse(success=False, message=msg)

        try:
            with self._nav_process_lock:
                if self._nav_process is None:
                    msg = "导航进程不存在"
                    rospy.logwarn(msg)
                    self.state = SlamState.IDLE
                    return StopNavigationResponse(success=False, message=msg)

                try:
                    pgid = os.getpgid(self._nav_process.pid)
                    rospy.loginfo(f"向导航进程组 {pgid} 发送SIGINT信号...")
                    os.killpg(pgid, signal.SIGINT)
                except ProcessLookupError:
                    rospy.logwarn("导航进程已不存在")
                    self._nav_process = None
                    self.state = SlamState.IDLE
                    return StopNavigationResponse(success=True, message="导航进程已结束")

            # 异步等待退出并清理
            def wait_for_nav_exit():
                start_wait = time.time()
                while True:
                    with self._nav_process_lock:
                        if self._nav_process is None:
                            break
                        ret = self._nav_process.poll()
                        if ret is not None:
                            rospy.loginfo(f"导航脚本已退出，返回码: {ret}")
                            self._nav_process = None
                            break

                    if time.time() - start_wait > self.stop_nav_wait_sec:
                        rospy.logwarn("等待导航脚本退出超时，强制终止...")
                        with self._nav_process_lock:
                            if self._nav_process:
                                try:
                                    pgid = os.getpgid(self._nav_process.pid)
                                    os.killpg(pgid, signal.SIGKILL)
                                    rospy.loginfo(f"已强制终止导航进程组 {pgid}")
                                except Exception:
                                    pass
                                self._nav_process = None
                        break

                    time.sleep(0.5)

                # 更新状态
                if self.state == SlamState.NAVIGATING:
                    self.state = SlamState.IDLE
                    self._task_start_time = None
                    self._status_message = "导航已停止"

            threading.Thread(target=wait_for_nav_exit, daemon=True).start()

            msg = "停止导航指令已发送"
            rospy.loginfo(msg)
            return StopNavigationResponse(success=True, message=msg)

        except Exception as e:
            msg = f"停止导航时发生错误: {str(e)}"
            rospy.logerr(msg)
            with self._nav_process_lock:
                self._nav_process = None
            self.state = SlamState.ERROR
            self._status_message = msg
            return StopNavigationResponse(success=False, message=msg)
    
    def _handle_get_status(self, req: GetSlamStatus) -> GetSlamStatusResponse:
        """
        处理状态查询请求
        
        Args:
            req: GetSlamStatus请求
            
        Returns:
            GetSlamStatusResponse: 当前系统状态
        """
        return GetSlamStatusResponse(
            status=self.state.value,
            message=self._status_message,
            uptime_sec=self._get_uptime()
        )

    def _handle_list_maps(self, req: ListMaps) -> ListMapsResponse:
        """
        查询地图列表，并判断每个地图是否可用于导航。

        判定规则（扫描 kuavo_slam/maps 下的子目录）：
        - 若存在 pointcloud_original.pcd => 认为该目录为“有效地图”
        - 若同时存在 map2d.yaml、map2d.pgm、pointcloud.pcd => 认为“可用于导航”
        """
        maps_dir = self.map_root
        if not os.path.isdir(maps_dir):
            msg = f"maps目录不存在: {maps_dir}"
            rospy.logwarn(f"{msg}")
            return ListMapsResponse(success=False, message=msg, map_names=[], nav_ready=[])

        try:
            map_names = []
            nav_ready = []
            map_paths = []
            created_at = []
            ori_pointcloud_bytes = []
            nav_pointcloud_bytes = []

            for entry in sorted(os.listdir(maps_dir)):
                subdir = os.path.join(maps_dir, entry)
                if not os.path.isdir(subdir):
                    continue

                # 有效地图判定
                ori_pcd_path = os.path.join(subdir, self.ori_pointcloud_name)
                valid_flag = os.path.isfile(ori_pcd_path)
                if not valid_flag:
                    continue

                # 可导航判定
                nav_pcd_path = os.path.join(subdir, self.nav_pointcloud_name)
                nav_flag = (
                    os.path.isfile(os.path.join(subdir, self.map2d_yaml_name)) and
                    os.path.isfile(os.path.join(subdir, self.map2d_pgm_name)) and
                    os.path.isfile(nav_pcd_path)
                )

                map_names.append(entry)
                nav_ready.append(bool(nav_flag))
                map_paths.append(subdir)

                # 地图创建时间：取原始点云文件mtime
                try:
                    ts = os.path.getmtime(ori_pcd_path)
                    created_at.append(datetime.fromtimestamp(ts).strftime('%Y%m%d_%H%M%S'))
                except Exception:
                    created_at.append('')

                # 原始点云大小
                try:
                    ori_pointcloud_bytes.append(int(os.path.getsize(ori_pcd_path)))
                except Exception:
                    ori_pointcloud_bytes.append(0)

                # 导航点云大小（仅对可导航地图有效）
                if nav_flag:
                    try:
                        nav_pointcloud_bytes.append(int(os.path.getsize(nav_pcd_path)))
                    except Exception:
                        nav_pointcloud_bytes.append(0)
                else:
                    nav_pointcloud_bytes.append(0)

            msg = f"已发现有效地图 {len(map_names)} 个"
            return ListMapsResponse(
                success=True,
                message=msg,
                map_names=map_names,
                nav_ready=nav_ready,
                map_paths=map_paths,
                created_at=created_at,
                ori_pointcloud_bytes=ori_pointcloud_bytes,
                nav_pointcloud_bytes=nav_pointcloud_bytes,
            )
        except Exception as e:
            msg = f"扫描maps目录失败: {e}"
            rospy.logerr(f"{msg}")
            return ListMapsResponse(
                success=False,
                message=msg,
                map_names=[],
                nav_ready=[],
                map_paths=[],
                created_at=[],
                ori_pointcloud_bytes=[],
                nav_pointcloud_bytes=[],
            )

    def _handle_process_map(self, req: ProcessMap) -> ProcessMapResponse:
        """
        地图处理服务回调：调用 pcd_cleaner/scripts/run_all.sh

        run_all.sh 参数：
          [地图根目录] [地图名] [屋顶高度阈值] [运行方式]

        其中地图根目录、屋顶高度阈值由配置文件提供；请求只传 map_name 和 mode。
        """
        map_name = (req.map_name or '').strip()
        mode = int(req.mode)

        # map_name 合法性与存在性检查（复用命名规则，但这里要求“已存在地图”）
        valid, err = self._validate_map_name(map_name)
        if not valid:
            return ProcessMapResponse(success=False, message=f"地图名非法: {err}")

        map_dir = os.path.join(self.map_root, map_name)
        if not os.path.isdir(map_dir):
            return ProcessMapResponse(success=False, message=f"地图目录不存在: {map_dir}")

        ori_pcd = os.path.join(map_dir, 'pointcloud_original.pcd')
        if not os.path.isfile(ori_pcd):
            return ProcessMapResponse(success=False, message=f"缺少原始点云: {ori_pcd}")

        if mode not in (0, 1, 2, 3, 4):
            return ProcessMapResponse(success=False, message="mode非法，仅支持0/1/2/3/4")

        if not os.path.isfile(self.map_process_script):
            return ProcessMapResponse(success=False, message=f"地图处理脚本不存在: {self.map_process_script}")

        # 防止并发执行
        with self._map_process_lock:
            if self._map_process is not None and self._map_process.poll() is None:
                return ProcessMapResponse(success=False, message="已有地图处理任务在运行中，请稍后再试")

            cmd = [
                '/bin/bash',
                self.map_process_script,
                self.map_root,
                map_name,
                str(self.ceiling_threshold),
                str(mode),
            ]

            try:
                rospy.loginfo(f"开始地图处理: map={map_name}, mode={mode}")
                rospy.loginfo(f"执行命令: {' '.join(cmd)}")

                self._map_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    universal_newlines=True,
                    bufsize=1,
                )

                # 输出监控线程（复用同一个monitor函数）
                threading.Thread(
                    target=self._monitor_process_output,
                    args=(self._map_process, f"地图处理:{map_name}"),
                    daemon=True
                ).start()

                # 更新状态
                self.state = SlamState.PROCESSING
                self._task_start_time = time.time()
                self._status_message = f"地图处理中: {map_name} (mode={mode})"

            except Exception as e:
                self._map_process = None
                self.state = SlamState.ERROR
                self._status_message = f"启动地图处理失败: {e}"
                return ProcessMapResponse(success=False, message=self._status_message)

        # 异步等待结束并恢复状态
        def _wait_done():
            proc = None
            with self._map_process_lock:
                proc = self._map_process
            if proc is None:
                return
            ret = proc.wait()
            rospy.loginfo(f"地图处理结束: map={map_name}, mode={mode}, code={ret}")
            with self._map_process_lock:
                self._map_process = None
            # 恢复状态（如果此时还处于processing）
            if self.state == SlamState.PROCESSING:
                self.state = SlamState.IDLE
                self._task_start_time = None
                if ret == 0:
                    self._status_message = f"地图处理完成: {map_name}"
                else:
                    self._status_message = f"地图处理失败(code={ret}): {map_name}"

        threading.Thread(target=_wait_done, daemon=True).start()

        return ProcessMapResponse(success=True, message=f"已启动地图处理: {map_name} (mode={mode})")
    
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
                    clean_line = self._strip_ansi(line.rstrip())
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
    
    @staticmethod
    def _strip_ansi(text: str) -> str:
        """移除ANSI转义序列（颜色代码等）"""
        import re
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        return ansi_escape.sub('', text)
    
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
