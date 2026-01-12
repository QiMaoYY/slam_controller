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
    GetSlamStatus, GetSlamStatusResponse,
    ListMaps, ListMapsResponse,
)


class SlamState(Enum):
    """SLAM系统状态枚举"""
    IDLE = "idle"                    # 空闲状态
    MAPPING = "mapping"              # 建图中
    LOCALIZING = "localizing"        # 定位中
    NAVIGATING = "navigating"        # 导航中
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
        
        # 获取kuavo_slam包路径（用于定位建图和导航脚本）
        self.kuavo_slam_path = rospy.get_param(
            '~kuavo_slam_path', 
            '/media/data/slam_ws/src/kuavo_slam'
        )
        self.mapping_script = os.path.join(
            self.kuavo_slam_path, 'scripts', 'start_mapping.sh'
        )
        self.nav_script = os.path.join(
            self.kuavo_slam_path, 'scripts', 'nav_run.sh'
        )
        
        # 系统状态
        self._state = SlamState.IDLE
        self._state_lock = threading.Lock()
        self._task_start_time: Optional[float] = None
        self._status_message = "系统就绪"
        
        # 子进程管理
        self._current_process: Optional[subprocess.Popen] = None
        self._process_lock = threading.Lock()
        self._output_thread: Optional[threading.Thread] = None
        
        # 注册ROS Service
        self._setup_services()
        
        # 注册节点关闭回调
        rospy.on_shutdown(self._on_shutdown)
        
        rospy.loginfo(f"[SlamManager] 建图脚本路径: {self.mapping_script}")
        rospy.loginfo(f"[SlamManager] 导航脚本路径: {self.nav_script}")
        rospy.loginfo("[SlamManager] 初始化完成，等待服务调用...")
    
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
        
        rospy.loginfo("[SlamManager] 服务已注册")
    
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
                rospy.loginfo(f"[SlamManager] 状态变更: {old_state.value} -> {new_state.value}")
    
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
        rospy.loginfo(f"[SlamManager] 收到开始建图请求 (校准={req.need_calibration})")
        
        # 检查当前状态
        if self.state != SlamState.IDLE:
            msg = f"无法开始建图：当前状态为 {self.state.value}"
            rospy.logwarn(f"[SlamManager] {msg}")
            return StartMappingResponse(success=False, message=msg)
        
        # 检查脚本是否存在
        if not os.path.exists(self.mapping_script):
            msg = f"建图脚本不存在: {self.mapping_script}"
            rospy.logerr(f"[SlamManager] {msg}")
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
            rospy.loginfo(f"[SlamManager] {msg}")
            return StartMappingResponse(success=True, message=msg)
            
        except Exception as e:
            msg = f"启动建图脚本失败: {str(e)}"
            rospy.logerr(f"[SlamManager] {msg}")
            self.state = SlamState.ERROR
            self._status_message = msg
            return StartMappingResponse(success=False, message=msg)
    
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
        
        # 检查是否已存在
        maps_dir = os.path.join(self.kuavo_slam_path, 'maps')
        map_dir = os.path.join(maps_dir, map_name)
        
        if os.path.exists(map_dir):
            return False, f"地图 '{map_name}' 已存在"
        
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
            faster_lio_pcd_dir = '/media/data/slam_ws/src/faster-lio/PCD'
            
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
            rospy.loginfo(f"[SlamManager] 创建地图目录: {map_dir}")
            
            # 移动并重命名PCD文件
            saved_files = []
            for pcd_file in pcd_files:
                src_filename = os.path.basename(pcd_file)
                # 重命名为 pointcloud_original.pcd
                dst_filename = f"pointcloud_original.pcd"
                dst_path = os.path.join(map_dir, dst_filename)
                
                # 移动文件
                import shutil
                shutil.move(pcd_file, dst_path)
                saved_files.append(dst_filename)
                rospy.loginfo(f"[SlamManager] 已保存: {src_filename} -> {dst_path}")
            
            msg = f"地图已保存到: {map_dir}/ (文件: {', '.join(saved_files)})"
            rospy.loginfo(f"[SlamManager] {msg}")
            return True, msg
            
        except Exception as e:
            error_msg = f"保存地图时发生错误: {str(e)}"
            rospy.logerr(f"[SlamManager] {error_msg}")
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
        rospy.loginfo(f"[SlamManager] 收到停止建图请求 (保存={req.save_map}, 地图名={req.map_name})")
        
        # 如果需要保存地图，先验证地图名
        if req.save_map:
            valid, error_msg = self._validate_map_name(req.map_name)
            if not valid:
                rospy.logwarn(f"[SlamManager] 地图名验证失败: {error_msg}")
                return StopMappingResponse(success=False, message=f"地图名非法: {error_msg}")
        
        # 检查当前状态
        if self.state != SlamState.MAPPING:
            msg = f"无法停止建图：当前状态为 {self.state.value}，不是建图状态"
            rospy.logwarn(f"[SlamManager] {msg}")
            return StopMappingResponse(success=False, message=msg)
        
        try:
            with self._process_lock:
                if self._current_process is None:
                    msg = "建图进程不存在"
                    rospy.logwarn(f"[SlamManager] {msg}")
                    self.state = SlamState.IDLE
                    return StopMappingResponse(success=False, message=msg)
                
                # 获取进程组ID并发送SIGINT信号
                # 这等同于在终端中按Ctrl+C
                try:
                    pgid = os.getpgid(self._current_process.pid)
                    rospy.loginfo(f"[SlamManager] 向进程组 {pgid} 发送SIGINT信号（触发脚本清理和保存地图）...")
                    os.killpg(pgid, signal.SIGINT)
                except ProcessLookupError:
                    rospy.logwarn("[SlamManager] 进程已不存在")
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
                            rospy.loginfo("[SlamManager] 建图进程已被清理")
                            break
                        ret = self._current_process.poll()
                        if ret is not None:
                            rospy.loginfo(f"[SlamManager] 建图脚本已退出，返回码: {ret}")
                            self._current_process = None
                            break
                    
                    if time.time() - start_wait > timeout:
                        rospy.logwarn("[SlamManager] 等待建图脚本退出超时，强制终止...")
                        with self._process_lock:
                            if self._current_process:
                                try:
                                    pgid = os.getpgid(self._current_process.pid)
                                    os.killpg(pgid, signal.SIGKILL)
                                    rospy.loginfo(f"[SlamManager] 已强制终止进程组 {pgid}")
                                except:
                                    pass
                                self._current_process = None
                        break
                    
                    time.sleep(0.5)
                
                
                # 处理地图保存
                if req.save_map:
                    rospy.loginfo(f"[SlamManager] 开始保存地图: {req.map_name}")
                    success, msg = self._save_map(req.map_name)
                    if success:
                        self._status_message = f"建图完成，地图已保存: {req.map_name}"
                    else:
                        self._status_message = f"建图完成，但保存地图失败: {msg}"
                        rospy.logerr(f"[SlamManager] {self._status_message}")
                else:
                    rospy.loginfo("[SlamManager] 不保存地图，跳过")
                    self._status_message = "建图完成（未保存地图）"
                
                # 更新状态
                self.state = SlamState.IDLE
                self._task_start_time = None
            
            # 启动后台等待线程
            wait_thread = threading.Thread(target=wait_for_process, daemon=True)
            wait_thread.start()
            
            # 立即返回响应，不等待进程退出
            if req.save_map:
                msg = f"停止建图指令已发送，将保存地图到: {req.map_name}"
            else:
                msg = "停止建图指令已发送（不保存地图）"
            rospy.loginfo(f"[SlamManager] {msg}")
            return StopMappingResponse(success=True, message=msg)
            
        except Exception as e:
            msg = f"停止建图时发生错误: {str(e)}"
            rospy.logerr(f"[SlamManager] {msg}")
            # 确保状态被重置
            with self._process_lock:
                self._current_process = None
            self.state = SlamState.ERROR
            self._status_message = msg
            return StopMappingResponse(success=False, message=msg)
    
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
        maps_dir = os.path.join(self.kuavo_slam_path, 'maps')
        if not os.path.isdir(maps_dir):
            msg = f"maps目录不存在: {maps_dir}"
            rospy.logwarn(f"[SlamManager] {msg}")
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
                ori_pcd_path = os.path.join(subdir, 'pointcloud_original.pcd')
                valid_flag = os.path.isfile(ori_pcd_path)
                if not valid_flag:
                    continue

                # 可导航判定
                nav_pcd_path = os.path.join(subdir, 'pointcloud.pcd')
                nav_flag = (
                    os.path.isfile(os.path.join(subdir, 'map2d.yaml')) and
                    os.path.isfile(os.path.join(subdir, 'map2d.pgm')) and
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
            rospy.logerr(f"[SlamManager] {msg}")
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
            rospy.logwarn(f"[SlamManager] 监控{task_name}输出时发生错误: {e}")
        
        finally:
            # 进程结束，检查是否需要更新状态
            rospy.loginfo(f"[SlamManager] {task_name}进程输出监控结束")
            
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
                                rospy.logwarn(f"[SlamManager] {self._status_message}")
    
    @staticmethod
    def _strip_ansi(text: str) -> str:
        """移除ANSI转义序列（颜色代码等）"""
        import re
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        return ansi_escape.sub('', text)
    
    def _on_shutdown(self):
        """节点关闭时的清理操作"""
        rospy.loginfo("[SlamManager] 节点正在关闭，清理资源...")
        
        # 如果有正在运行的进程，发送终止信号
        with self._process_lock:
            if self._current_process is not None:
                try:
                    pgid = os.getpgid(self._current_process.pid)
                    rospy.loginfo(f"[SlamManager] 终止运行中的进程组 {pgid}...")
                    os.killpg(pgid, signal.SIGINT)
                    
                    # 等待一段时间让进程完成清理
                    time.sleep(3)
                    
                    # 如果还没退出，强制终止
                    if self._current_process.poll() is None:
                        os.killpg(pgid, signal.SIGKILL)
                        
                except ProcessLookupError:
                    pass
                except Exception as e:
                    rospy.logwarn(f"[SlamManager] 清理进程时发生错误: {e}")
        
        rospy.loginfo("[SlamManager] 清理完成")
    
    def spin(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            # 检查子进程是否意外退出
            with self._process_lock:
                if self._current_process is not None:
                    ret = self._current_process.poll()
                    if ret is not None and self.state == SlamState.MAPPING:
                        rospy.logwarn(f"[SlamManager] 检测到建图进程已退出 (code={ret})")
                        self._current_process = None
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
        rospy.logerr(f"[SlamManager] 发生未处理的异常: {e}")
        raise


if __name__ == '__main__':
    main()
