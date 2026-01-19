#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam_controller 各类 Service 回调实现
"""

import os
import signal
import subprocess
import threading
import time
from datetime import datetime

import rospy

from slam_controller.srv import (
    StartMapping, StartMappingResponse,
    StopMapping, StopMappingResponse,
    StartNavigation, StartNavigationResponse,
    StopNavigation, StopNavigationResponse,
    GetSlamStatus, GetSlamStatusResponse,
    ListMaps, ListMapsResponse,
    ProcessMap, ProcessMapResponse,
)

from utils import (
    SlamState,
    bool_to_sh,
    validate_map_name,
    validate_nav_map_ready,
    save_map,
)


class SlamServiceHandlers:
    """
    Service 回调集合（作为 SlamManager 的 mixin 使用）

    依赖 SlamManager 的核心成员变量与方法：
    - state 属性 / _task_start_time / _status_message
    - 各类路径、参数、锁与进程对象
    - _monitor_process_output 方法
    """

    def _handle_start_mapping(self, req: StartMapping) -> StartMappingResponse:
        """
        处理开始建图请求
        """
        rospy.loginfo(f"收到开始建图请求 (校准={req.need_calibration})")

        if self.state != SlamState.IDLE:
            msg = f"无法开始建图：当前状态为 {self.state.value}"
            rospy.logwarn(f"{msg}")
            return StartMappingResponse(success=False, message=msg)

        if not os.path.exists(self.mapping_script):
            msg = f"建图脚本不存在: {self.mapping_script}"
            rospy.logerr(f"{msg}")
            return StartMappingResponse(success=False, message=msg)

        cmd = ['/bin/bash', self.mapping_script]
        if not req.need_calibration:
            cmd.append('--no-calib')

        try:
            with self._process_lock:
                self._current_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    universal_newlines=True,
                    bufsize=1,
                )

            self._output_thread = threading.Thread(
                target=self._monitor_process_output,
                args=(self._current_process, "建图"),
                daemon=True
            )
            self._output_thread.start()

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
        """
        map_name = (req.map_name or '').strip()
        rospy.loginfo(
            f"收到启动导航请求 (map={map_name}, rviz={req.enable_rviz}, 校准={req.need_calibration})"
        )

        if self.state != SlamState.IDLE:
            msg = f"无法启动导航：当前状态为 {self.state.value}"
            rospy.logwarn(f"{msg}")
            return StartNavigationResponse(success=False, message=msg)

        valid, err = validate_map_name(map_name)
        if not valid:
            return StartNavigationResponse(success=False, message=f"地图名非法: {err}")

        ok, msg = validate_nav_map_ready(
            self.map_root,
            map_name,
            self.map2d_yaml_name,
            self.map2d_pgm_name,
            self.nav_pointcloud_name,
        )
        if not ok:
            rospy.logwarn(msg)
            return StartNavigationResponse(success=False, message=msg)

        if not os.path.isfile(self.nav_script):
            msg = f"导航脚本不存在: {self.nav_script}"
            rospy.logerr(f"{msg}")
            return StartNavigationResponse(success=False, message=msg)

        cmd = [
            '/bin/bash',
            self.nav_script,
            map_name,
            self.map_root,
            bool_to_sh(req.enable_rviz),
            bool_to_sh(req.need_calibration),
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

            threading.Thread(
                target=self._monitor_process_output,
                args=(self._nav_process, f"导航:{map_name}"),
                daemon=True
            ).start()

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

    def _handle_stop_mapping(self, req: StopMapping) -> StopMappingResponse:
        """
        处理停止建图请求
        """
        map_name = (req.map_name or '').strip()
        rospy.loginfo(f"收到停止建图请求 (保存={req.save_map}, 地图名={map_name})")

        if req.save_map:
            valid, error_msg = validate_map_name(map_name)
            if not valid:
                rospy.logwarn(f"地图名验证失败: {error_msg}，将保持建图继续运行")
                return StopMappingResponse(success=False, message=f"地图名非法: {error_msg}")

            map_dir = os.path.join(self.map_root, map_name)
            if os.path.exists(map_dir):
                msg = f"地图名重复：'{map_name}' 已存在，将保持建图继续运行"
                rospy.logwarn(msg)
                return StopMappingResponse(success=False, message=msg)

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

                try:
                    pgid = os.getpgid(self._current_process.pid)
                    rospy.loginfo(f"向进程组 {pgid} 发送SIGINT信号（触发脚本清理和保存地图）...")
                    os.killpg(pgid, signal.SIGINT)
                except ProcessLookupError:
                    rospy.logwarn("进程已不存在")
                    self._current_process = None
                    self.state = SlamState.IDLE
                    return StopMappingResponse(success=True, message="建图进程已结束")

            def wait_for_process():
                """异步等待进程退出并处理地图保存"""
                start_wait = time.time()

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
                                except Exception:
                                    pass
                                self._current_process = None
                        break

                    time.sleep(0.5)

                if req.save_map:
                    rospy.loginfo(f"开始保存地图: {map_name}")
                    success, msg = save_map(
                        map_name,
                        self.kuavo_slam_path,
                        self.faster_lio_pcd_dir,
                        self.ori_pointcloud_name,
                    )
                    if success:
                        self._status_message = f"建图完成，地图已保存: {map_name}"
                    else:
                        self._status_message = f"建图完成，但保存地图失败: {msg}"
                        rospy.logerr(f"{self._status_message}")
                else:
                    rospy.loginfo("不保存地图，跳过")
                    self._status_message = "建图完成（未保存地图）"

                self.state = SlamState.IDLE
                self._task_start_time = None

            wait_thread = threading.Thread(target=wait_for_process, daemon=True)
            wait_thread.start()

            if req.save_map:
                msg = f"停止建图指令已发送，将保存地图到: {map_name}"
            else:
                msg = "停止建图指令已发送（不保存地图）"
            rospy.loginfo(f"{msg}")
            return StopMappingResponse(success=True, message=msg)

        except Exception as e:
            msg = f"停止建图时发生错误: {str(e)}"
            rospy.logerr(f"{msg}")
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
        """
        return GetSlamStatusResponse(
            status=self.state.value,
            message=self._status_message,
            uptime_sec=self._get_uptime()
        )

    def _handle_list_maps(self, req: ListMaps) -> ListMapsResponse:
        """
        查询地图列表，并判断每个地图是否可用于导航。
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

                ori_pcd_path = os.path.join(subdir, self.ori_pointcloud_name)
                valid_flag = os.path.isfile(ori_pcd_path)
                if not valid_flag:
                    continue

                nav_pcd_path = os.path.join(subdir, self.nav_pointcloud_name)
                nav_flag = (
                    os.path.isfile(os.path.join(subdir, self.map2d_yaml_name)) and
                    os.path.isfile(os.path.join(subdir, self.map2d_pgm_name)) and
                    os.path.isfile(nav_pcd_path)
                )

                map_names.append(entry)
                nav_ready.append(bool(nav_flag))
                map_paths.append(subdir)

                try:
                    ts = os.path.getmtime(ori_pcd_path)
                    created_at.append(datetime.fromtimestamp(ts).strftime('%Y%m%d_%H%M%S'))
                except Exception:
                    created_at.append('')

                try:
                    ori_pointcloud_bytes.append(int(os.path.getsize(ori_pcd_path)))
                except Exception:
                    ori_pointcloud_bytes.append(0)

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
        """
        map_name = (req.map_name or '').strip()
        mode = int(req.mode)

        valid, err = validate_map_name(map_name)
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

                threading.Thread(
                    target=self._monitor_process_output,
                    args=(self._map_process, f"地图处理:{map_name}"),
                    daemon=True
                ).start()

                self.state = SlamState.PROCESSING
                self._task_start_time = time.time()
                self._status_message = f"地图处理中: {map_name} (mode={mode})"

            except Exception as e:
                self._map_process = None
                self.state = SlamState.ERROR
                self._status_message = f"启动地图处理失败: {e}"
                return ProcessMapResponse(success=False, message=self._status_message)

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
            if self.state == SlamState.PROCESSING:
                self.state = SlamState.IDLE
                self._task_start_time = None
                if ret == 0:
                    self._status_message = f"地图处理完成: {map_name}"
                else:
                    self._status_message = f"地图处理失败(code={ret}): {map_name}"

        threading.Thread(target=_wait_done, daemon=True).start()

        return ProcessMapResponse(success=True, message=f"已启动地图处理: {map_name} (mode={mode})")

