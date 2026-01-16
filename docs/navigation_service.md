# Navigation 导航服务（简要说明）

## 服务名与类型

- **启动导航**
  - **服务名**：`/slam_manager/start_navigation`
  - **类型**：`slam_controller/StartNavigation`
- **停止导航**
  - **服务名**：`/slam_manager/stop_navigation`
  - **类型**：`slam_controller/StopNavigation`

## 功能

由 `slam_controller/scripts/slam_manager.py` 通过子会话（`start_new_session=True`）启动导航脚本，并将其输出转发到 ROS 日志。

- **导航脚本**：`src/kuavo_slam/scripts/nav_run.sh`
- **脚本路径配置**：`slam_controller/config/slam_controller.yaml` 中的 `scripts.nav_run`

脚本调用形式：

`nav_run.sh <MAP_NAME> <MAP_ROOT> <RVIZ:true|false> <CALIB:true|false>`

其中 `MAP_ROOT` 由配置 `maps.root` 提供。

## 启动导航请求

字段：

- `map_name`：地图名（对应 `maps.root/<map_name>` 子目录）
- `enable_rviz`：是否启动 RViz
- `need_calibration`：是否执行雷达自动校准

### 启动条件（地图可导航性）

`start_navigation` 会检查地图目录中是否同时存在以下文件，否则返回失败：

- `map2d.yaml`
- `map2d.pgm`
- `pointcloud.pcd`

（这些文件通常由地图处理服务 `/slam_manager/process_map` 生成）

## 停止导航请求

无参数。服务会向导航脚本所在进程组发送 `SIGINT`（等同 Ctrl+C）以触发其清理退出；超时则 `SIGKILL` 强制结束。

相关超时参数：

- `timeouts.stop_nav_wait_sec`：等待导航脚本退出的最长时间（秒）

## 调用示例

```bash
# 启动导航（地图名=demo_map，开启RViz，执行校准）
rosservice call /slam_manager/start_navigation "map_name: 'demo_map'
enable_rviz: true
need_calibration: true"

# 启动导航（关闭RViz，跳过校准）
rosservice call /slam_manager/start_navigation "map_name: 'demo_map'
enable_rviz: false
need_calibration: false"

# 停止导航
rosservice call /slam_manager/stop_navigation "{}"
```

