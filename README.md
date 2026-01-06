# SLAM Controller

SLAM系统总控节点，提供ROS Service接口统一管理Kuavo机器人的SLAM功能。

## 功能

- 🎯 **建图控制**：通过ROS Service启动/停止建图流程
- 📊 **状态管理**：实时维护SLAM系统状态（空闲/建图中/定位中/导航中）
- 🔄 **进程管理**：管理建图、导航脚本的生命周期
- 📝 **日志监控**：监控并转发子进程输出到ROS日志

## 依赖

- `kuavo_slam`：SLAM基础功能包（建图、定位、导航脚本）
- Python 3.8.20（conda demo环境）
- ROS Noetic

## 使用方法

### 启动节点

**方式1：使用启动脚本（推荐）**
```bash
# 自动配置conda环境
rosrun slam_controller run_slam_manager.sh
```

**方式2：使用roslaunch**
```bash
# 需要先激活conda环境
conda activate demo
source /opt/ros/noetic/setup.bash
source /media/data/slam_ws/devel/setup.bash
roslaunch slam_controller slam_manager.launch
```

### 服务接口

#### 1. 开始建图
```bash
# 含雷达校准
rosservice call /slam_manager/start_mapping "need_calibration: true"

# 跳过校准
rosservice call /slam_manager/start_mapping "need_calibration: false"
```

#### 2. 停止建图
```bash
rosservice call /slam_manager/stop_mapping "{}"
```
停止建图后，脚本会自动保存地图到 `kuavo_slam/PCD/` 目录。

#### 3. 获取系统状态
```bash
rosservice call /slam_manager/get_status "{}"
```

**返回示例：**
```yaml
status: "idle"          # idle/mapping/localizing/navigating/error
message: "系统就绪"
uptime_sec: 0          # 当前任务运行时间（秒）
```

## 系统状态

| 状态 | 描述 |
|------|------|
| `idle` | 空闲，可以开始新任务 |
| `mapping` | 建图进行中 |
| `localizing` | 定位中（预留）|
| `navigating` | 导航中（预留）|
| `error` | 发生错误 |

## 文件结构

```
slam_controller/
├── CMakeLists.txt              # CMake配置
├── package.xml                 # 包依赖定义
├── README.md                   # 本文档
├── srv/                        # Service定义
│   ├── StartMapping.srv        # 开始建图服务
│   ├── StopMapping.srv         # 停止建图服务
│   └── GetSlamStatus.srv       # 获取状态服务
├── scripts/                    # 脚本文件
│   ├── slam_manager.py         # 主控节点
│   └── run_slam_manager.sh     # 启动脚本
└── launch/                     # Launch文件
    └── slam_manager.launch     # 节点启动配置
```

## 技术细节

### 进程管理
- 使用 `subprocess.Popen` 启动建图脚本
- 通过 `start_new_session=True` 创建独立进程组
- 停止时发送 `SIGINT` 信号（等同于Ctrl+C），触发脚本的清理流程

### 异步处理
- 停止建图服务立即返回响应，避免阻塞
- 后台线程等待脚本完成清理和地图保存
- 自动检测进程退出并更新状态

### 线程安全
- 使用 `threading.Lock` 保护状态和进程对象
- 支持并发服务调用

## 注意事项

⚠️ **环境要求**
- 必须在 `conda activate demo` 环境下运行（Python 3.8.20）
- Python 3.13+ 与 rospy 存在兼容性问题

⚠️ **ROS Master**
- 节点需要ROS Master运行
- 停止建图时不会关闭ROS Master，确保其他节点正常运行

## 许可证

MIT License

