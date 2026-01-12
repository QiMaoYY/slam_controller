# 地图管理功能使用说明

## 概述

SLAM Controller 提供了完整的地图管理功能，支持在停止建图时自动保存地图，并按命名组织地图文件。

---

## 功能特性

✅ **命名保存地图** - 为每个地图指定唯一名称  
✅ **自动组织** - 在 `kuavo_slam/maps/` 下创建独立目录  
✅ **合法性验证** - 检查地图名字符、长度和重复性  
✅ **灵活选择** - 可选择保存或丢弃地图  
✅ **异步处理** - 地图保存在后台进行，不阻塞服务响应

---

## 服务接口

### StopMapping 服务

**服务名**: `/slam_manager/stop_mapping`  
**类型**: `slam_controller/StopMapping`

#### 请求参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `save_map` | bool | 是否保存地图 |
| `map_name` | string | 地图名称（仅当save_map=true时有效） |

#### 响应

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 操作是否成功 |
| `message` | string | 返回消息 |

---

## 使用方法

### 1. 保存地图

停止建图并保存地图到指定名称：

```bash
rosservice call /slam_manager/stop_mapping "save_map: true
map_name: 'office_floor1'"
```

**执行流程**：
1. 验证地图名合法性
2. 停止建图进程
3. 等待PCD文件写入完成
4. 在 `kuavo_slam/maps/office_floor1/` 创建目录
5. 移动PCD文件并重命名为 `office_floor1_ori.pcd`

**结果**：
```
kuavo_slam/maps/
└── office_floor1/
    └── office_floor1_ori.pcd
```

### 2. 不保存地图

停止建图但不保存地图（用于测试或失败的建图）：

```bash
rosservice call /slam_manager/stop_mapping "save_map: false
map_name: ''"
```

**执行流程**：
1. 停止建图进程
2. 清理临时文件
3. PCD文件保留在 `faster-lio/PCD/` 目录

---

## 地图名规则

### 合法字符
- 字母（a-z, A-Z）
- 数字（0-9）
- 下划线（_）
- 横线（-）

### 长度限制
- 最多 50 个字符
- 不能为空

### 命名示例

✅ **合法**：
- `office_floor1`
- `workshop_2024-01-05`
- `demo_map_v2`
- `building_A_level3`

❌ **非法**：
- `办公室地图` （包含中文）
- `map@v1` （包含特殊字符@）
- `my map` （包含空格）
- `` （空字符串）
- 已存在的地图名

---

## 地图文件结构

每个保存的地图都有独立的目录：

```
kuavo_slam/maps/
├── office_floor1/
│   └── office_floor1_ori.pcd
├── workshop/
│   └── workshop_ori.pcd
└── demo_map/
    └── demo_map_ori.pcd
```

**文件命名规则**：`{map_name}_ori.pcd`

---

## 错误处理

### 常见错误及解决方法

| 错误 | 原因 | 解决方法 |
|------|------|----------|
| `地图名称不能为空` | 未提供map_name | 指定有效的地图名称 |
| `地图名称只能包含...` | 包含非法字符 | 只使用字母、数字、下划线和横线 |
| `地图 'xxx' 已存在` | 地图名重复 | 使用不同的地图名或删除旧地图 |
| `未找到PCD地图文件` | 建图时间太短 | 建图更长时间后再保存 |
| `无法停止建图：当前状态为 idle` | 未在建图状态 | 先启动建图再停止 |

---

## 完整示例

### 场景：保存办公室第一层地图

```bash
# 1. 启动建图（跳过校准）
rosservice call /slam_manager/start_mapping "need_calibration: false"

# 2. 等待建图完成（移动机器人采集数据）
#    ... 建图中 ...

# 3. 查看当前状态
rosservice call /slam_manager/get_status "{}"
# 输出：
# status: "mapping"
# message: "建图进行中"
# uptime_sec: 120

# 4. 停止建图并保存
rosservice call /slam_manager/stop_mapping "save_map: true
map_name: 'office_floor1'"
# 输出：
# success: True
# message: "停止建图指令已发送，将保存地图到: office_floor1"

# 5. 等待几秒后检查状态
sleep 5
rosservice call /slam_manager/get_status "{}"
# 输出：
# status: "idle"
# message: "建图完成，地图已保存: office_floor1"
# uptime_sec: 0

# 6. 验证地图文件
ls -lh /media/data/slam_ws/src/kuavo_slam/maps/office_floor1/
# 输出：
# -rw-rw-r-- 1 user user 82M Jan  6 11:30 office_floor1_ori.pcd
```

---

## 进阶用法

### 批量建图工作流

```bash
#!/bin/bash
# 批量建图脚本示例

MAPS=("area_a" "area_b" "area_c")

for map_name in "${MAPS[@]}"; do
    echo "开始建图: $map_name"
    
    # 启动建图
    rosservice call /slam_manager/start_mapping "need_calibration: false"
    
    # 等待用户完成数据采集
    read -p "完成 $map_name 的数据采集后按回车..."
    
    # 保存地图
    rosservice call /slam_manager/stop_mapping "save_map: true
map_name: '$map_name'"
    
    # 等待处理完成
    sleep 10
    
    echo "地图 $map_name 已保存"
    echo "---"
done

echo "所有地图建图完成！"
```

---

## 注意事项

⚠️ **重要提示**

1. **地图名唯一性**：同名地图会导致保存失败，需要先删除旧地图
2. **建图时长**：建图时间过短可能没有生成PCD文件，建议至少采集10秒以上
3. **磁盘空间**：PCD地图文件较大（通常几十MB到几百MB），注意磁盘空间
4. **异步保存**：地图保存在后台进行，可通过`get_status`查询状态
5. **文件权限**：确保对 `kuavo_slam/maps/` 目录有写权限

---

## 相关命令

```bash
# 查看所有已保存的地图
ls -lh /media/data/slam_ws/src/kuavo_slam/maps/

# 删除指定地图
rm -rf /media/data/slam_ws/src/kuavo_slam/maps/office_floor1/

# 查看地图大小
du -sh /media/data/slam_ws/src/kuavo_slam/maps/*

# 检查faster-lio临时目录
ls -lh /media/data/slam_ws/src/faster-lio/PCD/
```

---

## 故障排查

### 地图未保存

1. 检查日志：`rostopic echo /rosout | grep SlamManager`
2. 确认建图时间足够长
3. 验证faster-lio配置：`pcd_save_en: true`

### 地图名验证失败

1. 使用纯英文、数字、下划线、横线
2. 长度不超过50字符
3. 确保名称未被使用

### 权限错误

```bash
# 修复maps目录权限
chmod 755 /media/data/slam_ws/src/kuavo_slam/maps/
```

---

## 更新日志

- **2026-01-06**: 初始版本，支持命名保存地图

