# 导航任务文件规范

## 概述

每个地图可配置一份导航任务文件，用于存储预定义的导航任务点序列。文件位于对应地图目录下，文件名固定为 `nav_tasks.yaml`。

**存放路径示例**：
```
maps/
├── map_demo/
│   ├── pointcloud_original.pcd
│   ├── pointcloud.pcd
│   ├── map2d.yaml
│   ├── map2d.pgm
│   └── nav_tasks.yaml          <-- 导航任务文件
└── map_office/
    └── ...
```

---

## 文件格式

采用 **YAML** 格式，具有良好的人类可读性和程序解析性。

### 顶层结构

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `version` | string | 是 | 文件格式版本号，当前为 `"1.0"` |
| `map_name` | string | 是 | 所属地图名称（与目录名一致） |
| `description` | string | 否 | 任务文件整体描述 |
| `created_at` | string | 否 | 创建时间（ISO 8601 格式） |
| `updated_at` | string | 否 | 最后更新时间（ISO 8601 格式） |
| `task_groups` | list | 是 | 任务组列表 |

### 任务组（Task Group）

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `id` | int | 是 | 任务组索引，从 0 开始，须连续且唯一 |
| `name` | string | 是 | 任务组名称，须在文件内唯一 |
| `description` | string | 否 | 任务组描述 |
| `point_count` | int | 是 | 任务点数量（须与 `points` 列表长度一致） |
| `points` | list | 是 | 任务点列表 |

### 任务点（Waypoint）

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `id` | int | 是 | 任务点索引，从 0 开始，须连续且唯一 |
| `x` | float | 是 | 目标位置 X 坐标（米，地图坐标系） |
| `y` | float | 是 | 目标位置 Y 坐标（米，地图坐标系） |
| `theta` | float | 是 | 目标朝向角度（弧度，范围 -π ~ π） |
| `name` | string | 否 | 任务点别名（如 "起点"、"充电桩"） |

---

## 完整示例

```yaml
# 导航任务文件示例
# 文件名: nav_tasks.yaml
# 存放位置: maps/<map_name>/nav_tasks.yaml

version: "1.0"
map_name: "map_demo"
description: "办公区域巡检任务集合"
created_at: "2026-01-16T10:30:00+08:00"
updated_at: "2026-01-16T15:45:00+08:00"

task_groups:
  # ========== 任务组 0: 日常巡检路线 ==========
  - id: 0
    name: "daily_patrol"
    description: "每日例行巡检路线，覆盖主要通道"
    point_count: 4
    points:
      - id: 0
        x: 0.0
        y: 0.0
        theta: 0.0
        name: "起点"

      - id: 1
        x: 5.2
        y: 0.0
        theta: 1.57
        name: "走廊入口"

      - id: 2
        x: 5.2
        y: 8.5
        theta: 3.14
        name: "会议室门口"

      - id: 3
        x: 0.0
        y: 8.5
        theta: -1.57
        name: "返回点"

  # ========== 任务组 1: 充电返航 ==========
  - id: 1
    name: "return_to_charge"
    description: "返回充电桩"
    point_count: 1
    points:
      - id: 0
        x: -1.0
        y: -0.5
        theta: 0.0
        name: "充电桩"
```

---

## 约束与校验规则

### 必须满足

1. **id 连续性**：任务组和任务点的 `id` 必须从 0 开始连续递增
2. **名称唯一性**：同一文件内所有任务组的 `name` 必须唯一
3. **数量一致性**：`point_count` 必须等于 `points` 列表的实际长度
4. **theta 范围**：建议保持在 `[-π, π]` 范围内（约 `[-3.14159, 3.14159]`）

### 建议遵循

1. **命名规范**：任务组 `name` 使用小写字母 + 下划线（如 `daily_patrol`）
2. **坐标精度**：坐标值保留小数点后 2~3 位即可
3. **注释说明**：为复杂任务添加 YAML 注释便于维护

---

## 程序读取示例（Python）

```python
import yaml
import os

def load_nav_tasks(map_root: str, map_name: str) -> dict:
    """加载指定地图的导航任务文件"""
    task_file = os.path.join(map_root, map_name, 'nav_tasks.yaml')
    if not os.path.isfile(task_file):
        return None
    
    with open(task_file, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    
    return data

def get_task_group_by_name(tasks: dict, group_name: str) -> dict:
    """根据名称获取任务组"""
    for group in tasks.get('task_groups', []):
        if group.get('name') == group_name:
            return group
    return None

# 使用示例
tasks = load_nav_tasks('/media/data/slam_ws/src/kuavo_slam/maps', 'map_demo')
if tasks:
    patrol = get_task_group_by_name(tasks, 'daily_patrol')
    if patrol:
        for pt in patrol['points']:
            print(f"Point {pt['id']}: ({pt['x']}, {pt['y']}, {pt['theta']})")
```

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-01-16 | 初始版本 |

