# 地图任务查询/写入服务（简要说明）

## 服务名与类型

- **查询任务**
  - 服务名：`/slam_manager/get_map_tasks`
  - 类型：`slam_controller/GetMapTasks`
- **写入任务**
  - 服务名：`/slam_manager/set_map_tasks`
  - 类型：`slam_controller/SetMapTasks`

## 功能

用于读取或写入 **指定地图** 的导航任务文件 `nav_tasks.yaml`。任务文件的结构定义见：

- `docs/nav_tasks_file.md`

文件位置：

```
<maps.root>/<map_name>/nav_tasks.yaml
```

其中 `maps.root` 由 `slam_controller/config/slam_controller.yaml` 配置。

## 请求与响应字段

### 1) GetMapTasks

请求：
- `map_name`：地图名称（对应 `maps.root/<map_name>`）

响应：
- `success`：是否成功
- `message`：补充信息
- `tasks_yaml`：任务文件内容（YAML 字符串）

**行为**：
- 若任务文件不存在：创建空任务文件并返回其内容
- 若文件存在但解析失败：返回失败并提示原因

### 2) SetMapTasks

请求：
- `map_name`：地图名称（对应 `maps.root/<map_name>`）
- `tasks_yaml`：任务文件内容（YAML 字符串）

响应：
- `success`：是否成功
- `message`：补充信息

**行为**：
- 若任务文件不存在：创建新文件
- 若文件已存在：直接覆盖
- 若 `tasks_yaml` 为空或不可解析：返回失败
- 若 `tasks_yaml` 中 `map_name` 与请求不一致：返回失败

## 调用示例

### 查询任务

```bash
rosservice call /slam_manager/get_map_tasks "map_name: 'map_demo'"
```

### 写入任务（覆盖/新建）

```bash
rosservice call /slam_manager/set_map_tasks "map_name: 'map_demo'
tasks_yaml: |
  version: \"1.0\"
  map_name: \"map_demo\"
  task_groups:
    - id: 0
      name: \"daily_patrol\"
      point_count: 1
      points:
        - id: 0
          x: 0.0
          y: 0.0
          theta: 0.0"
```

