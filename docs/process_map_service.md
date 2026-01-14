# ProcessMap 地图处理服务（简要说明）

## 服务名与类型

- **服务名**：`/slam_manager/process_map`
- **类型**：`slam_controller/ProcessMap`

## 功能

通过服务调用后台执行脚本：`src/pcd_cleaner/scripts/run_all.sh`，用于完成点云清理、2D地图生成、编辑与点云过滤等流程。

脚本参数格式：

`run_all.sh [地图根目录] [地图名] [屋顶高度阈值] [运行方式]`

其中：
- **地图根目录**、**屋顶高度阈值**：由 `slam_manager.py` 读取配置文件获得（请求中不传）
- **地图名**、**运行方式**：由服务请求传入

## 配置文件

默认配置文件：`slam_controller/config/slam_controller.yaml`

可通过ROS参数覆盖路径：`~global_config`

相关配置项（节选）：
- `maps.root`：地图根目录
- `map_processing.ceiling_threshold`：屋顶高度阈值
- `scripts.map_processing`：地图处理脚本路径

## 请求

- `map_name`：地图名（对应 `map_root/<map_name>`）
- `mode`：运行方式（与 `run_all.sh` 一致）
  - `0` 全流程（清理+2D转换+从init开始编辑+过滤）
  - `1` 仅执行 点云清理+2D转换
  - `2` 仅执行 编辑2D地图（从init复制一份重新编辑）
  - `3` 仅执行 智能点云过滤
  - `4` 仅执行 继续编辑2D地图（不从init复制）

## 行为与约束

- 若地图目录不存在或缺少 `pointcloud_original.pcd`，将返回失败
- 同一时间只允许一个地图处理任务运行；若已有任务在运行，返回失败
- 服务调用成功后立即返回（后台执行），可通过 `/slam_manager/get_status` 观察状态（`processing`）

## 调用示例

```bash
# 对 test2 地图执行“点云清理+2D转换”
rosservice call /slam_manager/process_map "map_name: 'test2'
mode: 1"
```


