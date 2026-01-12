# ListMaps 地图列表查询服务（简要说明）

## 服务名与类型

- **服务名**：`/slam_manager/list_maps`
- **类型**：`slam_controller/ListMaps`

## 功能

扫描目录：`/media/data/slam_ws/src/kuavo_slam/maps`（实际路径由节点参数 `~kuavo_slam_path` 决定，默认为 `/media/data/slam_ws/src/kuavo_slam`）

对 `maps` 下的每个**子文件夹**执行判定：

- **有效地图**：子文件夹中存在 `pointcloud_original.pcd`
- **可用于导航**：在“有效地图”的基础上，还同时存在：
  - `map2d.yaml`
  - `map2d.pgm`
  - `pointcloud.pcd`

## 请求

无参数。

## 响应字段

- **success**：是否查询成功
- **message**：补充信息（例如发现的有效地图数量、失败原因）
- **map_names**：有效地图名称列表（即子文件夹名）
- **nav_ready**：与 `map_names` 一一对应，表示该地图是否满足“可用于导航”的文件完整性要求
- **map_paths**：与 `map_names` 一一对应，地图目录完整路径
- **created_at**：与 `map_names` 一一对应，地图创建时间字符串（`YYYYMMDD_HHMMSS`，取 `pointcloud_original.pcd` 的mtime）
- **ori_pointcloud_bytes**：与 `map_names` 一一对应，`pointcloud_original.pcd` 文件大小（字节）
- **nav_pointcloud_bytes**：与 `map_names` 一一对应，`pointcloud.pcd` 文件大小（字节）；若 `nav_ready=false` 则为 `0`

## 调用示例

```bash
rosservice call /slam_manager/list_maps "{}"
```


