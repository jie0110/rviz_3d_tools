# rviz_3d_tools

ROS1 (Noetic) RViz 插件包，提供支持完整三维位置（XYZ + Yaw）的交互工具，用于替代标准 RViz 中只能在平面上设置位姿的工具。

## 功能概述

| 插件 | 快捷键 | 发布话题 | 消息类型 |
|------|--------|----------|----------|
| **3D Pose Estimate** | `P` | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` |
| **3D Nav Goal** | `G` | `/goal_3d` | `geometry_msgs/PoseStamped` |

两个工具均通过读取 OpenGL 深度缓冲区反投影鼠标点击位置，将三维坐标（包含 Z 轴）写入消息。当深度读取失败时自动回退到 Z=0 水平面。

## 使用方式

**操作步骤（两个工具相同）：**

1. 在 RViz 工具栏选择对应工具（或按快捷键）。
2. 在场景中**按下左键**，确定三维位置（此时箭头出现）。
3. **拖动鼠标**调整朝向（Yaw）。
4. **释放左键**，消息发布，箭头消失。

## 依赖

- ROS Noetic
- `rviz`
- `pluginlib`
- `geometry_msgs`
- `roscpp`
- Qt5 Widgets
- OpenGL

## 编译

```bash
cd ~/nav_ros1_ws
catkin_make --only-pkg-with-deps rviz_3d_tools
source devel/setup.bash
```

## 在 RViz 中加载插件

1. 启动 RViz：`rosrun rviz rviz`
2. 菜单 **Panels → Tool Properties**，或直接在工具栏空白处右键 → **Add tool**。
3. 找到 `rviz_3d_tools/Pose3DTool` 和 `rviz_3d_tools/Goal3DTool`，添加即可。

也可直接加载预配置文件：

```bash
rosrun rviz rviz -d $(rospack find rviz_3d_tools)/rviz/rviz_3d_tools.rviz
```

## 辅助脚本

### map_pub.py

将本地 PCD 点云文件以 `sensor_msgs/PointCloud2` 格式循环发布到 `/map` 话题（1 Hz），供 RViz 可视化地图点云使用。

```bash
# 修改脚本中的 pcd_file_path 后运行
rosrun rviz_3d_tools map_pub.py
```

依赖 Python 库：`open3d`、`numpy`。

示例地图文件位于 `map/output.pcd`。

## 目录结构

```
rviz_3d_tools/
├── include/rviz_3d_tools/
│   ├── pose_3d_tool.h       # Pose3DTool 类声明
│   └── goal_3d_tool.h       # Goal3DTool 类声明
├── src/
│   ├── pose_3d_tool.cpp     # 发布 /initialpose
│   └── goal_3d_tool.cpp     # 发布 /goal_3d
├── scripts/
│   └── map_pub.py           # PCD 点云发布节点
├── map/
│   └── output.pcd           # 示例地图点云
├── rviz/
│   └── rviz_3d_tools.rviz   # 预配置 RViz 文件
├── plugin_description.xml   # pluginlib 插件注册
├── CMakeLists.txt
└── package.xml
```

## 实现细节

两个工具的深度拾取逻辑相同：通过 `glReadPixels` 读取前帧缓冲区的深度值，结合相机的投影-视图矩阵逆变换得到世界坐标，避免触发 RViz SelectionManager 的额外渲染通道。朝向通过在固定 Z 平面上的射线相交求解，取 `atan2(dy, dx)` 作为 Yaw 角。
