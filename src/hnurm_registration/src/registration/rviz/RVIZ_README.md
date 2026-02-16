# RViz 重定位可视化配置

## 配置文件说明

`relocalization_view.rviz` 是专门为重定位节点设计的 RViz 配置文件，提供了清晰的配准效果对比和调试功能。

---

## 启动方式

### 方式一：命令行启动（推荐）

```bash
# 进入工作空间
cd /home/robot/Registrate2/V1

# 启动重定位节点（确保你的 launch 文件或节点已启动）
ros2 run registration registration_node

# 启动 RViz 并加载配置
rviz2 -d src/registration/rviz/relocalization_view.rviz
```

### 方式二：Launch 文件中启动

在你的 launch 文件中添加：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('registration'),
        'rviz',
        'relocalization_view.rviz'
    )
    
    return LaunchDescription([
        # 你的重定位节点
        Node(
            package='registration',
            executable='registration_node',
            name='relocalization_node'
        ),
        # RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])
```

---

## 显示元素详解

### 1. Grid（网格）
- **用途**: 提供空间参考
- **颜色**: 灰色 (160, 160, 164)
- **单元格大小**: 1m

### 2. TF（坐标变换）
- **用途**: 显示坐标系关系
- **显示内容**:
  - `map` → `odom` / `lidar` 的变换（来自配准结果）
- **Marker Scale**: 2（便于观察）
- **注意**: 配准成功后才会显示正确的 TF 树

### 3. Global Map (Static) - 全局地图点云 ⭐
- **话题**: `/global_pcd_map`
- **颜色**: 灰色 (200, 200, 200)
- **点大小**: 1 pixel
- **Decay Time**: 0（永久显示）
- **用途**: 作为配准参照的静态地图

### 4. Registered Cloud (Current) - 配准后点云 ⭐
- **话题**: `/pointcloud_registered`
- **颜色**: 橙色 (255, 85, 0) 🔶
- **点大小**: 3 pixels（比地图大，便于对比）
- **Decay Time**: 0.1s（只显示最新帧）
- **用途**: 显示当前帧配准结果
- **效果**: 配准成功时，橙色点云应与灰色地图重合

### 5. Source Cloud (Debug) - 源点云（调试用）
- **话题**: `/source_cloud_debug`
- **颜色**: 绿色 (0, 255, 0)
- **默认**: **关闭**（需要你的代码发布这个话题）
- **用途**: 调试时查看原始输入点云

### 6. Initial Pose Estimate - 初始位姿估计
- **话题**: `/initialpose`
- **显示**: 协方差椭圆 + 箭头
- **用途**: 显示通过 RViz 工具发布的初始位姿

---

## 关键工具

### 2D Pose Estimate（2D位姿估计）
- **位置**: RViz 顶部工具栏第 6 个图标
- **快捷键**: `P`
- **使用方法**:
  1. 点击工具栏中的 "2D Pose Estimate" 按钮
  2. 在地图中点击并拖动，设置初始位姿
  3. 箭头方向为机器人朝向
- **发布话题**: `/initialpose`
- **用途**: 给重定位节点提供初始猜测位姿

---

## 视觉对比效果说明

| 元素 | 颜色 | 点大小 | 视觉效果 |
|------|------|--------|----------|
| 全局地图 | 灰色 | 小 (1px) | 背景，不抢眼 |
| 配准点云 | 橙色 | 大 (3px) | 突出显示，便于观察重合度 |

### 配准成功时的视觉特征
✅ 橙色点云与灰色地图**基本重合**  
✅ TF 箭头显示正确的位置和朝向  
✅ 两团点云没有明显偏移或旋转错位

### 配准失败时的视觉特征
❌ 橙色点云与灰色地图**明显分离**  
❌ 可能存在旋转或平移偏差  
❌ 需要重新给出初始位姿或检查配准参数

---

## 使用流程

```
1. 启动重定位节点
        ↓
2. 启动 RViz（加载本配置）
        ↓
3. 观察灰色地图是否正常显示
        ↓
4. 使用 "2D Pose Estimate" 工具给出初始位姿
        ↓
5. 观察橙色点云是否与灰色地图重合
        ↓
6. 如未重合，重新调整初始位姿
```

---

## 常见问题

### Q: 看不到点云？
- 检查 Fixed Frame 是否为 `map`
- 检查话题名是否匹配：`/global_pcd_map` 和 `/pointcloud_registered`
- 使用 `ros2 topic list` 确认话题存在
- 使用 `ros2 topic hz /global_pcd_map` 确认有数据发布

### Q: 配准点云飘在地图旁边？
- 检查 TF 是否正常发布：`ros2 run tf2_tools view_frames`
- 确认 Fixed Frame 设为 `map` 而不是 `odom`
- 检查配准是否收敛（看日志）

### Q: 初始位姿发布后没反应？
- 确认节点订阅了 `/initialpose` 话题
- 检查是否有初始位姿回调的日志输出
- 确认 use_rviz_revise_ 参数是否为 true

### Q: 点云颜色/大小不满意？
- 在 Displays 面板中点击对应点云
- 修改 Color 或 Size (Pixels) 参数
- 修改后 File → Save Config 保存

---

## 自定义修改

### 修改点云颜色
1. 在 RViz 左侧 Displays 面板中找到对应点云
2. 展开后修改 **Color** 属性
3. 常用配色方案：
   - 地图：白色 (255,255,255) 或灰色 (200,200,200)
   - 配准点云：红色 (255,0,0)、绿色 (0,255,0)、青色 (0,255,255)

### 修改点云大小
- **Size (Pixels)**: 屏幕像素大小，不受缩放影响
- **Size (m)**: 实际米制大小，缩放时会变化
- 建议：地图用 1-2px，配准点云用 3-4px

### 添加更多显示
- **Axes**: 显示坐标轴
- **Path**: 显示轨迹（需要发布 nav_msgs/Path）
- **MarkerArray**: 显示自定义标记

---

## 配置参数速查

| 参数 | 值 | 说明 |
|------|-----|------|
| Fixed Frame | `map` | 全局坐标系 |
| 地图话题 | `/global_pcd_map` | 降采样全局点云 |
| 配准话题 | `/pointcloud_registered` | 配准后点云 |
| 初始位姿话题 | `/initialpose` | RViz 2D Pose Estimate |
| 状态话题 | `/registration_status` | 节点状态（INIT/TRACKING/RESET）|
