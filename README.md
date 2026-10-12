# AIC 2026 智慧社区巡检仿真项目

基于 ROS2 Jazzy + Gazebo Harmonic 的智慧社区巡检机器人仿真，实现差速机器人、
红绿灯动态切换、SLAM 建图和 Nav2 自主导航。

## 一、环境要求

- **Ubuntu 24.04 LTS**（VMware 虚拟机）
- **ROS2 Jazzy**
- **Gazebo Harmonic**（gz-sim 8.x，`ros-jazzy-ros-gz`）
- 依赖包：`ros-jazzy-navigation2`、`ros-jazzy-nav2-bringup`、`ros-jazzy-slam-toolbox`、`ros-jazzy-teleop-twist-keyboard`

安装依赖（若缺失）：
```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard
```

## 二、已完成功能

| 模块 | 状态 | 说明 |
|---|---|---|
| 机器人 `patrol_bot` | ✅ | 差速底盘 + 2D LiDAR + RGB 相机 + IMU，球体轮子碰撞（不打滑） |
| 传感器桥接 | ✅ | LiDAR/相机/IMU/odom/cmd_vel → ROS2，frame_id 前缀已重映射 |
| 红绿灯动态切换 | ✅ | 两盏灯红→绿→黄循环，双缓冲切换（无闪烁） |
| 红绿灯颜色识别 | ✅ | HSV 分割识别红/绿/黄状态，发布 `/traffic_light/*` 话题 + 标注图像 |
| SLAM 建图 | ✅ | slam_toolbox 在线异步建图，已保存地图 |
| Nav2 自主导航 | ✅ | AMCL 定位 + 规划 + 控制，单点导航已跑通 |
| 航点巡检 | ⏳ | 未完成（下一步） |
| 视觉识别（人偶/车牌） | ⏳ | 未完成 |

## 三、编译

```bash
cd ~/aic_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 四、启动方式

### 方式 1：GUI 完整演示（世界 + 机器人 + 红绿灯）

```bash
ros2 launch robot_description bringup.launch.py
```
- 打开 Gazebo 窗口，机器人在场地中央，红绿灯自动交替
- 需在图形桌面终端里跑（有 DISPLAY）
- **注意**：VMware 下 Gazebo GUI 长时间运行可能卡死（Xwayland 渲染问题），卡死不影响数据，重启即可

### 方式 2：无头导航（稳定，推荐导航用）

```bash
ros2 launch robot_description nav_bringup.launch.py
```
- 无头模式（gz sim -s，不渲染 GUI），启动 localization + navigation
- 配合 RViz 或命令行发导航目标

### 方式 3：建图（无头 + 自动避障，或手动遥控）

```bash
ros2 launch robot_description mapping.launch.py    # 自动避障建图（odom 有漂移，不推荐）
```
推荐**手动遥控建图**（慢速直线，无漂移）：
1. 先启动 GUI 演示（方式 1）
2. 另开终端启动 SLAM：
   ```bash
   ros2 launch robot_description slam.launch.py
   ```
3. 遥控建图（见下节）
4. 保存地图：
   ```bash
   cd ~/aic_ws/src/smart_community_gazebo/maps
   ros2 run nav2_map_server map_saver_cli -f community
   ```

## 五、遥控机器人（WASD）

```bash
ros2 run robot_description wasd_teleop.py
```
- **W** 前进 / **S** 后退 / **A** 左转 / **D** 右转 / **Q** 退出
- 必须**点击终端窗口**获得焦点后再按键
- 卡墙了按 S 后退，或重置位置：
  ```bash
  ros2 run robot_description reset_robot.py
  ```

## 六、Nav2 导航操作（重要：lifecycle 需手动激活）

`nav_bringup.launch.py` 启动后，**navigation 节点的 lifecycle 管理器常因 map TF 延迟而超时失败**，
需手动激活：

```bash
source ~/aic_ws/install/setup.bash

# 1. 激活定位（若未自动激活）
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl activate

# 2. 设置 AMCL 初始位姿（机器人起点，朝 +Y）
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}}}" -1

# 3. 激活 navigation 节点
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /behavior_server activate
ros2 lifecycle set /velocity_smoother activate
ros2 lifecycle set /smoother_server activate

# 4. 发导航目标（注意：目标必须在已建地图范围内）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.8, y: -1.2}, orientation: {z: 0.0, w: 1.0}}}}"
```

## 七、文件结构

```
aic_ws/
├── src/
│   ├── robot_description/          # 机器人描述包
│   │   ├── urdf/robot.xacro        # 差速机器人模型（LiDAR/相机/IMU/插件）
│   │   ├── config/
│   │   │   ├── mapper_params_online_async.yaml  # SLAM 参数
│   │   │   └── nav2_params.yaml                 # Nav2 参数
│   │   ├── launch/
│   │   │   ├── bringup.launch.py   # GUI 完整启动（世界+机器人+红绿灯+桥接）
│   │   │   ├── mapping.launch.py   # 无头建图
│   │   │   ├── nav_bringup.launch.py  # 无头导航
│   │   │   ├── slam.launch.py      # 单独启动 SLAM
│   │   │   ├── nav2.launch.py      # 单独启动 Nav2
│   │   │   └── spawn_robot.launch.py
│   │   └── scripts/
│   │       ├── frame_remap.py      # frame_id 前缀重映射 + odom→base_link TF
│   │       ├── traffic_light_detector.py  # 红绿灯颜色识别（HSV 分割）
│   │       ├── wasd_teleop.py      # WASD 遥控
│   │       ├── reset_robot.py      # 重置机器人位置
│   │       └── auto_mapping.py     # 自动避障建图（有漂移，不推荐）
│   └── smart_community_gazebo/     # 场景包（建模团队交付，见其 README.md）
│       ├── worlds/smart_community_v2.sdf   # 场地世界
│       ├── models/                 # 人偶/红绿灯/车辆模型
│       ├── maps/                   # 建好的地图（community.pgm + yaml）
│       └── scripts/traffic_light_controller.py  # 红绿灯动态切换
└── README.md                       # 本文档
```

## 八、关键踩坑记录（重要！）

1. **gz-sim 传感器需 world 加载 `gz-sim-sensors-system` 插件**，否则 LiDAR/相机静默不工作
2. **LiDAR 用 `type="gpu_lidar"`**（`ray` 类型在 gz-sim 8 已不支持）
3. **spawn 机器人用 SDF**（`gz sdf -p` 转换），传 URDF 会丢传感器
4. **`ros_gz_sim create` 参数是 `-name`**（不是 `-entity`）
5. **轮子碰撞用球体**（圆柱在 ODE 里打滑严重）
6. **轮子朝向用标准写法**（visual 里 rpy=1.5708，joint rpy=0），否则横着走
7. **轮轴方向 `0 -1 0`**（否则前进后退相反）
8. **frame_id 前缀**：gz 传感器 frame 带 `patrol_bot/` 前缀且用 sensor 名，需 frame_remap 重映射到 link 名
9. **odom→base_link TF 要从 odom 消息生成**（gz 不持续发布 TF），且 stamp 用 odom 原始时间戳（与 lidar 一致，否则 AMCL 丢数据）
10. **红绿灯 remove 服务要设 `type=MODEL`**，且先建后删避免闪烁
11. **Nav2 velocity_smoother 输出是 `/cmd_vel_smoothed`**，需桥接到 gz 的 cmd_vel
12. **lifecycle 管理器易超时**（VMware 慢），需手动激活节点
13. **gz GUI 卡死**：改用无头模式（`gz sim -s`）稳定

## 九、已知问题 & 下一步

1. **地图未覆盖场地上方区域**（只覆盖下方约 2.2m）——需重新走遍全场建图
2. **IMU 传感器被 gz-sim 静默丢弃**（日志无报错），待排查
3. **航点巡检未完成**——可用 nav2 的 `follow_waypoints` action 或自定义节点
4. **视觉识别部分完成**——红绿灯颜色识别已做（`traffic_light_detector.py`），人偶身份分类、车牌 OCR 待做
5. **红灯停绿灯行未接入导航**——`/traffic_light/state` 已发布，需在巡检逻辑里消费该话题控制停车/起步
5. **自动避障建图 odom 漂移**——差速机器人频繁转向打滑，建议手动建图

## 十、参考

- 场景建模说明：`src/smart_community_gazebo/README.md`
- 建模团队交接：`src/smart_community_gazebo/TEAM_HANDOFF.md`
- 任务要求：`src/smart_community_gazebo/reference/任务要求(1).txt`
