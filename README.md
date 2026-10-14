# AIC 2026 智慧社区巡检仿真项目

基于 ROS2 Jazzy + Gazebo Harmonic 的智慧社区巡检机器人仿真，实现差速机器人、
红绿灯动态切换、SLAM 建图、Nav2 自主导航，以及完整的航点巡检任务
（红绿灯检测 + 人偶身份识别 + 车牌 OCR + 路径合规）。

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
| 红绿灯颜色识别 | ✅ | HSV 分割识别红/绿/黄状态，发布 `/traffic_light/state` + 标注图像 |
| SLAM 建图 | ✅ | slam_toolbox 在线异步建图 + `coverage_mapping.py` 蛇形自动建图 |
| Nav2 自主导航 | ✅ | AMCL 定位 + 规划 + 控制 |
| 航点巡检 | ✅ | `waypoint_patrol.py` 按 `official_route.yaml` 逐点导航 |
| 人偶身份识别 | ✅ | `vision_detector.py` 身份 SVM + 街区社区/非社区人数统计 |
| 车牌 OCR | ✅ | `vision_detector.py` 车牌逐字符 OCR |

## 三、编译

```bash
cd ~/aic
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 四、启动方式

### 方式 1：GUI 完整演示（世界 + 机器人 + 红绿灯 + Nav2）

```bash
ros2 launch robot_description bringup.launch.py
```
- 打开 Gazebo 窗口，机器人生成在起点 (3.88, 3.88) 朝西（yaw=π），红绿灯自动交替
- 需在图形桌面终端里跑（有 DISPLAY）
- **注意**：VMware 下 Gazebo GUI 长时间运行可能卡死（Xwayland 渲染问题），卡死不影响数据，重启即可

### 方式 2：无头导航（稳定，推荐）

```bash
ros2 launch robot_description nav_bringup.launch.py
```
- 无头模式（gz sim -s，不渲染 GUI），启动 Gazebo + 桥接 + 红绿灯 + localization + navigation
- 与方式 1 一样包含完整 Nav2 和视觉所需话题，跑巡检最稳定

### 方式 3：航点巡检（完整比赛流程，三个终端）

启动后需手动激活 lifecycle 并发布初始位姿（VMware 下 lifecycle 管理器易超时），
用现成的 `nav_start.sh` 一条命令搞定。顺序如下：

**终端 1 — 仿真 + 导航（无头）**
```bash
cd ~/aic && source install/setup.bash
ros2 launch robot_description nav_bringup.launch.py
```

**终端 2 — 激活 lifecycle + 发布初始位姿**
```bash
bash ~/aic/nav_start.sh
```
脚本会自动激活定位/导航节点，并以带协方差的方式连发 3 次初始位姿
`(3.88, 3.88, yaw=π)`，避免启动时车头偏转。看到「等 5 秒让 AMCL 收敛」提示后再进终端 3。

**终端 3 — 视觉识别 + 巡检调度**
```bash
cd ~/aic && source install/setup.bash
ros2 launch robot_description patrol.launch.py          # 跑一轮
ros2 launch robot_description patrol.launch.py loop:=true   # 循环巡检
```

巡检过程中终端按顺序输出：红绿灯等待、人偶识别（类别/内容/置信度）、车牌 OCR、
终点街区统计（社区/非社区人数）。标注图像发布在 `/vision/annotated`，
可用 `ros2 run rqt_image_view rqt_image_view /vision/annotated` 查看。

### 方式 4：建图

推荐**自动蛇形建图**（无头，覆盖全场）：
```bash
ros2 launch robot_description mapping.launch.py    # 左下角 spawn，自动蛇形扫场
# 跑完后另开终端保存地图：
cd ~/aic/src/smart_community_gazebo/maps
ros2 run nav2_map_server map_saver_cli -f community
```
也可以手动遥控建图（慢速直线，无漂移）：先启动方式 1，再另开终端
`ros2 launch robot_description slam.launch.py`，用遥控节点（见下节）走遍全场后保存。

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

`nav_bringup.launch.py` 启动后，navigation 节点的 lifecycle 管理器常因 map TF 延迟而超时失败，
需手动激活。**直接跑 `bash ~/aic/nav_start.sh` 即可**（内含下面全部步骤）：

```bash
source ~/aic/install/setup.bash

# 1. 激活定位
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl activate

# 2. 设置 AMCL 初始位姿（机器人起点 (3.88, 3.88)，朝西 yaw=π）
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 3.88, y: 3.88}, orientation: {z: 1.0, w: 0.0}}}}"

# 3. 激活导航节点
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /smoother_server activate
ros2 lifecycle set /behavior_server activate
ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /velocity_smoother activate
ros2 lifecycle set /collision_monitor activate
```

> 注意：`nav_start.sh` 里初始位姿改用 Python 连发 3 次并带非零协方差，比上面这条
> `--once` 更可靠（慢虚拟机下 `--once` 可能丢失导致车头偏转）。生产/比赛直接用脚本。

## 七、文件结构

```
aic/
├── nav_start.sh                     # 终端2：激活 lifecycle + 发布初始位姿
├── src/
│   ├── robot_description/           # 机器人描述包
│   │   ├── urdf/robot.xacro         # 差速机器人模型（LiDAR/相机/IMU/插件）
│   │   ├── config/nav2_params.yaml  # Nav2 参数
│   │   ├── launch/
│   │   │   ├── bringup.launch.py    # GUI 完整启动（世界+机器人+红绿灯+Nav2）
│   │   │   ├── nav_bringup.launch.py # 无头导航（巡检推荐）
│   │   │   ├── patrol.launch.py     # 视觉识别 + 巡检调度
│   │   │   ├── mapping.launch.py    # 无头蛇形建图
│   │   │   ├── slam.launch.py       # 单独启动 SLAM
│   │   │   ├── nav2.launch.py       # 单独启动 Nav2
│   │   │   └── spawn_robot.launch.py
│   │   └── scripts/
│   │       ├── frame_remap.py       # frame_id 前缀重映射 + odom→base_link TF
│   │       ├── traffic_light_detector.py # 红绿灯颜色识别（HSV 分割）
│   │       ├── vision_detector.py   # 人偶身份识别 + 车牌 OCR（/vision/trigger 按需触发）
│   │       ├── waypoint_patrol.py   # 航点巡检调度（逐点导航 + 触发识别/等灯/统计）
│   │       ├── coverage_mapping.py  # 蛇形全覆盖自动建图
│   │       ├── wasd_teleop.py       # WASD 遥控
│   │       ├── reset_robot.py       # 重置机器人位置
│   │       ├── auto_mapping.py      # 自动避障建图（有漂移，不推荐）
│   │       └── explore_mapping.py   # 探索式建图
│   ├── vision_training/             # 视觉训练/推理（非 ROS 包，人偶+车牌模型）
│   └── smart_community_gazebo/      # 场景包
│       ├── worlds/smart_community_v2.sdf   # 场地世界
│       ├── models/                  # 人偶/红绿灯/车辆模型
│       ├── config/official_route.yaml       # 巡检航点（含每点 action）
│       ├── maps/                    # 建好的地图（community.pgm + yaml）
│       └── scripts/traffic_light_controller.py # 红绿灯动态切换
└── README.md                        # 本文档
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
11. **Nav2 velocity_smoother 输出是 `/cmd_vel_smoothed`**，经 collision_monitor 后再桥接到 gz 的 cmd_vel
12. **lifecycle 管理器易超时**（VMware 慢），需手动激活节点
13. **gz GUI 卡死**：改用无头模式（`gz sim -s`）稳定
14. **gz 里程计以出生点为原点**：odom 一开始是 (0,0,yaw=0)，而地图里车头朝西（yaw=π），
    靠 `/initialpose` 对齐 map→odom。初始位姿要**带协方差、连发多次**，否则慢虚拟机下
    单次 `--once` 丢失会导致 AMCL 朝向错误、启动时车头偏转、错过第一个红绿灯
15. **相机朝向即车头 +X 方向**：巡检航点若按「行进方向」给 yaw，会导致相机背对识别目标；
    车牌/人偶识别点要显式让车头朝目标（车牌朝 +X、A 街区朝 +X、B 街区朝 −X）
16. **第二个红绿灯识别不稳定**：`waypoint_patrol.py` 里 `_green_seen` 已实现「识别到第一个绿灯后跳过后续等待」

## 九、已知问题 & 下一步

1. **IMU 传感器被 gz-sim 静默丢弃**（日志无报错），待排查
2. **第二个红绿灯等待已跳过**：只靠第一盏灯判断通行（第二盏相机视角不理想），如需完全合规需调整航点或加装传感器
3. **人偶/车牌识别依赖相机正对**：偏离目标过远或角度过大时可能漏检，可调整航点站位或识别距离阈值
4. **自动避障建图 odom 漂移**：差速机器人频繁转向打滑，推荐 `coverage_mapping.py` 慢速蛇形建图

## 十、参考

- 场景建模说明：`src/smart_community_gazebo/README.md`
- 建模团队交接：`src/smart_community_gazebo/TEAM_HANDOFF.md`
- 任务要求：`docs/复赛资料/任务要求.txt`
