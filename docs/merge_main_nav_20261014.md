# main 与最新导航分支合并说明

合并基础：main 017b7dd 与 feature/nav-v5-integration a565644。

地图按要求完整保留导航分支的 community.pgm/community.yaml；原点为
[-0.344, -0.360, 0]，map 坐标中的 START 是 (0,0,0)。官方航点文件仍是
Gazebo 世界坐标，两个巡检程序在发送目标前均转换到 START 局部坐标。
nav_start.sh 不再引用队友电脑的 /home/biao/aic；只用于起点处的新仿真。

按用户要求统一采用 22 点观察路线，官方路线文件 official_route.yaml
现在包含这 22 点。START/FINISH 的世界坐标统一为保存地图所用的
(3.884,3.886)，其他观察位置与队友版本一致；未修改地图。
team_inspection_route.yaml 保留队友原始路线作为对照。

V5 的 A_BOTTOM_VIEW/B_TOP_VIEW 分别拍照，附加转向点保留。拍照时使用
路线中的观察朝向，随后恢复沿道路前进的朝向，防止下一段 FollowPath
因车头朝着目标而拒绝启动。22 点含 START/FINISH；V5 另有两个红灯前
停车点，START 不重复发送，所以打印 23 个导航目标。
中段 B_RIGHT_VIEW 仍沿用 V5 的红灯前代理观察（MID_STAGING，提前
10 cm），并在绿灯后实际经过 B_RIGHT_VIEW；日志明确记录此差异。

保留双方脚本、视觉模型、训练资源及 YOLO CPU 部署包。构建安装列表和
Python/ROS 依赖取并集，并移除重复声明。

bringup 的 start_navigation/start_traffic_detector 默认 true，保留队友的
入口行为。V5/v3 导航及 SLAM 全流程入口显式关闭这两个开关，防止重复
Nav2、map_server、AMCL 或 traffic_light_detector。

保留小车缩放能力：队友入口为 scale=0.5，V5/v3 和 START 覆盖入口明确用
scale=1.0，与各自既有的代价地图及 V5 审计包络相符。只支持这两个尺寸。

推荐继续验证已使用的 V5 路径，在全新仿真环境运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_description aic_v5.launch.py
```

另一终端：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_description aic_v5_patrol.py --mode preview
ros2 run robot_description aic_v5_patrol.py --mode patrol --ack-official-crossing
```

队友的巡检实现与 V5 是独立入口，不应同时启动两个调度器。
队友程序原有的第二个红绿灯跳过逻辑和超时通行逻辑仍需后续修复；
本次保留其功能，不把它认定为比赛合规。车牌 CPU 节点仍是独立部署包，
本次合并不等于已经接入每个巡检点的结果汇总。

一并纳入先前准备的绿灯缓存过期修复：先完成路径计算，再在停车状态下
确认四张新鲜且不同的稳定绿灯图像，随后才向 Nav2 分派路径；保持原有
运行中消息过期和红黄灯保护，没有扩大通行阈值。

验证：Python/XML/YAML 解析、安装脚本存在性、地图字节一致性、航点转换、
启动开关与尺寸、V5 路线离线占用审计。本机是 Windows，Ubuntu colcon
构建和完整 Gazebo 巡检尚未实测。

可复现兼容检查：`python3 tools/check_merge_compatibility.py`。
