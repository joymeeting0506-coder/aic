# joymeeting0506-coder/aic 专用部署步骤

已核对仓库 `https://github.com/joymeeting0506-coder/aic` 的提交 `ec9f590919c1dc8ab1521e416b3e59c50fcefe3f`。原链接 `https://github.com/joymeeting0506-coder/-` 当前重定向到该仓库。此文根据源码确定接口，不等于已连接用户的 Ubuntu 实测。

## 已确认的接口

| 项目 | 源码配置 |
|---|---|
| 系统 | Ubuntu 24.04，ROS 2 Jazzy，Gazebo Harmonic |
| 相机 | robot.xacro 中的 camera，640×480 |
| Gazebo 相机话题 | camera |
| ROS 原始图像 | /camera/image_raw |
| frame_remap 转发图像 | /camera/image_clean，frame_id 映射为 camera_link |
| 新 YOLO 节点输出 | /plate_recognition/result，std_msgs/msg/String，JSON |

`bringup.launch.py` 已包含相机桥接和 frame_remap。`nav_bringup.launch.py` 启动了 frame_remap，但桥接列表只有速度、里程计、雷达和时钟，**缺少相机**。使用导航启动时需要单独补相机桥接。

现有 `src/vision_training/vision_app.py` 用 HOG/SVM 做识别，输入来自 `cv2.VideoCapture` 或图片文件；`--source 0` 是普通摄像头设备，不是 Gazebo ROS 相机。把新 `.pt` 文件放进去，不会让该程序自动切换为 YOLO。本部署包通过独立 ROS 节点接入同一相机，现有人物、红绿灯和导航程序继续按原方式运行。

## 一次性安装

把更新后的 `aic_lpr_ros2_cpu.zip` 复制到 Ubuntu `~/Downloads`：

```bash
mkdir -p ~/aic_models
unzip ~/Downloads/aic_lpr_ros2_cpu.zip -d ~/aic_models
source /opt/ros/jazzy/setup.bash
cd ~/aic_models/aic_lpr_ros2_cpu
bash install_ubuntu_cpu.sh
.venv/bin/python predict.py --source sample_images --out smoke_predictions --device cpu
```

如果已安装过上一版，可在解压时确认覆盖包内脚本；保留 `.venv` 即可，无需删除环境或重新训练。

## 终端 A：启动仿真

如果 Gazebo 小车已经运行，沿用该实例。若尚未启动，可用仓库已有的 GUI 启动文件：

```bash
source /opt/ros/jazzy/setup.bash
source ~/aic_ws/install/setup.bash
ros2 launch robot_description bringup.launch.py render_mode:=sensor_safe
```

`~/aic_ws` 是仓库根 README 的示例工作空间；如果你实际在 `~/aic` 或别处编译，请替换为实际 `install/setup.bash`。保留你已验证可用的渲染模式；此处 `sensor_safe` 是仓库支持的 VMware 软件渲染模式。

## 仅导航启动时：终端 B 补相机桥接

若运行的是 `nav_bringup.launch.py`，或检查发现 `/camera/image_raw` 没有发布者，在独立终端运行：

```bash
cd ~/aic_models/aic_lpr_ros2_cpu
bash bridge_aic_camera.sh
```

使用当前仓库的 `bringup.launch.py` 时不需要这一步，它已经有同一桥接。可以先执行 `ros2 topic info /camera/image_raw --verbose` 确认发布者，避免重复桥接。相机桥接只能转发已有图像，如果 Gazebo 的 camera 传感器本身没有输出，还需要检查仿真传感器及渲染环境。

## 终端 C：启动 YOLO 识别

```bash
cd ~/aic_models/aic_lpr_ros2_cpu
bash start_aic_jazzy.sh
```

默认读取 `/camera/image_clean`、使用 CPU 和两线程、最高每秒处理两帧。此脚本只加载环境和启动识别，不启动 Gazebo，也不发布运动指令。

如果没有启动 frame_remap，但 `/camera/image_raw` 正常，可显式改读原始图像：

```bash
bash start_aic_jazzy.sh -p image_topic:=/camera/image_raw
```

如果工作空间不是 `~/aic_ws`：

```bash
AIC_WORKSPACE=~/aic bash start_aic_jazzy.sh
```

所有终端的 ROS_DOMAIN_ID 和 RMW_IMPLEMENTATION 应与现有仿真环境保持一致，脚本不会重设它们。

## 终端 D：查看结果

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /plate_recognition/result
```

JSON 中 `status=ok` 且 `plates` 非空时，`plates[*].text` 为号码。检查 `needs_review` 与原始图像时间戳后再交给任务节点；建议多帧一致性判断。`waiting_for_camera` 表示检查话题/桥接，`plates=[]` 表示当前帧未检出车牌。

## 验证范围

已完成模型在本机 CPU 上的四图推理、ROS 原始图像行填充/颜色转换测试及节点 Python 语法检查。包内 shell 脚本、ROS 话题通信、Gazebo 运行和虚拟机实际性能仍需在 Ubuntu 验证。没有修改或推送远程仓库，没有自动替换原 SVM 程序或任务决策节点。

源码依据：

- https://github.com/joymeeting0506-coder/aic/blob/ec9f590919c1dc8ab1521e416b3e59c50fcefe3f/src/robot_description/launch/bringup.launch.py
- https://github.com/joymeeting0506-coder/aic/blob/ec9f590919c1dc8ab1521e416b3e59c50fcefe3f/src/robot_description/launch/nav_bringup.launch.py
- https://github.com/joymeeting0506-coder/aic/blob/ec9f590919c1dc8ab1521e416b3e59c50fcefe3f/src/robot_description/scripts/frame_remap.py
- https://github.com/joymeeting0506-coder/aic/blob/ec9f590919c1dc8ab1521e416b3e59c50fcefe3f/src/vision_training/vision_app.py
