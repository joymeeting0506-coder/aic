# Ubuntu 虚拟机 + ROS 2 + CPU 部署

本包订阅 Gazebo 相机的 ROS 2 `sensor_msgs/msg/Image`，发布车牌识别 JSON。无需修改 URDF、Gazebo 车辆模型或导航代码。当前没有连接你的 Ubuntu，ROS 节点只做了语法检查和图像转换逻辑测试，需按下面步骤实机验证。

## 1. 复制并解压

将 Windows 的 `aic_lpr_ros2_cpu.zip` 复制到 Ubuntu 的 `~/Downloads`（共享文件夹、拖拽或 scp 均可）。

```bash
sudo apt-get install -y unzip
mkdir -p ~/aic_models
unzip ~/Downloads/aic_lpr_ros2_cpu.zip -d ~/aic_models
cd ~/aic_models/aic_lpr_ros2_cpu
```

目录内必须同时有 `models/plate_detector.pt`、`models/characters.pt`、`common.py`、`predict.py`、`ros_image.py`、`ros2_plate_node.py`。

## 2. 确认 ROS 版本并安装依赖

```bash
ls /opt/ros
```

例如输出 `humble`，执行 `source /opt/ros/humble/setup.bash`；输出 `jazzy` 就改为 `source /opt/ros/jazzy/setup.bash`。每个新终端都需 source，或者使用已有的比赛工作空间环境。

```bash
echo "$ROS_DISTRO"
source /opt/ros/humble/setup.bash  # 示例：替换为你的实际发行版
cd ~/aic_models/aic_lpr_ros2_cpu
bash install_ubuntu_cpu.sh
```

安装脚本适用于 Ubuntu 22.04/24.04 的系统 Python 3.10～3.12。其他版本先提供 `cat /etc/os-release`、`ls /opt/ros`、`/usr/bin/python3 --version`。它创建独立 `.venv` 并保留系统 ROS Python 可见性，不更改系统 Python 的 pip 包。PyTorch 使用[官方 CPU 软件源](https://pytorch.org/get-started/locally/)，不需要把 Windows CUDA 环境复制过去。

本节点直接解析原始 8 位相机图像，支持 bgr8/rgb8/mono8/bgra8/rgba8 和带行填充的数据，不依赖 cv_bridge。深度图、Bayer 图或压缩图不能作为默认输入。

## 3. 先验证模型离线读号

```bash
cd ~/aic_models/aic_lpr_ros2_cpu
.venv/bin/python predict.py --source sample_images --out smoke_predictions --device cpu
```

查看 `smoke_predictions/predictions.json`。前三张来源图应为 `皖BK3P58`，第四张应为 `苏APL12A`。这一步只确认迁移后的推理环境；CPU/CUDA 的浮点差异可能影响边界置信度，应检查实际输出。

## 4. 启动原有 Gazebo 小车并找到相机话题

按你原本的比赛启动方式运行 Gazebo。另开终端、source 同样的 ROS/比赛环境：

```bash
ros2 topic list -t
```

找 `[sensor_msgs/msg/Image]` 类型的**彩色相机**话题，例如 `/camera/image_raw`、`/camera/color/image_raw` 或带机器人前缀的名称。不要选择 `/camera_info`、深度图或 `/compressed`。

```bash
ros2 topic info /camera/image_raw --verbose
```

其中 `/camera/image_raw` 只是示例，后面全部换成你的实际名称。如果完全没有 Image 话题，需要先解决现有 Gazebo 相机插件或 ROS–Gazebo 图像桥接，识别节点本身不能创建相机。

## 5. 启动识别

```bash
source /opt/ros/humble/setup.bash  # 按实际发行版修改
# 如果比赛使用了 ROS_DOMAIN_ID/RMW_IMPLEMENTATION，使用与 Gazebo 一致的设置
cd ~/aic_models/aic_lpr_ros2_cpu
.venv/bin/python ros2_plate_node.py --ros-args \
  -p image_topic:=/camera/image_raw \
  -p device:=cpu \
  -p rate_hz:=2.0 \
  -p cpu_threads:=2
```

最多每秒处理 2 帧，实际速度取决于虚拟机 CPU。相机订阅采用 best-effort、depth=1，只保留最新一帧，避免推理较慢时积压相机队列。该 QoS 可接收 best-effort 或 reliable 相机发布者；参见 [ROS image_pipeline 官方说明](https://github.com/ros-perception/image_pipeline/blob/rolling/image_pipeline/doc/tutorials.rst)。

## 6. 查看识别输出

再开终端并 source ROS：

```bash
ros2 topic echo /plate_recognition/result
```

话题类型为 `std_msgs/msg/String`，data 是 JSON，示意如下（数值仅是格式示例）：

```json
{"status":"ok","header":{"frame_id":"camera_link","stamp":{"sec":12,"nanosec":0}},"inference_ms":250.0,"plates":[{"text":"皖BK3P58","format_valid":true,"needs_review":false,"plate_confidence":0.9,"min_character_confidence":0.8}]}
```

无车牌时 `plates: []`；没有相机输入时 `status: waiting_for_camera`；处理错误时 `status: error`。结果携带原始图像时间戳。消费结果时要检查状态、图像时间戳、`needs_review` 和号码；不要反复使用旧结果。

接到比赛逻辑时，订阅该结果话题，解析 `json.loads(msg.data)`，取 `plates[*].text`。建议经过连续多帧一致性判断再用于任务决策。当前节点只做识别，不发布速度、停车或开闸指令。要替换原比赛视觉节点的输入，还需其订阅话题、消息类型或源代码。

## 常见问题

- `No module named rclpy`：先 source ROS，并使用包内 `.venv/bin/python`；不要用 Conda 的不同版本 Python。
- `No module named ultralytics/torch`：安装脚本未成功或启动时用了错误的 Python。
- 一直 `waiting_for_camera`：核对相机话题及类型、Gazebo 是否运行、两个终端的 `ROS_DOMAIN_ID` 是否一致；图像桥接需先工作。
- 识别很慢：将 `rate_hz` 改成 `1.0`，并给虚拟机分配足够 CPU/内存；`rate_hz` 是上限，不是保证帧率。无需在虚拟机中重新训练。
- 字号太小或远处读错：让车靠近或提高有效相机分辨率，再保存新场景用于评估。现有合成测试分数不代表比赛泛化能力。
- 标注图片中文显示方框：只是可视化字体问题，JSON 中文号码不依赖字体。
