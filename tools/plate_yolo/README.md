# AIC 2026 智慧社区：YOLO 车牌检测 + 现有 SVM 车牌内容分类（v1）

**基于 2026-10-13 的 `feature/nav-v5-integration` 审计文件设计。此压缩包是可执行的训练和接入工具，**不是已经训练好的 `best.pt`，也不含官方参赛素材或比赛照片。运行前备份你的项目；`patch_v5.py` 默认只输出修改预览，必须显式 `--apply` 才会修改。

## A. 模型范围（先看）

- **YOLO 单类别 `license_plate`：**在 Gazebo RGB 相机图像中检测车牌矩形，输出位置和检测置信度。
- **现有 `plate_classifier.xml`：**对 YOLO 裁出的蓝牌图片进行三类内容分类，输出 `inspection_models/metadata.json` 里已有的三个中文车牌文本。*它不是能够读任意车牌字符的 OCR；评分时必须在真实摄像头截帧上核对是否正确。*
- 红绿灯识别代码、`/aic/traffic/observation`、Nav2、17 个航点不做修改。车牌 YOLO 只在 `PLATE_1/2/3` 检查点调用，避免给 5 秒/15 秒灯控节点添加持续推理负载。
- 未提供权重时原有 HSV+SVM 车牌识别仍可工作。

## B. 安装（建议在 Ubuntu 24.04 的一个新 venv 操作）

把本文件夹放在 `~/aic_ws/tools/plate_yolo/`。Ubuntu 终端：

```bash
sudo apt update
sudo apt install -y python3-venv python3-opencv python3-numpy
python3 -m venv --system-site-packages ~/aic_ws/.venv_plate
source ~/aic_ws/.venv_plate/bin/activate
python -m pip install -U pip
python -m pip install ultralytics pillow
python -c 'import cv2, rclpy, ultralytics; print("cv2/ROS/YOLO modules are ready")'
```

**注意：** VMware 虚拟机一般没有 NVIDIA CUDA 直通；在 VMware 用 `--device cpu` 训练可能需要较长时间。有 NVIDIA GPU 的 Windows 宿主机或 Colab 可加速训练，再把 `best.pt` 拷回 Ubuntu。训练需联网下载首次使用的官方检测预训练权重 `yolo11n.pt`；用本地模型文件可避免下载。Ultralytics 使用许可请在公开发布前确认。

## C. 准备数据（最重要；不能用 3 张官方平面车牌素材冒充 Gazebo 训练结果）

需要**机器人真实 RGB 相机图像**，包括三处停车位不同距离（例如停车前后）、角度、光照、背景；建议至少 80～150 张去重后标注的含牌图像，再加入一些没有车牌的负样本。数量是起步建议，**不能保证精度**。最好来自 >=3 个独立录制 session，不要只拿同一帧复制几十份。

先采集现有日志里不同会话的 `camera_*.jpg`：

```bash
cd ~/aic_ws/tools/plate_yolo
python collect.py --results ~/aic_ws/aic_results --raw ~/aic_ws/plate_yolo_work/raw --per-session 40
python annotate.py --raw ~/aic_ws/plate_yolo_work/raw
```

逐图鼠标**拖一个矩形只框住蓝色车牌**，按 `n` 保存并下一张；没有车牌的图按 `0` 标为空背景；模糊/看不到车牌不确定的按 `s` 跳过；`r` 清空框；`q` 退出。标注只会写入 `plate_yolo_work/raw/labels/*.txt`，不会更改原始相机截图。若采集结果没有清晰拍到车牌，先到 `PLATE_1,2,3` 检查点真实保存照片或视频截图，再放进 raw/images（至少另录一次会话，建议文件名格式 `session_id__camera_001.jpg`）。

数据完成后分成 train/val。**默认按录制会话划分，不允许同一会话同时进入训练与验证**；只有一个会话时会拒绝，除非显式 `--allow-frame-split`（这样验证分数可能虚高）：

```bash
python split.py --raw ~/aic_ws/plate_yolo_work/raw --out ~/aic_ws/plate_yolo_work/dataset
```

确认生成 `~/aic_ws/plate_yolo_work/dataset/plate.yaml`，格式固定为单类别：`0: license_plate`。若需要重做分割，请手动备份/删除上一次 `plate_yolo_work/dataset` 或使用新的 `--out`，避免混入旧文件。

## D. 训练 YOLO（**这一步完成之前没有真正的 YOLO 车牌模型**）

```bash
cd ~/aic_ws/tools/plate_yolo
python train.py --data ~/aic_ws/plate_yolo_work/dataset/plate.yaml \
  --model yolo11n.pt --epochs 60 --imgsz 640 --batch 8 --device cpu
```

如果在实际有 CUDA 的设备上训练，改成 `--device 0`。训练过程中会出现 `runs/plate_detect*/weights/best.pt`，训练完成后打印 `BEST_WEIGHTS` 绝对路径与验证集 `mAP50`, `mAP50_95`, precision/recall；这些是**车牌框检测指标，不是车牌内容识别准确率**。当验证集来自另一次 Gazebo 场景录制，指标才更有参考价值。

## E. 独立验证整条车牌识别链（无需启动 Gazebo/Nav2）

对未参与训练的一批新图片运行：

```bash
python verify.py \
  --workspace ~/aic_ws \
  --weights ~/aic_ws/plate_yolo_work/runs/plate_detect/weights/best.pt \
  --images ~/aic_ws/plate_yolo_work/my_heldout_images
```

**以上权重路径仅是示例；如果训练保存到了 `plate_detect2` 等目录，请填真实的 `BEST_WEIGHTS` 路径。**

输出 `plate_yolo_work/verification/results.json` 以及带检测框照片（文字结果在 JSON）。和原图逐一核对：是否框住正确车牌、有没有识错号码、是否对 3 个检查点都有效。

## F. 合并到现有 ROS 2 v5.6（不会改红绿灯权限）

先确认当前分支/改动，建议提交或者备份：

```bash
cd ~/aic_ws
git status --short
python3 tools/plate_yolo/patch_v5.py --workspace ~/aic_ws     # dry-run，只预览差异
```

检查 patch 输出合理之后，再主动应用：

```bash
python3 tools/plate_yolo/patch_v5.py --workspace ~/aic_ws --apply
python3 -m py_compile src/robot_description/scripts/aic_v5_vision.py src/robot_description/launch/aic_v5.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_description
source install/setup.bash
```

每个被改变的文件都会留下 `.plate_backup_年月日_时分秒` 备份。**脚本使用精确锚点；如果你的文件和 10 月 13 日审计版本有差异，会拒绝修改而不是强行覆盖。**

使用已有 v5.6 一体化启动（不能重复启动第二个 `/aic/traffic/observation` 发布者）：

```bash
# 先激活能 import ultralytics + rclpy 的虚拟环境
source ~/aic_ws/.venv_plate/bin/activate
source ~/aic_ws/install/setup.bash

# Gazebo/Nav2 与已配置场景都尚未启动时，只能在正常启动流程中使用下面命令
ros2 launch robot_description aic_v5.launch.py \
  plate_yolo_weights:=/填写实际best.pt绝对路径
```

或者，当 Gazebo/Nav2 **已经正常运行且机器人处于静止状态**，继续使用团队已有 `tools/aic_one_run.sh`（它会启动视觉然后巡检，不要与上面的 launch 一起用）：

```bash
export AIC_PLATE_YOLO_WEIGHTS=/填写实际best.pt绝对路径
bash ~/aic_ws/tools/aic_one_run.sh
```

ROS 日志在 `~/aic_ws/aic_results/vision_integrated.log` 和 `vision_*/inspection_events.jsonl`；看 `plate_detector=YOLO_CUSTOM_PLATE`、`plate_predictions` 的 `text` 与 `bbox`。即使有 `MODEL_INFERENCE_COMPLETED` 也不是正确识别的证明，必须对照三张牌号人工核验。未检测到车牌会写 `NO_PLATE_DETECTED`，**不会伪造识别结果**。

## G. 注意事项

1. 请勿直接把 COCO 预训练 `yolo11n.pt` 当成已完成车牌检测权重；COCO 不带本工程专门的 `license_plate` 类。
2. 本包仅添加一个选配的 YOLO 检测分支，不能保证摄像头在导航检查点已经拍清车牌；车辆停位与朝向仍需实际核对。
3. 与 ROS Jazzy 一起运行时请确认 venv 可以 `import rclpy`；若出现 `numpy` 二进制兼容错误，请停止，不要往系统 Python 全局强制 pip 安装/卸载包。
4. 当前是 3 种固定车牌的分类方案。若要求**任意车牌通用文字 OCR**，需要另加字符识别与标注训练，不能把三分类结果宣称为通用 OCR。
5. 若你将数据/代码提供给我继续改进，请上传 **30～50 张真实 Gazebo 相机含车牌照片 + 对应文字真值 + 这套工具训练结果或日志**（不要只上传静态车牌 PNG）。
