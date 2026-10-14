# AIC 2026：从没有标注数据开始训练 YOLO 车牌检测（v2）

这个版本新增：**自动合成车牌检测数据集并生成 YOLO 标签**，不需要先做手动标注即可进行一次**流程验证**。但**只有合成图片训练出的模型不等于比赛中能识别车牌**；最终必须加入并验证 `/camera/image_raw` 的实际 Gazebo 图片。合成的车牌文本仅用于模拟训练，不是对官方三张车牌的真实识别结果。

**模型定义：单类别 `license_plate`，只检测位置。** 现有的 3 种固定仿真车牌分类继续调用 SVM；若更换为任意随机车牌则需要专门 OCR，YOLO 检测框本身不会读出数字。详情参见原有 `README.md`。

## 0. 解压放置

```bash
mkdir -p ~/aic_ws/tools/plate_yolo
# 将本工具包 ZIP 解压进 ~/aic_ws/tools/plate_yolo 后，确保以下脚本直接存在该目录
cd ~/aic_ws/tools/plate_yolo
ls generate_synthetic.py train.py collect.py annotate.py split.py mix_datasets.py
```

**只装到新虚拟环境，不动 ROS 系统 Python：**

```bash
sudo apt install -y python3-venv python3-opencv python3-numpy fonts-noto-cjk fonts-dejavu-core
python3 -m venv --system-site-packages ~/aic_ws/.venv_plate
source ~/aic_ws/.venv_plate/bin/activate
python -m pip install ultralytics pillow
```

注意：仅为了运行 `generate_synthetic.py` 只需要 numpy、OpenCV、Pillow 与 CJK 字体；训练需要 `ultralytics` 和 PyTorch。

## 1. 无标注数据也能生成标准 YOLO 检测数据集

```bash
cd ~/aic_ws/tools/plate_yolo
python generate_synthetic.py --train 600 --val 120
```

得到：

```text
~/aic_ws/plate_yolo_work/dataset_synthetic/
├── images/train/*.jpg
├── images/val/*.jpg
├── labels/train/*.txt     # 一张图一个 YOLO 格式 0 xcenter ycenter width height
├── labels/val/*.txt
├── synthetic_ground_truth.csv  # 图上的模拟牌号，可用于校对，不是 OCR 训练标签
└── plate.yaml
```

可先用 `python generate_synthetic.py --train 30 --val 10 --out ~/aic_ws/plate_yolo_work/quickcheck` 快速试运行。**脚本拒绝写入非空同名目录**，重复运行要换 `--out` 目录以免污染旧数据。

合成背景是简化的卡通车尾与社区建筑。其作用是帮助打通训练和数据标注的管道；与 Gazebo 的真实车辆纹理存在明显**域差异**。请勿把合成验证集的 mAP 用作复赛“识别率”。不推荐仅用此合成权重提交比赛。

可加 `--background-dir /路径/不含车牌的Gazebo背景图`，但文件中**绝不能有未标注的真实车牌**，否则会引入错误负标签。

## 2. 尽快增加真实 Gazebo 标注（强烈建议）

你已经有 `~/aic_ws/aic_results/vision_*/camera_*.jpg`，但是不少帧可能不是车牌视角。优先从 PLATE_1、PLATE_2、PLATE_3 路段真实采集。尽量使用至少 2 个独立录制会话，包含近/中/远视角与转向姿态，不能拿连续重复的同一帧冒充独立样本。

```bash
python collect.py --results ~/aic_ws/aic_results --raw ~/aic_ws/plate_yolo_work/raw --per-session 30
python annotate.py --raw ~/aic_ws/plate_yolo_work/raw
```

标注界面：鼠标拖框仅框蓝色车牌，`n` 保存；没有牌 `0`；不确定 `s` 跳过。注意**不要标整个汽车或只有纯车牌图片**，检测框对应相机图片中的真实车牌部分。

```bash
python split.py --raw ~/aic_ws/plate_yolo_work/raw --out ~/aic_ws/plate_yolo_work/dataset_real
python augment_real.py --dataset ~/aic_ws/plate_yolo_work/dataset_real --copies 5
python mix_datasets.py \
  --synthetic ~/aic_ws/plate_yolo_work/dataset_synthetic \
  --real ~/aic_ws/plate_yolo_work/dataset_real \
  --out ~/aic_ws/plate_yolo_work/dataset_mixed
```

混合数据集**只包含来自独立录制 session 的真实 Gazebo 验证集**，训练集可以是“真实 + 合成”，不会将合成验证集混入指标。

## 3. 训练、验证

```bash
python train.py --data ~/aic_ws/plate_yolo_work/dataset_mixed/plate.yaml \
  --model yolo11n.pt --epochs 60 --imgsz 640 --batch 8 --device cpu
```

如果有 Windows 主机 NVIDIA GPU，建议在宿主机或云端做训练（把整套数据目录复制过去，并调整 `plate.yaml` 的 `path`）。VMware 通常无法直接使用宿主机 GPU 来训练，CPU 可能需要很久。

运行 `verify.py --workspace ~/aic_ws --weights /真实best.pt路径 --images /独立实际车牌图片目录` 验证 YOLO 检测 + 现有 SVM 字符内容分类的整条链。只有真实结果正确之后，再执行 `patch_v5.py` 的 dry-run 与 `--apply`，见主 README。

**提交前必须保留 3 处车牌的原始相机截图、检测框、检测置信度以及识别号码核验记录**。既不能仅展示合成训练图片，也不能把生成时知道的车牌字符串冒充 OCR 实际识别。

## 4. 一天内优先级

1. 先确认 3 个车牌巡检点能拍清楚完整蓝牌（越清晰越好）。
2. 采集至少 2 段不同时间/视角的 Gazebo 视频或图片，标注真实图片，训练并验证单类别 YOLO。
3. 若限时内 YOLO 效果不好，保留现有 HSV/SVM 三车牌识别作为演示兜底，并如实说明其适用边界。不要为了 YOLO 修改红绿灯安全控制或破坏已通过的导航链。
