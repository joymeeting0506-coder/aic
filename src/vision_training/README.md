# 仿真视觉训练与识别

此目录提供基于仓库素材的轻量 OpenCV 训练程序和视频/摄像头推理程序。人物模型取自 `src/smart_community_gazebo/models/*person*/meshes/person_board.png`，身份标签取自 `person_manifest.csv`；车牌图片及文字取自 `models/plate_1..3` 和 `layout_manifest.json`。

## 训练

系统需要已有的 Python 3、NumPy 和 OpenCV（当前 Ubuntu 环境提供 `python3`, `numpy`, `cv2`）：

```bash
cd /home/cyan/aic/vision_training
python3 train_models.py
```

训练会在 `trained_models/` 内生成三个 OpenCV SVM 模型和 `metadata.json`。默认使用每张人物素材 60 个增强样本，并对每张车牌生成 240 个增强样本；可用 `--person-samples`、`--plate-samples` 和 `--seed` 调整。

## 运行

USB 摄像头：

```bash
python3 vision_app.py --source 0
```

视频文件或单张图片：

```bash
python3 vision_app.py --source /path/to/inspection.mp4
python3 vision_app.py --source /path/to/frame.jpg --output annotated.jpg
```

验证已训练的社区身份分类模型（逐张读取素材包内 18 个立牌纹理，并输出 CSV 和红/绿框结果图）：

```bash
python3 validate_identity.py
```

默认报告写入 `identity_validation.csv`，结果总览图写入 `identity_validation.jpg`；可用 `--output` 和 `--image` 指定路径。绿框表示社区人员，红框表示非社区人员。图中置信度是基于素材特征相似度的估算值，并非校准概率。该验证把立牌纹理直接送入训练好的身份 SVM；由于这些纹理参加过模型训练，结果不代表未见图像的泛化能力，也不衡量整幅场景里的人员检出率。

社区人员用绿框，非社区人员用红框；车牌不画框，程序会在原图右侧扩展独立结果面板，显示中文车牌号码，终端也会同步打印结果。中文结果使用系统 CJK 字体绘制，省份简称不会再显示为问号。SVM 阈值可用 `--threshold` 调整。只测车牌时增加 `--plates-only`，可跳过耗时的人物滑窗扫描。

## 适用范围

项目只提供 18 张人物立牌纹理、3 张车牌纹理和仿真资源，没有真实相机图像及人工框标注。训练程序用亮度、透视、模糊和噪声增强这些素材，并以地图/场景纹理生成背景负样本。人物类别对应当前素材中的社区/非社区身份，车牌识别范围是场景内的三张已知号牌（苏A·B8Q62、苏A·Q7M26、皖B·K3P58）。这是一套用于当前仿真素材的原型，遇到真实行人、不同车牌或显著侧视遮挡时需要补充真实标注数据并重新训练。

外部中国车牌样本与实测记录见 [`external_plate_samples/README.md`](external_plate_samples/README.md)。当前固定号码分类器在 CCPD 外部图片上没有通过号码识别测试。
