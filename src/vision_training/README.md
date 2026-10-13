# 仿真视觉训练与识别

此目录提供基于仓库素材的轻量 OpenCV 训练程序和视频/摄像头推理程序。人物模型取自 `src/smart_community_gazebo/models/*person*/meshes/person_board.png`，身份标签取自 `person_manifest.csv`；车牌图片及文字取自 `models/plate_1..3` 和 `layout_manifest.json`。

## 训练

系统需要已有的 Python 3、NumPy 和 OpenCV（当前 Ubuntu 环境提供 `python3`, `numpy`, `cv2`）：

```bash
cd /home/biao/aic/src/vision_training
python3 train_models.py            # 人物检测/身份 + 整牌模板（回退用）
python3 train_plate_ocr.py         # 车牌逐字符 OCR（省简称 31 类 + 字母数字 34 类）
```

- `train_models.py` 生成 `person_detector.xml`、`person_identity.xml`、`plate_classifier.xml` 和 `metadata.json`。默认每张人物素材 60 个增强样本，每张车牌 240 个增强样本；可用 `--person-samples`、`--plate-samples` 和 `--seed` 调整。
- `train_plate_ocr.py` 用系统 CJK/Latin 字体合成省简称与字母数字样本，叠加透视/亮度/旋转/宽高比增强，并混入三张仿真车牌里真实切出的字符样本，训练 `province_classifier.xml`（31 类）和 `char_classifier.xml`（34 类）。可用 `--samples`（每类合成样本数）、`--per-real`（每个真实字符增强数）和 `--seed` 调整。

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

## 车牌 OCR

车牌识别采用「定位 → 透视矫正 → 分割 7 个字符 → 逐字符 HOG/SVM 识别」的真 OCR 管线（`plate_ocr.py`），能读出任意中国蓝牌的 `[省简称][字母]·[5 位字母数字]` 号牌，不再局限于场景内固定的三张号牌。识别流程：

1. `locate_plate`：蓝色 HSV 掩码 + 轮廓筛选定位号牌，用 `minAreaRect` 取四角并透视矫正到归一化尺寸；
2. `segment_characters`：白字掩码（含 Otsu/自适应阈值兜底）→ 连通域剔除空心边框 → 形态学闭合重连省简称笔画 → 按高度剔除间隔圆点与铆钉 → 恰好 7 段时返回；
3. `read_plate`：第 0 位用省简称分类器，其余 6 位用字母数字分类器，拼成 `苏A·195K9`。

若 OCR 模型缺失或任一环节失败，`vision_app.py` 会回退到旧的整牌模板分类器（仅识别三张已知仿真号牌），因此旧检出仍可用。

## 适用范围

项目只提供 18 张人物立牌纹理、3 张车牌纹理和仿真资源，没有真实相机图像及人工框标注。人物类别对应当前素材中的社区/非社区身份。车牌 OCR 的字符分类器由字体合成样本 + 三张仿真车牌真实字符混合训练，对仿真三张号牌全串识别 100%（苏A·B8Q62、苏A·Q7M26、皖B·K3P58）。遇到真实行人、遮挡或极端视角时需要补充真实标注数据并重新训练。

外部中国车牌样本与实测记录见 [`external_plate_samples/README.md`](external_plate_samples/README.md)。经典 OpenCV 方案在 CCPD 真实照片上的全串识别仍有限（定位 4/5，全串 0/5），详见该文件的结果表。
