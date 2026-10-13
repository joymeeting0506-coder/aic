# 外部中国车牌样本验证

## 样本来源

5 张车辆/车牌图片取自 CCPD 官方 GitHub 仓库的 `rpnet/demo/` 目录。CCPD 仓库说明数据集采用 MIT 许可；此处保留了仓库的 `LICENSE` 文件。数据集论文引用信息见 CCPD 官方仓库说明：<https://github.com/detectRecog/CCPD>。

## 当前程序验证结果

运行方式：

```bash
cd /home/biao/aic/src/vision_training
for n in 0 1 2 3 4; do
  python3 vision_app.py \
    --source "external_plate_samples/ccpd_demo_${n}.jpg" \
    --output "external_plate_samples/result_${n}.jpg" \
    --no-display --plates-only
done
```

本轮结果（逐字符 OCR 管线，模型由字体合成 + 仿真真实字符混合训练）：

| 图片 | 图片上可读号码 | 程序输出 | 车牌定位 |
|---|---|---|---|
| `ccpd_demo_0.jpg` | 皖A·195K9 | 琼A·03CP7 | 检出 |
| `ccpd_demo_1.jpg` | 皖A·UJ299 | 无结果 | 未检出 |
| `ccpd_demo_2.jpg` | 皖A·T022C | 赣A·XQ22C | 检出 |
| `ccpd_demo_3.jpg` | 皖A·MK620 | 苏A·Q7M26 | 检出 |
| `ccpd_demo_4.jpg` | 皖A·256R2 | 苏A·Q7M26 | 检出 |

因此这 5 张样本中车牌区域检出 4/5，号码全串识别正确 0/5。与旧版「整牌三分类」相比，本轮程序已是真正的逐字符 OCR：`ccpd_demo_0/2` 输出的是逐字符读取结果（如 `ccpd_demo_2` 的 `22C` 已读对），而 `ccpd_demo_3/4` 因字符分割失败回退到了整牌模板分类器（`苏A·Q7M26`）。

局限与原因：

- 经典 OpenCV 方案对真实照片的光照不均、模糊和斜视角敏感：蓝色掩码在 `ccpd_demo_1` 上未能形成可用的号牌轮廓，`ccpd_demo_3/4` 的透视矫正不完整导致字符碎片化、无法切成 7 段。
- 省简称（皖）在合成字体与真实车牌字体差异下易误判（琼/赣）。
- 这些结果不能用于宣称真实车牌识别具有鲁棒性；如需比赛级真实车牌精度，应引入真实标注数据或基于 CNN 的端到端 LPR 模型。

生成的 `result_*.jpg` 会在右侧显示独立识别面板，原始车辆图像不被结果文字覆盖。
