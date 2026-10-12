# 外部中国车牌样本验证

## 样本来源

5 张车辆/车牌图片取自 CCPD 官方 GitHub 仓库的 `rpnet/demo/` 目录。CCPD 仓库说明数据集采用 MIT 许可；此处保留了仓库的 `LICENSE` 文件。数据集论文引用信息见 CCPD 官方仓库说明：<https://github.com/detectRecog/CCPD>。

## 当前程序验证结果

运行方式：

```bash
cd /home/cyan/aic/vision_training
for n in 0 1 2 3 4; do
  python3 vision_app.py \
    --source "external_plate_samples/ccpd_demo_${n}.jpg" \
    --output "external_plate_samples/result_${n}.jpg" \
    --no-display --plates-only
done
```

本轮结果：

| 图片 | 图片上可读号码 | 程序输出 | 车牌定位 |
|---|---|---|---|
| `ccpd_demo_0.jpg` | 皖A·195K9 | 苏A·B8Q62 | 检出 |
| `ccpd_demo_1.jpg` | 皖A·UJ299 | 无结果 | 未检出 |
| `ccpd_demo_2.jpg` | 皖A·T022C | 苏A·Q7M26 | 检出 |
| `ccpd_demo_3.jpg` | 皖A·MK620 | 苏A·Q7M26 | 检出 |
| `ccpd_demo_4.jpg` | 皖A·256R2 | 苏A·Q7M26 | 检出 |

因此这 5 张样本中车牌区域检出 4/5，号码全串识别正确 0/5。现有分类器只会从仿真项目的 3 个已知号码中选一个，并没有逐字符读取任意车牌的 OCR 能力；这些结果不能用于宣称真实车牌识别具有鲁棒性。生成的 `result_*.jpg` 会在右侧显示独立识别面板，原始车辆图像不被结果文字覆盖。
