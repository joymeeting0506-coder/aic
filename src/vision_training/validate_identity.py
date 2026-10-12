#!/usr/bin/env python3
"""Run the trained community identity SVM on every supplied person board."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

from vision_app import (
    HERE,
    PERSON_HOG,
    PERSON_SIZE,
    chinese_font,
    draw_mixed_text,
    load_models,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        default="identity_validation.csv",
        help="CSV report path (default: identity_validation.csv)",
    )
    parser.add_argument(
        "--image",
        default="identity_validation.jpg",
        help="Annotated contact sheet path (default: identity_validation.jpg)",
    )
    args = parser.parse_args()

    manifest = HERE.parent / "src/smart_community_gazebo/person_manifest.csv"
    if not manifest.exists():
        raise FileNotFoundError(f"Person manifest not found: {manifest}")
    _, identity_model, _, metadata = load_models()
    labels = metadata["person_labels"]
    output = Path(args.output)
    if not output.is_absolute():
        output = HERE / output
    output.parent.mkdir(parents=True, exist_ok=True)
    image_output = Path(args.image)
    if not image_output.is_absolute():
        image_output = HERE / image_output
    image_output.parent.mkdir(parents=True, exist_ok=True)

    manifest_rows = list(csv.DictReader(manifest.open(encoding="utf-8-sig", newline="")))
    templates = []
    for item in manifest_rows:
        path = HERE.parent / "src/smart_community_gazebo/models" / item["model"] / "meshes/person_board.png"
        board = cv2.imread(str(path), cv2.IMREAD_COLOR)
        feature_image = cv2.resize(board, PERSON_SIZE, interpolation=cv2.INTER_AREA)
        feature = PERSON_HOG.compute(feature_image).reshape(-1).astype(np.float32)
        templates.append(feature / max(float(np.linalg.norm(feature)), 1e-12))
    templates = np.asarray(templates)

    rows: list[dict[str, str]] = []
    cards: list[tuple[Image.Image, dict[str, str]]] = []
    for item in manifest_rows:
            image_path = HERE.parent / "src/smart_community_gazebo/models" / item["model"] / "meshes/person_board.png"
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                raise FileNotFoundError(f"Could not read person board: {image_path}")
            resized = cv2.resize(image, PERSON_SIZE, interpolation=cv2.INTER_AREA)
            feature = PERSON_HOG.compute(resized).reshape(1, -1).astype(np.float32)
            index = int(round(float(identity_model.predict(feature)[1][0, 0])))
            if not 0 <= index < len(labels):
                raise RuntimeError(f"Model returned an invalid identity index {index}")
            prediction = labels[index]
            expected = item["identity"]
            normalized = feature.ravel() / max(float(np.linalg.norm(feature)), 1e-12)
            similarities = templates @ normalized
            group_scores = {
                group: float(np.max(similarities[[r["identity"] == group for r in manifest_rows]]))
                for group in ("community", "noncommunity")
            }
            # This is a normalized visual-similarity estimate, not a calibrated probability.
            logits = {group: value / 0.08 for group, value in group_scores.items()}
            peak = max(logits.values())
            exp_scores = {group: math.exp(value - peak) for group, value in logits.items()}
            confidence = exp_scores[prediction["identity"]] / sum(exp_scores.values())
            rows.append({
                "person": item["name"],
                "expected": expected,
                "predicted": prediction["identity"],
                "predicted_label": prediction["name"],
                "confidence_estimate": f"{confidence:.4f}",
                "correct": str(expected == prediction["identity"]),
                "image": str(image_path),
            })
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            cards.append((Image.fromarray(rgb), rows[-1]))

    with output.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    correct = sum(row["correct"] == "True" for row in rows)
    columns, cell_w, cell_h, image_h = 3, 250, 450, 370
    sheet = Image.new("RGB", (columns * cell_w, math.ceil(len(cards) / columns) * cell_h), (245, 245, 245))
    draw = ImageDraw.Draw(sheet)
    for i, (board, row) in enumerate(cards):
        x, y = (i % columns) * cell_w, (i // columns) * cell_h
        ratio = min((cell_w - 36) / board.width, image_h / board.height)
        size = (int(board.width * ratio), int(board.height * ratio))
        board = board.resize(size, Image.Resampling.LANCZOS)
        bx, by = x + (cell_w - size[0]) // 2, y + 8
        sheet.paste(board, (bx, by))
        color = (0, 185, 0) if row["predicted"] == "community" else (220, 35, 35)
        draw.rectangle((bx, by, bx + size[0] - 1, by + size[1] - 1), outline=color, width=5)
        readable = "社区人员" if row["predicted"] == "community" else "非社区人员"
        draw_mixed_text(draw, (x + 8, y + image_h + 8), f"{row['person']}  {readable}", 18, color)
        draw_mixed_text(
            draw,
            (x + 8, y + image_h + 34),
            f"置信度估算 {float(row['confidence_estimate']):.1%}  类别 {row['predicted_label']}",
            15,
            (45, 45, 45),
        )
    sheet.save(image_output, quality=92)

    print("身份分类验证（直接对立牌纹理调用已训练的 identity SVM）")
    for row in rows:
        status = "正确" if row["correct"] == "True" else "错误"
        print(f"{row['person']}: 预期={row['expected']}，识别={row['predicted']} ({row['predicted_label']})，置信度估算={float(row['confidence_estimate']):.1%}，{status}")
    print(f"准确率: {correct}/{len(rows)} = {correct / len(rows):.1%}")
    print(f"CSV: {output.resolve()}")
    print(f"结果图: {image_output.resolve()}")
    print("注意：这些立牌纹理参与过训练；图中置信度是按立牌特征相似度估算，并非校准概率；结果不代表整图检测率或未见场景泛化能力。")


if __name__ == "__main__":
    main()
