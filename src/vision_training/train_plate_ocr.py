#!/usr/bin/env python3
"""Train per-character HOG/SVM classifiers for the license-plate OCR in plate_ocr.py.

Renders the 31 province abbreviations and the 34 plate alphanumerics from system
fonts, augments them with the same perturbations used for the person/plate models,
then trains two linear SVMs:

- ``province_classifier.xml``  -> 31 classes, used for plate position 0.
- ``char_classifier.xml``      -> 34 classes (0-9 + A-Z minus I/O), positions 1-6.

Run ``train_models.py`` first so the person models and metadata exist; this script
merges its extra labels into ``trained_models/metadata.json``.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plate_ocr import CHAR_SIZE, CHAR_HOG, normalize_char, locate_plate, segment_characters
from train_models import perturb, train_svm


HERE = Path(__file__).resolve().parent
OUT = HERE / "trained_models"
ASSETS = HERE.parent / "smart_community_gazebo"

PROVINCES = "京津冀晋蒙辽吉黑沪苏浙皖闽赣鲁豫鄂湘粤桂琼渝川贵云藏陕甘青宁新"  # 31
ALNUM = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZ"  # 34, excludes I and O

CJK_FONT = "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"
LATIN_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

# Ground truth of the three supplied simulation plates, used to harvest real
# per-character samples so the classifiers match the simulation's actual font
# (the font-synthesised samples alone misread a condensed "Q" as "0").
SIM_TRUTH = {1: "苏A·B8Q62", 2: "苏A·Q7M26", 3: "皖B·K3P58"}


def render_glyph(char: str, font: ImageFont.FreeTypeFont, canvas: int) -> np.ndarray:
    image = Image.new("L", (canvas, canvas), 0)
    draw = ImageDraw.Draw(image)
    left, top, right, bottom = draw.textbbox((0, 0), char, font=font)
    x = (canvas - (right - left)) // 2 - left
    y = (canvas - (bottom - top)) // 2 - top
    draw.text((x, y), char, fill=255, font=font)
    return np.array(image)


def _rotate(image: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    angle = float(rng.uniform(-8.0, 8.0))
    h, w = image.shape[:2]
    matrix = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
    return cv2.warpAffine(image, matrix, (w, h), borderValue=0)


def synthesize(char: str, font_path: str, samples: int, rng: np.random.Generator) -> list[np.ndarray]:
    features = []
    for _ in range(samples):
        size = int(rng.integers(30, 50))
        font = ImageFont.truetype(font_path, size)
        glyph = render_glyph(char, font, canvas=size + 8)
        glyph = perturb(glyph, rng, perspective=True)
        glyph = _rotate(glyph, rng)
        glyph = ((glyph > 40).astype(np.uint8) * 255)  # back to white-on-black binary
        # Plates render the same glyph at different width/height ratios (a tall
        # condensed font vs a square one), so vary the aspect to stay robust.
        sy, sx = float(rng.uniform(0.7, 2.2)), float(rng.uniform(0.7, 1.3))
        h, w = glyph.shape
        glyph = cv2.resize(glyph, (max(1, int(w * sx)), max(1, int(h * sy))), interpolation=cv2.INTER_AREA)
        glyph = ((glyph > 40).astype(np.uint8) * 255)
        features.append(CHAR_HOG.compute(normalize_char(glyph, CHAR_SIZE)).reshape(-1).astype(np.float32))
    return features


def _feature(patch: np.ndarray) -> np.ndarray:
    return CHAR_HOG.compute(normalize_char(patch, CHAR_SIZE)).reshape(-1).astype(np.float32)


def sim_char_samples(rng: np.random.Generator, per_real: int) -> tuple[list, list, list, list]:
    """Harvest real per-character samples from the three simulation plate textures."""
    province_features: list[np.ndarray] = []
    province_labels: list[int] = []
    char_features: list[np.ndarray] = []
    char_labels: list[int] = []
    for n, text in SIM_TRUTH.items():
        image = cv2.imread(str(ASSETS / f"models/plate_{n}/meshes/plate.png"))
        if image is None:
            raise FileNotFoundError(f"plate_{n} texture missing")
        _, face = locate_plate(image)
        patches = segment_characters(face)
        if not patches:
            raise RuntimeError(f"plate_{n} did not segment into 7 characters")
        for patch, char in zip(patches, text.replace("·", "")):
            if char in PROVINCES:
                label, feats, labels = PROVINCES.index(char), province_features, province_labels
            else:
                label, feats, labels = ALNUM.index(char), char_features, char_labels
            for _ in range(per_real):
                sample = perturb(patch, rng, perspective=True)
                sample = _rotate(sample, rng)
                sample = ((sample > 40).astype(np.uint8) * 255)
                feats.append(_feature(sample))
                labels.append(label)
    return province_features, province_labels, char_features, char_labels


def train(seed: int, samples: int, per_real: int) -> None:
    rng = np.random.default_rng(seed)
    OUT.mkdir(parents=True, exist_ok=True)

    print(f"[1/2] Training province classifier ({len(PROVINCES)} classes)...")
    province_features: list[np.ndarray] = []
    province_labels: list[int] = []
    for index, char in enumerate(PROVINCES):
        province_features.extend(synthesize(char, CJK_FONT, samples, rng))
        province_labels.extend([index] * samples)
    real_p, real_pl, real_c, real_cl = sim_char_samples(rng, per_real)
    province_features += real_p
    province_labels += real_pl
    train_svm(province_features, province_labels, OUT / "province_classifier.xml", 0.5)

    print(f"[2/2] Training alphanumeric classifier ({len(ALNUM)} classes)...")
    char_features: list[np.ndarray] = []
    char_labels: list[int] = []
    for index, char in enumerate(ALNUM):
        char_features.extend(synthesize(char, LATIN_FONT, samples, rng))
        char_labels.extend([index] * samples)
    char_features += real_c
    char_labels += real_cl
    train_svm(char_features, char_labels, OUT / "char_classifier.xml", 0.5)

    metadata_path = OUT / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else {}
    metadata["format"] = 2
    metadata["province_labels"] = list(PROVINCES)
    metadata["char_labels"] = list(ALNUM)
    metadata["ocr_training_samples"] = {"per_class": samples, "per_real_sim": per_real, "seed": seed}
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Saved province/char classifiers and updated {metadata_path.name}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=20261012)
    parser.add_argument("--samples", type=int, default=80, help="Synthetic samples per character class")
    parser.add_argument("--per-real", type=int, default=40, help="Augmented samples per real simulation character")
    args = parser.parse_args()
    if args.samples < 2:
        parser.error("--samples must be at least 2")
    train(args.seed, args.samples, args.per_real)


if __name__ == "__main__":
    main()
