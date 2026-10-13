#!/usr/bin/env python3
"""Train lightweight OpenCV HOG/SVM models from the simulation's own textures."""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import cv2
import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parent
ASSETS = REPO / "smart_community_gazebo"
OUT = HERE / "trained_models"
PERSON_SIZE = (64, 128)  # width, height
PLATE_SIZE = (128, 64)   # width, height
PERSON_HOG = cv2.HOGDescriptor(
    PERSON_SIZE, (16, 16), (8, 8), (8, 8), 9
)
PLATE_HOG = cv2.HOGDescriptor(
    PLATE_SIZE, (16, 16), (8, 8), (8, 8), 9
)


def hog_vector(image: np.ndarray, size: tuple[int, int], hog: cv2.HOGDescriptor) -> np.ndarray:
    image = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return hog.compute(image).reshape(-1).astype(np.float32)


def perturb(image: np.ndarray, rng: np.random.Generator, perspective: bool = True) -> np.ndarray:
    height, width = image.shape[:2]
    result = image.copy()
    if perspective:
        amount = float(rng.uniform(0.0, 0.08))
        src = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
        dx, dy = amount * width, amount * height
        dst = src + np.float32([
            [rng.uniform(-dx, dx), rng.uniform(-dy, dy)],
            [rng.uniform(-dx, dx), rng.uniform(-dy, dy)],
            [rng.uniform(-dx, dx), rng.uniform(-dy, dy)],
            [rng.uniform(-dx, dx), rng.uniform(-dy, dy)],
        ])
        matrix = cv2.getPerspectiveTransform(src, dst)
        result = cv2.warpPerspective(result, matrix, (width, height), borderMode=cv2.BORDER_REPLICATE)

    alpha = float(rng.uniform(0.72, 1.28))
    beta = float(rng.uniform(-28, 28))
    result = cv2.convertScaleAbs(result, alpha=alpha, beta=beta)
    if rng.random() < 0.35:
        result = cv2.GaussianBlur(result, (3, 3), float(rng.uniform(0.2, 1.1)))
    if rng.random() < 0.6:
        noise = rng.normal(0, rng.uniform(1.5, 8.0), result.shape).astype(np.int16)
        result = np.clip(result.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return result


def load_person_assets() -> tuple[list[tuple[str, str, np.ndarray]], list[str]]:
    manifest = ASSETS / "person_manifest.csv"
    people = []
    with manifest.open(encoding="utf-8-sig") as stream:
        next(stream, None)
        for row in stream:
            fields = row.strip().split(",")
            if len(fields) < 8:
                continue
            name, model, identity = fields[0], fields[2], fields[7]
            image_path = ASSETS / "models" / model / "meshes/person_board.png"
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                raise FileNotFoundError(image_path)
            people.append((name, identity, image))
    if len(people) != 18:
        raise RuntimeError(f"Expected 18 person textures, found {len(people)} in {manifest}")
    return people, [entry[0] for entry in people]


def negative_crops(rng: np.random.Generator, count: int) -> list[np.ndarray]:
    sources = []
    for path in [
        ASSETS / "maps/community.pgm",
        ASSETS / "reference/community_01.png",
        ASSETS / "models/vehicle_background/meshes/car.png",
    ]:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is not None:
            sources.append(image)
    if not sources:
        raise FileNotFoundError("No scene/map/background images available for negative examples")

    samples = []
    for _ in range(count):
        source = sources[int(rng.integers(len(sources)))]
        h, w = source.shape[:2]
        crop_h = max(24, min(h, int(rng.uniform(0.18, 0.95) * h)))
        crop_w = max(12, min(w, int(crop_h / 2.0)))
        x = int(rng.integers(max(1, w - crop_w + 1)))
        y = int(rng.integers(max(1, h - crop_h + 1)))
        samples.append(cv2.resize(source[y:y + crop_h, x:x + crop_w], PERSON_SIZE))
    return samples


def train_svm(samples: list[np.ndarray], labels: list[int], path: Path, c: float) -> None:
    matrix = np.asarray(samples, dtype=np.float32)
    targets = np.asarray(labels, dtype=np.int32).reshape(-1, 1)
    model = cv2.ml.SVM_create()
    model.setType(cv2.ml.SVM_C_SVC)
    model.setKernel(cv2.ml.SVM_LINEAR)
    model.setC(c)
    model.setTermCriteria((cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 4000, 1e-5))
    if not model.train(matrix, cv2.ml.ROW_SAMPLE, targets):
        raise RuntimeError(f"OpenCV failed to train {path.name}")
    model.save(str(path))


def train(seed: int, person_samples: int, plate_samples: int) -> None:
    rng = np.random.default_rng(seed)
    random.seed(seed)
    OUT.mkdir(parents=True, exist_ok=True)
    people, person_names = load_person_assets()

    print("[1/2] Training person detector and community identity classifier...")
    person_features: list[np.ndarray] = []
    detection_labels: list[int] = []
    identity_features: list[np.ndarray] = []
    identity_labels: list[int] = []
    for identity_index, (_, _, image) in enumerate(people):
        for sample_index in range(person_samples):
            sample = image if sample_index == 0 else perturb(image, rng)
            feature = hog_vector(sample, PERSON_SIZE, PERSON_HOG)
            person_features.append(feature)
            detection_labels.append(1)
            identity_features.append(feature)
            identity_labels.append(identity_index)

    backgrounds = negative_crops(rng, person_samples * len(people))
    for sample in backgrounds:
        person_features.append(hog_vector(perturb(sample, rng, perspective=False), PERSON_SIZE, PERSON_HOG))
        detection_labels.append(0)

    train_svm(person_features, detection_labels, OUT / "person_detector.xml", 0.08)
    train_svm(identity_features, identity_labels, OUT / "person_identity.xml", 0.12)

    print("[2/2] Training license-plate template classifier...")
    layout = json.loads((ASSETS / "layout_manifest.json").read_text(encoding="utf-8"))
    plate_features: list[np.ndarray] = []
    plate_labels: list[int] = []
    plate_texts = [entry["text"] for entry in sorted(layout["plates"], key=lambda item: item["slot"])]
    for index, text in enumerate(plate_texts, start=1):
        plate_path = ASSETS / f"models/plate_{index}/meshes/plate.png"
        image = cv2.imread(str(plate_path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(plate_path)
        for sample_index in range(plate_samples):
            sample = image if sample_index == 0 else perturb(image, rng)
            plate_features.append(hog_vector(sample, PLATE_SIZE, PLATE_HOG))
            plate_labels.append(index - 1)

    train_svm(plate_features, plate_labels, OUT / "plate_classifier.xml", 0.15)
    metadata = {
        "format": 1,
        "seed": seed,
        "person_labels": [
            {"index": i, "name": name, "identity": identity}
            for i, (name, identity, _) in enumerate(people)
        ],
        "plate_labels": plate_texts,
        "training_samples": {
            "person_positive_per_asset": person_samples,
            "person_negative": len(backgrounds),
            "plate_per_template": plate_samples,
        },
        "notice": "Synthetic augmentation from supplied textures; validate on camera frames before competition use.",
    }
    (OUT / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Saved trained models and metadata to: {OUT}")
    print(f"Person positive examples: {len(people) * person_samples}; negative examples: {len(backgrounds)}")
    print(f"Plate templates: {', '.join(plate_texts)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=20261012)
    parser.add_argument("--person-samples", type=int, default=60, help="Augmented samples per person texture")
    parser.add_argument("--plate-samples", type=int, default=240, help="Augmented samples per plate template")
    args = parser.parse_args()
    if args.person_samples < 2 or args.plate_samples < 2:
        parser.error("sample counts must be at least 2")
    train(args.seed, args.person_samples, args.plate_samples)


if __name__ == "__main__":
    main()
