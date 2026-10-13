#!/usr/bin/env python3
"""Detect supplied person boards and blue plate boards in a camera/video stream."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).resolve().parent
MODELS = HERE / "trained_models"
PERSON_SIZE = (64, 128)
PLATE_SIZE = (128, 64)
PERSON_HOG = cv2.HOGDescriptor(PERSON_SIZE, (16, 16), (8, 8), (8, 8), 9)
PLATE_HOG = cv2.HOGDescriptor(PLATE_SIZE, (16, 16), (8, 8), (8, 8), 9)
CHINESE_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
)
LATIN_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
)


def load_models():
    required = [MODELS / "person_detector.xml", MODELS / "person_identity.xml", MODELS / "plate_classifier.xml", MODELS / "metadata.json"]
    missing = [path for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("Model files missing; run train_models.py first: " + ", ".join(map(str, missing)))
    metadata = json.loads(required[-1].read_text(encoding="utf-8"))
    detector = cv2.ml.SVM_load(str(required[0]))
    identity = cv2.ml.SVM_load(str(required[1]))
    plate_model = cv2.ml.SVM_load(str(required[2]))
    # Per-character OCR models are optional: when they are missing the code falls
    # back to the whole-plate template classifier, so older checkouts still run.
    province_svm = char_svm = None
    province_labels = metadata.get("province_labels", [])
    char_labels = metadata.get("char_labels", [])
    province_path = MODELS / "province_classifier.xml"
    char_path = MODELS / "char_classifier.xml"
    if province_path.exists() and char_path.exists() and province_labels and char_labels:
        province_svm = cv2.ml.SVM_load(str(province_path))
        char_svm = cv2.ml.SVM_load(str(char_path))
    return detector, identity, plate_model, metadata, province_svm, char_svm, province_labels, char_labels


def classify_patch(svm, hog, patch, size):
    resized = cv2.resize(patch, size, interpolation=cv2.INTER_AREA)
    feature = hog.compute(resized).reshape(1, -1).astype(np.float32)
    _, prediction = svm.predict(feature)
    return int(round(float(prediction[0, 0])))


def nms(boxes, threshold=0.35, score_index=4):
    if not boxes:
        return []
    boxes = sorted(boxes, key=lambda item: item[score_index], reverse=True)
    kept = []
    while boxes:
        current = boxes.pop(0)
        kept.append(current)
        x1, y1, w1, h1 = current[:4]
        remaining = []
        for box in boxes:
            x2, y2, w2, h2 = box[:4]
            ix1, iy1 = max(x1, x2), max(y1, y2)
            ix2, iy2 = min(x1 + w1, x2 + w2), min(y1 + h1, y2 + h2)
            intersection = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            union = w1 * h1 + w2 * h2 - intersection
            if union <= 0 or intersection / union < threshold:
                remaining.append(box)
        boxes = remaining
    return kept


def detect_people(frame, detector, identity_model, metadata, threshold=0.0, max_width=960):
    original_h, original_w = frame.shape[:2]
    scale_down = min(1.0, max_width / original_w)
    image = cv2.resize(frame, None, fx=scale_down, fy=scale_down) if scale_down < 1 else frame
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    boxes = []
    # Vertical board aspect is about 1:3; scan a coarse image pyramid.
    for window_w in (22, 28, 36, 46, 58, 74, 94, 120):
        window_h = window_w * 2
        if window_h > h or window_w > w:
            continue
        stride = max(4, window_w // 5)
        for y in range(0, h - window_h + 1, stride):
            for x in range(0, w - window_w + 1, stride):
                patch = cv2.cvtColor(image[y:y + window_h, x:x + window_w], cv2.COLOR_BGR2GRAY)
                patch = cv2.resize(patch, PERSON_SIZE, interpolation=cv2.INTER_AREA)
                feature = PERSON_HOG.compute(patch).reshape(1, -1).astype(np.float32)
                _, raw = detector.predict(feature, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)
                # OpenCV's binary raw margin is negative for our positive label (1).
                margin = -float(raw[0, 0])
                if margin < threshold:
                    continue
                crop = image[y:y + window_h, x:x + window_w]
                identity_index = classify_patch(identity_model, PERSON_HOG, crop, PERSON_SIZE)
                labels = metadata["person_labels"]
                if not 0 <= identity_index < len(labels):
                    continue
                label = labels[identity_index]
                factor = 1.0 / scale_down
                boxes.append((int(x * factor), int(y * factor), int(window_w * factor), int(window_h * factor), margin, label))

    # The broad scanner creates many nested boxes; keep higher-margin regions.
    kept = nms(boxes, threshold=0.28)
    return [(x, y, w, h, label, confidence) for x, y, w, h, confidence, label in kept]


def detect_plates(frame, plate_model, plate_texts, province_svm=None, char_svm=None, province_labels=(), char_labels=()):
    # Primary path: true per-character OCR reads any standard blue plate.
    if province_svm is not None and char_svm is not None:
        from plate_ocr import locate_plate, read_plate
        bbox, face = locate_plate(frame)
        if face is not None:
            text = read_plate(face, province_svm, char_svm, province_labels, char_labels)
            if text:
                x, y, w, h = bbox
                return [(x, y, w, h, text, 0.90)]

    # Fallback: whole-plate template classifier (only knows the 3 sim plates).
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([90, 65, 35]), np.array([140, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    frame_area = frame.shape[0] * frame.shape[1]
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        ratio = w / max(h, 1)
        area = w * h
        if not 1.55 <= ratio <= 3.4 or area < max(500, frame_area * 0.004):
            continue
        patch = frame[y:y + h, x:x + w]
        index = classify_patch(plate_model, PLATE_HOG, patch, PLATE_SIZE)
        if 0 <= index < len(plate_texts):
            found.append((x, y, w, h, plate_texts[index], 0.90))
    return nms(found, threshold=0.35, score_index=5)


def _font_from_candidates(candidates, size):
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return None


def chinese_font(size=28):
    font = _font_from_candidates(CHINESE_FONT_CANDIDATES, size)
    if font is not None:
        return font
    raise FileNotFoundError(
        "No Chinese font found. Install fonts with: sudo apt install fonts-noto-cjk"
    )


def latin_font(size=28):
    font = _font_from_candidates(LATIN_FONT_CANDIDATES, size)
    if font is not None:
        return font
    raise FileNotFoundError("No Latin font found on this system")


def draw_mixed_text(draw, position, text, size, fill):
    """Draw CJK glyphs and Latin glyphs with fonts that support each range."""
    x, y = position
    cjk = chinese_font(size)
    latin = latin_font(size)
    for char in text:
        font = cjk if "\u3400" <= char <= "\u9fff" else latin
        draw.text((x, y), char, font=font, fill=fill)
        x += draw.textlength(char, font=font)
    return x


def draw_results(frame, people, plates):
    for x, y, w, h, label, confidence in people:
        community = label["identity"] == "community"
        color = (0, 210, 0) if community else (0, 0, 230)
        text = f"{'community' if community else 'non-community'} {confidence:.2f}"
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, text, (x, max(20, y - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    # Put plate results in a separate right-hand panel so they never cover the
    # source frame or the license plates. Pillow renders the Chinese glyphs.
    height, width = frame.shape[:2]
    panel_width = max(260, min(380, width // 3))
    canvas = np.full((height, width + panel_width, 3), (35, 35, 35), dtype=np.uint8)
    canvas[:, :width] = frame
    cv2.line(canvas, (width, 0), (width, height), (95, 95, 95), 2)

    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    size = max(22, min(34, height // 18))
    draw_mixed_text(draw, (width + 18, 18), "识别结果", size + 4, (255, 255, 255))
    if not plates:
        draw_mixed_text(draw, (width + 18, 70), "未检测到车牌", size, (190, 190, 190))
    else:
        for index, (_, _, _, _, text, _confidence) in enumerate(plates):
            y = 72 + index * (size + 20)
            draw_mixed_text(draw, (width + 18, y), f"{index + 1}. {text}", size, (255, 220, 0))
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="0", help="Camera index, video path, or image path")
    parser.add_argument("--threshold", type=float, default=0.0, help="Person SVM decision threshold")
    parser.add_argument("--plates-only", action="store_true", help="Skip person scanning (useful for plate-only evaluation)")
    parser.add_argument("--no-display", action="store_true", help="Run headlessly and write annotated video")
    parser.add_argument("--output", default="vision_output.mp4", help="Annotated video path for video sources")
    args = parser.parse_args()

    detector, identity_model, plate_model, metadata, province_svm, char_svm, province_labels, char_labels = load_models()
    source = int(args.source) if args.source.isdigit() else args.source
    is_image = isinstance(source, str) and Path(source).suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp"}
    if is_image:
        frame = cv2.imread(source)
        if frame is None:
            raise FileNotFoundError(source)
        people = [] if args.plates_only else detect_people(frame, detector, identity_model, metadata, args.threshold)
        plates = detect_plates(frame, plate_model, metadata["plate_labels"], province_svm, char_svm, province_labels, char_labels)
        annotated = draw_results(frame.copy(), people, plates)
        out_path = Path(args.output) if args.output.lower().endswith((".png", ".jpg", ".jpeg")) else Path("annotated_image.jpg")
        cv2.imwrite(str(out_path), annotated)
        print(f"Saved annotated image: {out_path.resolve()}")
        if not args.no_display:
            cv2.imshow("Community inspection vision", annotated)
            cv2.waitKey(0)
        return

    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise RuntimeError(f"Could not open video source: {args.source}")
    writer = None
    frame_index = 0
    print("Press q to quit. Person boards: green=community, red=non-community; plate results appear in the right panel.")
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frame_index += 1
        people = [] if args.plates_only else detect_people(frame, detector, identity_model, metadata, args.threshold)
        plates = detect_plates(frame, plate_model, metadata["plate_labels"], province_svm, char_svm, province_labels, char_labels)
        for _, _, _, _, label, confidence in people:
            print(f"person: {label['identity']} ({label['name']}), confidence={confidence:.3f}")
        for _, _, _, _, text, _score in plates:
            print(f"license_plate: {text}")
        annotated = draw_results(frame.copy(), people, plates)
        if not args.no_display:
            cv2.imshow("Community inspection vision", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        elif isinstance(source, str):
            if writer is None:
                fps = capture.get(cv2.CAP_PROP_FPS) or 20.0
                size = (annotated.shape[1], annotated.shape[0])
                writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
            writer.write(annotated)
    capture.release()
    if writer is not None:
        writer.release()
        print(f"Saved annotated video: {Path(args.output).resolve()}")
    cv2.destroyAllWindows()
    if frame_index == 0:
        sys.exit("No frames received from the source")


if __name__ == "__main__":
    main()
