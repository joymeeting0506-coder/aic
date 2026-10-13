#!/usr/bin/env python3
"""Chinese blue-plate OCR: locate -> rectify -> segment 7 characters -> classify each.

Works on the standard 7-character blue plate layout `[province][letter]·[5 alnum]`
with a separator dot between positions 2 and 3. New-energy 8-character plates are
out of scope. Depends only on OpenCV + NumPy, matching the project's lightweight
constraint; the per-character SVM models are trained by ``train_plate_ocr.py``.
"""

from __future__ import annotations

import cv2
import numpy as np


CHAR_SIZE = (32, 40)  # width, height of the normalized character canvas
CHAR_HOG = cv2.HOGDescriptor(CHAR_SIZE, (16, 16), (8, 8), (8, 8), 9)
BLUE_LOW = np.array([90, 65, 35])
BLUE_HIGH = np.array([140, 255, 255])


def _order_points(pts: np.ndarray) -> np.ndarray:
    """Order four points as [top-left, top-right, bottom-right, bottom-left]."""
    pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    ordered[0] = pts[np.argmin(s)]
    ordered[2] = pts[np.argmax(s)]
    ordered[1] = pts[np.argmin(d)]
    ordered[3] = pts[np.argmax(d)]
    return ordered


def rectify_plate(image: np.ndarray, contour: np.ndarray, width: int = 440) -> np.ndarray:
    """Perspective-correct the plate quad to a horizontal, fixed-width image."""
    box = _order_points(cv2.boxPoints(cv2.minAreaRect(contour)))
    top = float(np.linalg.norm(box[1] - box[0]))
    bottom = float(np.linalg.norm(box[2] - box[3]))
    left = float(np.linalg.norm(box[3] - box[0]))
    right = float(np.linalg.norm(box[2] - box[1]))
    w = max(top, bottom)
    h = max(left, right)
    if h > w:  # minAreaRect reported the short side as width; rotate the quad 90deg
        box = np.roll(box, 1, axis=0)
        w, h = h, w
    height = int(round(width * h / max(w, 1e-6)))
    height = max(60, min(220, height))
    dst = np.array(
        [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(box, dst)
    return cv2.warpPerspective(image, matrix, (width, height))


def _crop_blue_face(plate: np.ndarray) -> np.ndarray:
    """Crop to the blue plate face so white borders/frames are excluded."""
    hsv = cv2.cvtColor(plate, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return plate
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
    return plate[y:y + h, x:x + w]


def locate_plate(frame: np.ndarray):
    """Find the blue plate and return ``(bbox, rectified_face)`` or ``(None, None)``."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = frame.shape[0] * frame.shape[1]
    best = None
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        ratio = w / max(h, 1)
        area = w * h
        if not 1.55 <= ratio <= 3.4 or area < max(500, frame_area * 0.004):
            continue
        if best is None or area > best[0]:
            best = (area, contour, x, y, w, h)
    if best is None:
        return None, None
    _, contour, x, y, w, h = best
    plate = rectify_plate(frame, contour)
    face = _crop_blue_face(plate)
    return (x, y, w, h), face


def _segment_mask(char_bin: np.ndarray):
    """Split one white-on-black mask into 7 character patches, or ``None``."""
    height, width = char_bin.shape

    # Zero out the hollow frame ring: its bounding box is nearly solid-area sized,
    # so the white-pixel fill ratio stays far below any real glyph.
    count, labels, stats, _ = cv2.connectedComponentsWithStats(char_bin, connectivity=8)
    for i in range(1, count):
        _, _, w, h, area = stats[i]
        if w * h > 0 and area / (w * h) < 0.15:
            char_bin[labels == i] = 0

    # Reconnect nearby strokes (the province glyph often splits into stacked parts);
    # the vertical kernel bridges small vertical gaps without merging adjacent chars.
    char_bin = cv2.morphologyEx(char_bin, cv2.MORPH_CLOSE, np.ones((7, 3), np.uint8))

    # Re-label and keep only tall components: characters span most of the face
    # height, while the separator dot and mounting rivets are a small fraction of it.
    count, labels, stats, _ = cv2.connectedComponentsWithStats(char_bin, connectivity=8)
    glyphs: list[tuple[int, int, int, int]] = []
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if h < 0.18 * height:
            continue
        glyphs.append((int(x), int(y), int(w), int(h)))

    if len(glyphs) != 7:
        return None
    glyphs.sort(key=lambda g: g[0])
    return [char_bin[y:y + h, x:x + w] for x, y, w, h in glyphs]


def _binarize(plate_face: np.ndarray):
    """Yield candidate white-on-black masks, most reliable first.

    Simulation textures have pure white glyphs (the HSV mask is exact), while real
    photos vary in lighting, so an Otsu threshold on intensity is tried next.
    """
    hsv = cv2.cvtColor(plate_face, cv2.COLOR_BGR2HSV)
    yield ((hsv[:, :, 1] < 70) & (hsv[:, :, 2] > 140)).astype(np.uint8) * 255
    gray = cv2.cvtColor(plate_face, cv2.COLOR_BGR2GRAY)
    _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    yield otsu
    # Adaptive threshold copes with uneven lighting across a real photo.
    adaptive = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY, 41, 8)
    yield adaptive
    # Blue-dominance: white glyphs have near-zero (blue - max(green, red)), while the
    # blue background has a large positive value, so low values mark the glyphs.
    b = plate_face[:, :, 0].astype(np.int16)
    dominance = b - np.maximum(plate_face[:, :, 1], plate_face[:, :, 2]).astype(np.int16)
    dominance = np.clip(dominance, 0, 255).astype(np.uint8)
    _, otsu_dom = cv2.threshold(dominance, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    yield otsu_dom


def segment_characters(plate_face: np.ndarray):
    """Split the plate face into 7 character patches, or ``None`` if not found.

    White glyphs are isolated from the blue face with a low-saturation / high-value
    mask. Connected-component analysis then removes the decorative hollow frame ring
    (low fill ratio), reconnects strokes of the CJK province glyph that the mask may
    split, drops the separator dot / mounting rivets / noise (much shorter than a
    character), and requires exactly 7 glyphs left to right.
    """
    if plate_face is None or plate_face.size == 0:
        return None
    for mask in _binarize(plate_face):
        result = _segment_mask(mask)
        if result is not None:
            return result
    return None


def normalize_char(patch: np.ndarray, size: tuple[int, int] = CHAR_SIZE) -> np.ndarray:
    """Center a glyph in a fixed canvas, preserving aspect ratio (white on black)."""
    canvas = np.zeros((size[1], size[0]), np.uint8)
    if patch is None or patch.size == 0:
        return canvas
    ys, xs = np.nonzero(patch)
    if ys.size == 0:
        return canvas
    glyph = patch[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    gh, gw = glyph.shape
    scale = min(size[1] / gh, size[0] / gw)
    nw = max(1, int(round(gw * scale)))
    nh = max(1, int(round(gh * scale)))
    resized = cv2.resize(glyph, (nw, nh), interpolation=cv2.INTER_AREA)
    x = (size[0] - nw) // 2
    y = (size[1] - nh) // 2
    canvas[y:y + nh, x:x + nw] = resized
    return canvas


def recognize_char(patch: np.ndarray, svm, labels: list[str], hog=CHAR_HOG, size=CHAR_SIZE) -> str:
    feature = hog.compute(normalize_char(patch, size)).reshape(1, -1).astype(np.float32)
    _, prediction = svm.predict(feature)
    index = int(round(float(prediction[0, 0])))
    return labels[index] if 0 <= index < len(labels) else "?"


def read_plate(plate_face: np.ndarray, province_svm, char_svm, province_labels, char_labels):
    """Recognize a rectified plate face and return e.g. ``苏A·195K9`` or ``None``."""
    chars = segment_characters(plate_face)
    if not chars:
        return None
    province = recognize_char(chars[0], province_svm, province_labels)
    rest = "".join(recognize_char(c, char_svm, char_labels) for c in chars[1:])
    return f"{province}{rest[0]}·{rest[1:]}"
