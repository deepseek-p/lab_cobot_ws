"""Colour-based device-indicator recognition with conservative ambiguity handling."""
from __future__ import annotations

import cv2
import numpy as np


STATE_BY_COLOR = {
    "green": "running",
    "amber": "idle",
    "red": "fault",
}


def _mask(hsv, color):
    if color == "red":
        return cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) | cv2.inRange(
            hsv, (170, 100, 100), (180, 255, 255))
    if color == "green":
        return cv2.inRange(hsv, (40, 70, 70), (95, 255, 255))
    if color == "amber":
        return cv2.inRange(hsv, (15, 100, 100), (38, 255, 255))
    raise ValueError(f"unsupported indicator color: {color}")


def classify_indicator(bgr_image, roi=(0, 0, 0, 0), min_pixels=20, dominance=0.60):
    """Classify one indicator ROI as running, idle, fault, or unknown.

    A state is emitted only when one colour owns enough coloured pixels and is
    dominant.  A model that renders multiple LEDs at once is intentionally
    reported as ``unknown`` instead of inventing a device state.
    """
    image = np.asarray(bgr_image)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("indicator image must be a BGR HxWx3 array")
    x, y, width, height = [int(value) for value in roi]
    if width > 0 and height > 0:
        x = max(0, x)
        y = max(0, y)
        image = image[y:y + height, x:x + width]
    if image.size == 0:
        return _unknown({"green": 0, "amber": 0, "red": 0})
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    counts = {color: int(np.count_nonzero(_mask(hsv, color)))
              for color in STATE_BY_COLOR}
    color, pixels = max(counts.items(), key=lambda item: item[1])
    total = sum(counts.values())
    confidence = pixels / total if total else 0.0
    if pixels < int(min_pixels) or confidence < float(dominance):
        return _unknown(counts, confidence)
    return {
        "state": STATE_BY_COLOR[color],
        "color": color,
        "confidence": confidence,
        "pixel_counts": counts,
    }


def _unknown(counts, confidence=0.0):
    return {
        "state": "unknown",
        "color": "unknown",
        "confidence": confidence,
        "pixel_counts": counts,
    }
