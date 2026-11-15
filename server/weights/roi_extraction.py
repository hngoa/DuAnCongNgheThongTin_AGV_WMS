"""SLAM_KLTN — Standalone ROI Extraction Module for Tray Objects.

Capabilities:
1. Autonomous object localization on full Android images using YOLO11n detector (default).
2. Explicit failure when YOLO is required in production but unavailable (no silent fallback).
3. Fallback adaptive OpenCV thresholding only when explicitly permitted by configuration.
4. Multiple object detection detection: flags when multiple candidates exist on tray.
5. Strict EXIF orientation consistency between YOLO detector and downstream classifiers.
6. Empty tray detection (dark, gray, or bright empty trays with no cargo object).
7. Aspect-ratio preserving letterbox padding to 224x224.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import List, Optional, Tuple, Union

import cv2
import numpy as np
from PIL import Image

try:
    from .preprocessing import (
        crop_roi_with_context,
        is_blank_or_textureless,
        letterbox_pad,
        load_image_rgb,
        validate_bbox,
    )
except (ImportError, ValueError):
    from preprocessing import (
        crop_roi_with_context,
        is_blank_or_textureless,
        letterbox_pad,
        load_image_rgb,
        validate_bbox,
    )

logger = logging.getLogger(__name__)

EMPTY_TRAY_MAX_CONTOUR_AREA = 500


class ObjectDetector:
    """YOLO-based object detector for autonomous Station A tray localization."""

    def __init__(
        self,
        weights_path: Optional[Union[str, Path]] = None,
        conf_threshold: float = 0.25,
        require_yolo: bool = True,
        allow_opencv_fallback: bool = False,
    ):
        self.conf_threshold = conf_threshold
        self.require_yolo = require_yolo
        self.allow_opencv_fallback = allow_opencv_fallback
        self.model = None

        if weights_path is None:
            self.weights_path = Path(__file__).resolve().parent / "yolo11n_v5b_best.pt"
        else:
            self.weights_path = Path(weights_path)

        try:
            import os
            os.environ["YOLO_AUTO_INSTALL"] = "0"
            from ultralytics import YOLO
            if not self.weights_path.is_file():
                raise FileNotFoundError(f"YOLO detector checkpoint not found at: {self.weights_path}")
            self.model = YOLO(str(self.weights_path))
            logger.info(f"Loaded YOLO detector from {self.weights_path}")
        except Exception as e:
            if self.require_yolo and not self.allow_opencv_fallback:
                raise RuntimeError(
                    f"Production error: YOLO detector is required but failed to initialize from '{self.weights_path}': {e}. "
                    f"Set allow_opencv_fallback=True only if OpenCV heuristic fallback is explicitly intended."
                ) from e
            logger.warning(
                f"YOLO detector unavailable ({e}). Falling back to OpenCV heuristic localization as allowed by configuration."
            )
            self.model = None

    def detect(
        self,
        image_input: Union[str, Path, Image.Image, np.ndarray],
    ) -> Tuple[Optional[Tuple[int, int, int, int]], int, List[Tuple[int, int, int, int]]]:
        """Detect cargo objects on the tray with EXIF orientation consistency.

        Returns:
            Tuple of:
            - primary_bbox: (x1, y1, x2, y2) of most confident object, or None
            - num_detections: Total count of detected objects
            - all_boxes: List of all detected bounding boxes
        """
        # Always apply EXIF orientation transpose first to ensure identical coordinate space
        pil_img = load_image_rgb(image_input)
        w_img, h_img = pil_img.size

        # Check for completely blank / textureless image (e.g. empty bright or dark tray)
        if is_blank_or_textureless(pil_img):
            return None, 0, []

        if self.model is not None:
            try:
                # Convert PIL RGB to OpenCV BGR for native YOLO format
                bgr_arr = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
                results = self.model(bgr_arr, conf=self.conf_threshold, agnostic_nms=True, iou=0.45, verbose=False)
                if len(results) > 0 and len(results[0].boxes) > 0:
                    boxes = results[0].boxes
                    sorted_indices = sorted(range(len(boxes)), key=lambda i: float(boxes.conf[i]), reverse=True)

                    primary_raw = boxes.xyxy[sorted_indices[0]].cpu().numpy().astype(int)
                    try:
                        primary_box = validate_bbox(tuple(primary_raw), w_img, h_img)
                    except ValueError:
                        return None, 0, []

                    w_pri = primary_box[2] - primary_box[0]
                    h_pri = primary_box[3] - primary_box[1]
                    area_pri = max(1, w_pri * h_pri)

                    valid_boxes = [primary_box]
                    self.yolo_detections = {
                        primary_box: {
                            "class_id": int(boxes.cls[sorted_indices[0]]),
                            "class_name": self.model.names.get(int(boxes.cls[sorted_indices[0]]), "unknown") if hasattr(self.model, "names") else "unknown",
                            "conf": float(boxes.conf[sorted_indices[0]])
                        }
                    }
                    for idx in sorted_indices[1:]:
                        b_raw = boxes.xyxy[idx].cpu().numpy().astype(int)
                        try:
                            vb = validate_bbox(tuple(b_raw), w_img, h_img)
                        except ValueError:
                            continue
                        w_b = vb[2] - vb[0]
                        h_b = vb[3] - vb[1]
                        area_b = w_b * h_b
                        conf_b = float(boxes.conf[idx])

                        # Suppress tiny edge noise/reflections (< 12% area or conf < 0.30)
                        if (area_b / area_pri < 0.12) or (conf_b < 0.30):
                            continue
                        valid_boxes.append(vb)
                        self.yolo_detections[vb] = {
                            "class_id": int(boxes.cls[idx]),
                            "class_name": self.model.names.get(int(boxes.cls[idx]), "unknown") if hasattr(self.model, "names") else "unknown",
                            "conf": conf_b
                        }

                    if valid_boxes:
                        return valid_boxes[0], len(valid_boxes), valid_boxes
                return None, 0, []
            except Exception as e:
                if self.require_yolo and not self.allow_opencv_fallback:
                    raise RuntimeError(f"YOLO inference error during detection: {e}") from e
                logger.warning(f"YOLO inference error ({e}). Attempting OpenCV fallback.")

        # Fallback to OpenCV tray foreground detection if permitted
        fg_box = detect_tray_foreground(pil_img)
        if fg_box is not None:
            return fg_box, 1, [fg_box]
        return None, 0, []


def detect_tray_foreground(
    image_input: Union[str, Path, Image.Image, np.ndarray],
    min_contour_area: int = EMPTY_TRAY_MAX_CONTOUR_AREA,
    is_bgr: bool = False,
) -> Optional[Tuple[int, int, int, int]]:
    """Adaptive OpenCV thresholding on downscaled image for tray object localization."""
    pil_img = load_image_rgb(image_input, is_bgr=is_bgr)
    if is_blank_or_textureless(pil_img):
        return None

    rgb = np.array(pil_img)
    h, w = rgb.shape[:2]

    # Quick check for pure dark image
    gray_full = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    if float(np.mean(gray_full)) < 15.0 and float(np.max(gray_full)) < 40.0:
        return None

    # Downscale for robust thresholding
    small = cv2.resize(rgb, (640, 640), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Otsu thresholding against dark tray
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Morphological closing
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    valid = []
    for c in contours:
        a = cv2.contourArea(c)
        if 500 < a < 0.85 * 640 * 640:
            bx, by, bw, bh = cv2.boundingRect(c)
            valid.append((a, bx, by, bw, bh))

    if not valid:
        return None

    valid.sort(key=lambda x: x[0], reverse=True)
    best = valid[0]
    sx = float(w) / 640.0
    sy = float(h) / 640.0
    x1 = int(round(best[1] * sx))
    y1 = int(round(best[2] * sy))
    x2 = int(round((best[1] + best[3]) * sx))
    y2 = int(round((best[2] + best[4]) * sy))

    try:
        return validate_bbox((x1, y1, x2, y2), w, h)
    except ValueError:
        return None


def is_empty_tray(
    image_input: Union[str, Path, Image.Image, np.ndarray],
    detector: Optional[ObjectDetector] = None,
    is_bgr: bool = False,
) -> bool:
    """Check if the provided image shows an empty tray (dark or bright) with no target object."""
    if detector is not None:
        bbox, num_dets, _ = detector.detect(image_input)
        return bbox is None or num_dets == 0
    bbox = detect_tray_foreground(image_input, is_bgr=is_bgr)
    return bbox is None


def extract_roi(
    image_input: Union[str, Path, Image.Image, np.ndarray],
    bbox: Optional[Tuple[int, int, int, int]] = None,
    detector: Optional[ObjectDetector] = None,
    pad_ratio: float = 0.14,
    target_size: int = 224,
    is_bgr: bool = False,
) -> Tuple[Image.Image, Optional[Tuple[int, int, int, int]], str]:
    """Extract square letterboxed ROI for classification."""
    img = load_image_rgb(image_input, is_bgr=is_bgr)
    w, h = img.size

    effective_bbox = bbox
    if effective_bbox is not None:
        try:
            effective_bbox = validate_bbox(effective_bbox, w, h)
        except ValueError as e:
            raise ValueError(f"Invalid bounding box provided: {e}") from e
    else:
        if detector is not None:
            effective_bbox, num_dets, _ = detector.detect(img)
            if num_dets > 1:
                return letterbox_pad(img, target_size=target_size), effective_bbox, "MULTIPLE_OBJECTS"
        else:
            effective_bbox = detect_tray_foreground(img)

        if effective_bbox is None:
            if is_blank_or_textureless(img):
                padded = letterbox_pad(img, target_size=target_size)
                return padded, None, "NO_OBJECT"
            effective_bbox = (0, 0, w, h)

    cropped = crop_roi_with_context(
        img,
        bbox=effective_bbox,
        pad_ratio=pad_ratio,
        target_size=target_size,
    )
    return cropped, effective_bbox, "OK"
