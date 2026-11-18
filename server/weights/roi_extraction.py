"""SLAM_KLTN — Standalone OpenCV ROI Extraction Module for Tray Objects.

Capabilities:
1. Autonomous single & multi-object localization on Station A / Shot 2 images using OpenCV (no YOLO).
2. Chromatic foreground segmentation (HSV Saturation + RGB Chroma) immune to white-paper tray shadows.
3. Hue-bucket splitting so adjacent multi-color cargo blocks separate cleanly into individual ROIs.
4. Monochrome / dark-tray Otsu fallback for achromatic objects on dark or uniform bright backgrounds.
5. Strict EXIF orientation consistency and empty tray rejection (dark, gray, or bright paper trays).
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
    """OpenCV-based foreground & hue object detector for Station A tray localization (no YOLO)."""

    def __init__(
        self,
        weights_path: Optional[Union[str, Path]] = None,
        conf_threshold: float = 0.25,
        require_yolo: bool = False,
        allow_opencv_fallback: bool = True,
    ):
        self.conf_threshold = conf_threshold
        self.require_yolo = False
        self.allow_opencv_fallback = True
        self.weights_path = Path(weights_path) if weights_path is not None else None
        self.model = None
        logger.info("Initialized OpenCV ROI ObjectDetector (YOLO disabled; MobileNetV3-Small only mode).")

    @staticmethod
    def _is_nested_duplicate(
        candidate: Tuple[int, int, int, int],
        kept_boxes: List[Tuple[int, int, int, int]],
        containment_thresh: float = 0.65,
    ) -> bool:
        """Return True if candidate heavily overlaps or encloses/sits inside an accepted box."""
        cx1, cy1, cx2, cy2 = candidate
        area_c = max(1, (cx2 - cx1) * (cy2 - cy1))
        for kx1, ky1, kx2, ky2 in kept_boxes:
            ix1 = max(cx1, kx1)
            iy1 = max(cy1, ky1)
            ix2 = min(cx2, kx2)
            iy2 = min(cy2, ky2)
            if ix2 <= ix1 or iy2 <= iy1:
                continue
            inter_area = (ix2 - ix1) * (iy2 - iy1)
            area_k = max(1, (kx2 - kx1) * (ky2 - ky1))
            if (inter_area / min(area_c, area_k)) >= containment_thresh:
                return True
        return False

    def detect(
        self,
        image_input: Union[str, Path, Image.Image, np.ndarray],
    ) -> Tuple[Optional[Tuple[int, int, int, int]], int, List[Tuple[int, int, int, int]]]:
        """Detect cargo objects on the tray with EXIF orientation consistency.

        Returns:
            Tuple of:
            - primary_bbox: (x1, y1, x2, y2) of largest detected object, or None
            - num_detections: Total count of detected objects
            - all_boxes: List of all detected bounding boxes
        """
        pil_img = load_image_rgb(image_input)
        if is_blank_or_textureless(pil_img):
            return None, 0, []

        boxes = detect_tray_foregrounds(pil_img)
        if not boxes:
            return None, 0, []
        return boxes[0], len(boxes), boxes


def _scale_and_filter_contours(
    contours: List[np.ndarray],
    scale_x: float,
    scale_y: float,
    w_img: int,
    h_img: int,
    min_area_small: float,
    max_area_small: float,
    check_tray_margins: bool = False,
    small_w: int = 320,
    small_h: int = 320,
) -> List[Tuple[int, Tuple[int, int, int, int]]]:
    """Convert valid contours into (area_full, validated_bbox) entries."""
    entries: List[Tuple[int, Tuple[int, int, int, int]]] = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < min_area_small or area >= max_area_small:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        if check_tray_margins:
            cx_norm = (bx + 0.5 * bw) / float(small_w)
            cy_norm = (by + 0.5 * bh) / float(small_h)
            if cx_norm > 0.93 or cx_norm < 0.04 or cy_norm < 0.14:
                continue
        x1 = int(round(bx * scale_x))
        y1 = int(round(by * scale_y))
        x2 = int(round((bx + bw) * scale_x))
        y2 = int(round((by + bh) * scale_y))
        try:
            vb = validate_bbox((x1, y1, x2, y2), w_img, h_img)
            entries.append((int(round((vb[2] - vb[0]) * (vb[3] - vb[1]))), vb))
        except ValueError:
            continue
    return entries


def _suppress_duplicates(
    area_boxes: List[Tuple[int, Tuple[int, int, int, int]]]
) -> List[Tuple[int, int, int, int]]:
    """Sort largest-first and suppress overlapping nested duplicate boxes."""
    area_boxes.sort(key=lambda item: item[0], reverse=True)
    kept: List[Tuple[int, int, int, int]] = []
    for _, box in area_boxes:
        if not ObjectDetector._is_nested_duplicate(box, kept, containment_thresh=0.65):
            kept.append(box)
    return kept


def detect_tray_foregrounds(
    image_input: Union[str, Path, Image.Image, np.ndarray],
    min_contour_area: int = EMPTY_TRAY_MAX_CONTOUR_AREA,
    is_bgr: bool = False,
) -> List[Tuple[int, int, int, int]]:
    """Detect one or more cargo bounding boxes on the tray using pure OpenCV (no YOLO)."""
    pil_img = load_image_rgb(image_input, is_bgr=is_bgr)
    if is_blank_or_textureless(pil_img):
        return []

    rgb = np.array(pil_img)
    h_img, w_img = rgb.shape[:2]

    gray_full = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    if float(np.mean(gray_full)) < 15.0 and float(np.max(gray_full)) < 40.0:
        return []

    # ------------------------------------------------------------------
    # Pass 1: Chromatic foreground localization (HSV Saturation + Chroma)
    # Immune to gray cardboard borders, A4 paper creases, and tray shadows.
    # ------------------------------------------------------------------
    color_dim = 320
    small_rgb = cv2.resize(rgb, (color_dim, color_dim), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small_rgb, cv2.COLOR_RGB2HSV)
    h_chan, s_chan, _ = cv2.split(hsv)
    chroma = np.ptp(small_rgb, axis=2)

    fg_color = ((chroma >= 28) & (s_chan >= 42)).astype(np.uint8) * 255
    k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    fg_color = cv2.morphologyEx(fg_color, cv2.MORPH_OPEN, k3)
    fg_color = cv2.morphologyEx(fg_color, cv2.MORPH_CLOSE, k5)

    contours_c, _ = cv2.findContours(fg_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area_c = 0.015 * color_dim * color_dim
    max_area_c = 0.85 * color_dim * color_dim
    sx_c = float(w_img) / float(color_dim)
    sy_c = float(h_img) / float(color_dim)

    hue_masks_defs = (
        ((h_chan < 14) | (h_chan >= 170)),
        ((h_chan >= 14) & (h_chan < 45)),
        ((h_chan >= 45) & (h_chan < 125)),
        ((h_chan >= 125) & (h_chan < 170)),
    )

    color_entries: List[Tuple[int, Tuple[int, int, int, int]]] = []
    for cnt in contours_c:
        c_area = float(cv2.contourArea(cnt))
        if c_area < min_area_c or c_area >= max_area_c:
            continue
        bx, by, bw, bh = cv2.boundingRect(cnt)
        cx_norm = (bx + 0.5 * bw) / float(color_dim)
        cy_norm = (by + 0.5 * bh) / float(color_dim)
        if cx_norm > 0.93 or cx_norm < 0.04 or cy_norm < 0.14:
            continue

        comp_mask = np.zeros((color_dim, color_dim), dtype=np.uint8)
        cv2.drawContours(comp_mask, [cnt], -1, 255, -1)
        comp_fg = (comp_mask > 0) & (fg_color > 0)
        fg_pixels = max(1, int(np.count_nonzero(comp_fg)))

        # Split adjacent/touching colored objects if >=2 distinct hue buckets exist
        active_buckets = []
        for h_rule in hue_masks_defs:
            bucket = (comp_fg & h_rule).astype(np.uint8) * 255
            bucket = cv2.morphologyEx(bucket, cv2.MORPH_OPEN, k3)
            if np.count_nonzero(bucket) >= max(min_area_c, 0.15 * fg_pixels):
                active_buckets.append(bucket)

        if len(active_buckets) >= 2:
            for bucket in active_buckets:
                sub_cnts, _ = cv2.findContours(bucket, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                color_entries.extend(
                    _scale_and_filter_contours(
                        sub_cnts, sx_c, sy_c, w_img, h_img,
                        min_area_small=min_area_c,
                        max_area_small=max_area_c,
                        check_tray_margins=False,
                    )
                )
        else:
            color_entries.extend(
                _scale_and_filter_contours(
                    [cnt], sx_c, sy_c, w_img, h_img,
                    min_area_small=min_area_c,
                    max_area_small=max_area_c,
                    check_tray_margins=False,
                )
            )

    if color_entries:
        return _suppress_duplicates(color_entries)

    # ------------------------------------------------------------------
    # Pass 2: Achromatic / Dark-tray fallback (e.g. matte dark tray or synthetic tests)
    # ------------------------------------------------------------------
    mono_dim = 640
    small_mono = cv2.resize(rgb, (mono_dim, mono_dim), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small_mono, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    border_strip = np.concatenate([gray[:6, :].ravel(), gray[-6:, :].ravel(), gray[:, :6].ravel(), gray[:, -6:].ravel()])
    border_med = float(np.median(border_strip))
    border_std = float(np.std(border_strip))

    if border_med < 85.0:
        thresh_mode = cv2.THRESH_BINARY + cv2.THRESH_OTSU
        min_contrast = 30.0
    elif border_std < 12.0:
        thresh_mode = cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        min_contrast = 45.0
    else:
        # Uneven cardboard/white paper tray with zero chromatic cargo -> empty tray (NO_OBJECT)
        return []

    _, thresh = cv2.threshold(blurred, 0, 255, thresh_mode)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    fg_vals = gray[closed > 0]
    bg_vals = gray[closed == 0]
    if fg_vals.size == 0 or bg_vals.size == 0:
        return []
    if abs(float(np.mean(fg_vals)) - float(np.mean(bg_vals))) < min_contrast:
        return []

    contours_m, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours_m:
        return []

    mono_entries = _scale_and_filter_contours(
        contours_m,
        scale_x=float(w_img) / float(mono_dim),
        scale_y=float(h_img) / float(mono_dim),
        w_img=w_img,
        h_img=h_img,
        min_area_small=float(min_contour_area),
        max_area_small=0.85 * mono_dim * mono_dim,
        check_tray_margins=False,
    )
    return _suppress_duplicates(mono_entries)


def detect_tray_foreground(
    image_input: Union[str, Path, Image.Image, np.ndarray],
    min_contour_area: int = EMPTY_TRAY_MAX_CONTOUR_AREA,
    is_bgr: bool = False,
) -> Optional[Tuple[int, int, int, int]]:
    """Return the primary foreground bounding box on the tray, or None when empty."""
    boxes = detect_tray_foregrounds(image_input, min_contour_area=min_contour_area, is_bgr=is_bgr)
    return boxes[0] if boxes else None


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
