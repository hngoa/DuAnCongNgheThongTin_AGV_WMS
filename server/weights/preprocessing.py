"""SLAM_KLTN — Standardized Preprocessing Module for Hybrid MobileNetV3-Small.

Guarantees:
1. Strict EXIF orientation transpose via ImageOps.exif_transpose().
2. RGB color space consistency (converts BGR/RGBA/Grayscale to RGB).
3. Rigorous bounding box validation: rejects inverted coordinates and out-of-frame bboxes.
4. Context-aware ROI crop: expands bounding box to square with pad_ratio margin.
5. Idempotent Letterbox Padding: pads to target_size x target_size with neutral gray (114, 114, 114).
6. ImageNet normalization: Mean [0.485, 0.456, 0.406], Std [0.229, 0.224, 0.225].
7. ZERO double-crop and ZERO double-padding:
   - Pre-cropped ROIs are only letterboxed (no secondary crop).
   - Images already 224x224 are returned unchanged (no redundant padding).
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple, Union

import cv2
import numpy as np
from PIL import Image, ImageOps
import torch

IMAGE_SIZE = 224
NORM_MEAN = [0.485, 0.456, 0.406]
NORM_STD = [0.229, 0.224, 0.225]
PAD_RGB = (114, 114, 114)


def validate_bbox(
    bbox: Tuple[int, int, int, int],
    img_width: int,
    img_height: int,
    min_box_dim: int = 5,
) -> Tuple[int, int, int, int]:
    """Strictly validate bounding box coordinates.

    Args:
        bbox: (x1, y1, x2, y2)
        img_width: Image width in pixels
        img_height: Image height in pixels
        min_box_dim: Minimum allowed width/height after clipping

    Returns:
        Clipped valid coordinates (cx1, cy1, cx2, cy2)

    Raises:
        ValueError: If bbox is inverted, degenerate, or completely out-of-bounds.
    """
    if len(bbox) != 4:
        raise ValueError(f"Bounding box must contain exactly 4 coordinates (x1, y1, x2, y2), got: {bbox}")

    x1, y1, x2, y2 = bbox
    x1, y1, x2, y2 = int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))

    # 1. Reject inverted or degenerate coordinates
    if x2 <= x1:
        raise ValueError(f"Inverted horizontal coordinates: x1 ({x1}) >= x2 ({x2})")
    if y2 <= y1:
        raise ValueError(f"Inverted vertical coordinates: y1 ({y1}) >= y2 ({y2})")

    # 2. Reject out-of-frame bbox
    if x2 <= 0 or y2 <= 0 or x1 >= img_width or y1 >= img_height:
        raise ValueError(
            f"Bounding box ({x1}, {y1}, {x2}, {y2}) lies entirely outside image dimensions ({img_width}x{img_height})"
        )

    # 3. Clip within frame
    cx1 = max(0, min(x1, img_width))
    cy1 = max(0, min(y1, img_height))
    cx2 = max(0, min(x2, img_width))
    cy2 = max(0, min(y2, img_height))

    if (cx2 - cx1) < min_box_dim or (cy2 - cy1) < min_box_dim:
        raise ValueError(
            f"Bounding box clipped area is degenerate (< {min_box_dim}px): ({cx1}, {cy1}, {cx2}, {cy2})"
        )

    return cx1, cy1, cx2, cy2


def is_blank_or_textureless(
    img: Union[Image.Image, np.ndarray],
    max_std_thresh: float = 8.0,
) -> bool:
    """Check if an image or crop is completely uniform or featureless (e.g. empty bright or dark tray)."""
    if isinstance(img, Image.Image):
        gray = np.array(img.convert("L"))
    elif isinstance(img, np.ndarray):
        if img.ndim == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if img.shape[2] == 3 else cv2.COLOR_BGR2GRAY)
        else:
            gray = img
    else:
        return True

    # Uniform surface (empty bright or dark tray) has negligible contrast / pixel standard deviation
    if float(np.std(gray)) < max_std_thresh:
        return True

    return False


def load_image_rgb(
    image_input: Union[str, Path, Image.Image, np.ndarray],
    is_bgr: bool = False,
) -> Image.Image:
    """Load an image and apply EXIF orientation transpose, returning an RGB PIL Image.

    Supports file paths, Path objects, PIL Images, and NumPy arrays (grayscale, RGB, BGR, RGBA).
    """
    if isinstance(image_input, Image.Image):
        img = image_input.copy()
    elif isinstance(image_input, np.ndarray):
        arr = image_input
        if arr.size == 0:
            raise ValueError("Input numpy array is empty.")
        if arr.ndim == 2:
            img = Image.fromarray(arr).convert("RGB")
        elif arr.ndim == 3:
            if is_bgr and arr.shape[2] == 3:
                arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
            elif is_bgr and arr.shape[2] == 4:
                arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2RGB)
            if arr.shape[2] == 3:
                img = Image.fromarray(arr)
            elif arr.shape[2] == 4:
                img = Image.fromarray(arr[:, :, :3])
            else:
                raise ValueError(f"Unsupported channel count: {arr.shape[2]}")
        else:
            raise ValueError(f"Unsupported array dimensions: {arr.ndim}")
    else:
        path = Path(image_input)
        if not path.is_file():
            raise FileNotFoundError(f"Image file does not exist: {path}")
        if path.stat().st_size == 0:
            raise ValueError(f"Image file is empty (0 bytes): {path}")
        with Image.open(path) as raw_img:
            img = ImageOps.exif_transpose(raw_img)
            return img.convert("RGB")

    img = ImageOps.exif_transpose(img)
    return img.convert("RGB")


def letterbox_pad(
    img: Image.Image,
    target_size: int = IMAGE_SIZE,
    pad_color: Tuple[int, int, int] = PAD_RGB,
) -> Image.Image:
    """Resize image maintaining aspect ratio and pad to target_size x target_size.

    Idempotent: If img is already target_size x target_size, returns copy unchanged.
    """
    w, h = img.size
    if w <= 0 or h <= 0:
        raise ValueError(f"Invalid image dimensions: {w}x{h}")
    if w == target_size and h == target_size:
        return img.copy()

    scale = float(target_size) / float(max(w, h))
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))

    resized = img.resize((nw, nh), Image.Resampling.BILINEAR)
    canvas = Image.new("RGB", (target_size, target_size), pad_color)
    paste_x = (target_size - nw) // 2
    paste_y = (target_size - nh) // 2
    canvas.paste(resized, (paste_x, paste_y))
    return canvas


def crop_roi_with_context(
    img: Image.Image,
    bbox: Tuple[int, int, int, int],
    pad_ratio: float = 0.14,
    target_size: int = IMAGE_SIZE,
    pad_color: Tuple[int, int, int] = PAD_RGB,
) -> Image.Image:
    """Crop bounding box with context margin expanded to a square, then letterbox pad."""
    w, h = img.size
    x1, y1, x2, y2 = validate_bbox(bbox, w, h)

    bw = x2 - x1
    bh = y2 - y1
    side = max(bw, bh) * (1.0 + 2.0 * pad_ratio)
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)

    nx1 = max(0, int(round(cx - 0.5 * side)))
    ny1 = max(0, int(round(cy - 0.5 * side)))
    nx2 = min(w, int(round(cx + 0.5 * side)))
    ny2 = min(h, int(round(cy + 0.5 * side)))

    crop = img.crop((nx1, ny1, nx2, ny2))
    return letterbox_pad(crop, target_size=target_size, pad_color=pad_color)


def preprocess_for_mobilenet(
    image_input: Union[str, Path, Image.Image, np.ndarray],
    bbox: Optional[Tuple[int, int, int, int]] = None,
    pad_ratio: float = 0.14,
    target_size: int = IMAGE_SIZE,
    is_bgr: bool = False,
) -> torch.Tensor:
    """Preprocess image into normalized NCHW torch.Tensor for MobileNetV3-Small.

    - If bbox is given: strictly validates coordinates, crops with context, letterbox pads.
    - If bbox is None: treats image_input as pre-cropped ROI, applying only letterbox pad.
    - Strictly prevents double-cropping or double-padding.
    """
    img = load_image_rgb(image_input, is_bgr=is_bgr)

    if bbox is not None:
        padded = crop_roi_with_context(img, bbox=bbox, pad_ratio=pad_ratio, target_size=target_size)
    else:
        padded = letterbox_pad(img, target_size=target_size, pad_color=PAD_RGB)

    arr = np.array(padded, dtype=np.float32) / 255.0
    arr = (arr - np.array(NORM_MEAN, dtype=np.float32)) / np.array(NORM_STD, dtype=np.float32)
    tensor = torch.from_numpy(arr).permute(2, 0, 1).unsqueeze(0).float()
    return tensor
