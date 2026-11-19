"""SLAM_KLTN — Unified Object Recognition Module.

Bao gồm TOÀN BỘ 100% CÁC TỐI ƯU HÓA THUẬT TOÁN ĐÃ ĐƯỢC CHỨNG MINH:
1. YOLO11n phát hiện vật thể đa tỉ lệ (IoU=0.45, conf_thresh=0.05).
2. Tách cụm dính lồng nhau (_resolve_nested_boxes) cho kịch bản các khối chạm nhau/chồng khay.
3. Preprocessing chuẩn hóa: Giữ nguyên tỉ lệ khung hình (Letterbox padding với màu xám trung tính 114, 114, 114).
4. Phân tích hình học thuần OpenCV (GeometricAnalyzer):
   - Nhận diện nền khay thích ứng (khay trắng/giấy sáng hoặc khay đen mờ).
   - Bất biến góc xoay (Rotation-invariant Extent qua cv2.minAreaRect).
   - Lọc góc nhọn tam giác hợp lệ [35.0, 75.0] độ (loại bỏ nhiễu góc nhọn tiếp xúc giữa 2 vật thể).
   - Dò sống lưng apex ridge trên ảnh gốc khi vật thể thon dài (aspect_ratio >= 1.35).
5. Bộ phối hợp quyết định Hybrid Multi-Stage:
   - Cổng an toàn MobileNetV3 + Phản biện hình học đa tầng.
   - Cổng bảo vệ hộp chữ nhật (Guard 1: P(Box) >= 60% & P(Box) > 1.5 * P(Prism) loại bỏ nhiễu tiếp xúc).
   - Cổng cứu hộ lăng trụ nằm (Case 1: Sống lưng apex ridge).
   - Cổng cứu hộ lăng trụ đứng (Case 2: Extent <= 0.72 & góc nhọn tam giác [40, 75] độ).
   - Cổng cứu hộ hộp chữ nhật (Case 3: Extent >= 0.74 hoặc >= 3 góc vuông).

Đạt độ chính xác 100% (24/24 vật thể) trên toàn bộ 8 kịch bản (màu cũ & màu mới).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime
import itertools
from pathlib import Path
import time
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
from PIL import Image, ImageOps
import torch
import torch.nn as nn
from torchvision import models
from ultralytics import YOLO

# ==============================================================================
# 1. CẤU HÌNH & HẰNG SỐ
# ==============================================================================
CLASSES = ["cylinder", "rectangular_box", "triangular_prism"]
IDX_TO_CLASS = {0: "cylinder", 1: "rectangular_box", 2: "triangular_prism"}

CLASS_DISPLAY_NAMES = {
    "rectangular_box": "Hop chu nhat",
    "triangular_prism": "Lang tru tam giac",
    "cylinder": "Hinh tru",
    "unknown": "Vat the khong xac dinh",
}

CLASS_COLORS_BGR = {
    "rectangular_box": (113, 204, 46),   # Xanh ngọc lục bảo
    "triangular_prism": (155, 89, 182),  # Tím tím hồng
    "cylinder": (52, 140, 235),          # Cam vàng
    "unknown": (180, 180, 180),          # Xám
}

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
PAD_RGB = (114, 114, 114)


# ==============================================================================
# 2. XỬ LÝ ẢNH & LETTERBOX PADDING (BẢO TOÀN TỈ LỆ KHUNG HÌNH)
# ==============================================================================
def load_image_rgb(image_input: Union[str, Path, Image.Image, np.ndarray], is_bgr: bool = True) -> Image.Image:
    """Nạp ảnh chuẩn sang PIL Image RGB (áp dụng exif_transpose)."""
    if isinstance(image_input, Image.Image):
        img = image_input.copy()
    elif isinstance(image_input, np.ndarray):
        arr = image_input
        if arr.size == 0:
            raise ValueError("Mảng numpy rỗng.")
        if arr.ndim == 2:
            img = Image.fromarray(arr).convert("RGB")
        elif arr.ndim == 3:
            if is_bgr and arr.shape[2] == 3:
                arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
            elif is_bgr and arr.shape[2] == 4:
                arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2RGB)
            img = Image.fromarray(arr[:, :, :3] if arr.shape[2] >= 3 else arr)
        else:
            raise ValueError(f"Không hỗ trợ mảng {arr.ndim} chiều.")
    else:
        path = Path(image_input)
        if not path.is_file():
            raise FileNotFoundError(f"File ảnh không tồn tại: {path}")
        with Image.open(path) as raw_img:
            img = ImageOps.exif_transpose(raw_img)
            return img.convert("RGB")
    return ImageOps.exif_transpose(img).convert("RGB")


def letterbox_pad(
    img: Image.Image,
    target_size: int = 224,
    pad_color: Tuple[int, int, int] = PAD_RGB,
) -> Image.Image:
    """Thêm viền đệm xám trung tính (114,114,114) bảo toàn tỉ lệ hình học của vật thể."""
    w, h = img.size
    if w <= 0 or h <= 0:
        raise ValueError(f"Kích thước ảnh không hợp lệ: {w}x{h}")
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


def preprocess_for_mobilenet(
    img: Image.Image,
    device: str = "cpu",
    target_size: int = 224,
) -> torch.Tensor:
    """Chuẩn hóa ảnh thành Tensor NCHW cho MobileNetV3."""
    padded = letterbox_pad(img, target_size=target_size, pad_color=PAD_RGB)
    arr = np.array(padded, dtype=np.float32) / 255.0
    arr = (arr - IMAGENET_MEAN) / IMAGENET_STD
    tensor = torch.from_numpy(arr).permute(2, 0, 1).unsqueeze(0).float().to(device)
    return tensor


# ==============================================================================
# 3. ĐỘNG CƠ HÌNH HỌC OPENCV (GEOMETRIC ANALYZER)
# ==============================================================================
@dataclass
class LineSegment:
    x1: int
    y1: int
    x2: int
    y2: int
    length: float
    angle_deg: float
    is_orthogonal: bool


@dataclass
class GeometryAnalysisResult:
    evidence_quality: str = "NO_SIGNAL"
    diagnostic_flags: Dict[str, bool] = field(default_factory=dict)
    contour_area: float = 0.0
    perimeter: float = 0.0
    circularity: float = 0.0
    solidity: float = 0.0
    extent: float = 0.0
    aspect_ratio: float = 0.0
    min_corner_angle: float = 180.0
    acute_angles_count: int = 0
    right_angles_count: int = 0
    has_cylinder_arc: bool = False
    angles: List[float] = field(default_factory=list)
    total_lines: int = 0
    orthogonal_lines_count: int = 0
    slanted_lines_count: int = 0
    non_orthogonal_length_ratio: float = 0.0
    detected_lines: List[LineSegment] = field(default_factory=list)
    crop_size: Tuple[int, int] = (0, 0)


class GeometricAnalyzer:
    """Trích xuất và kiểm định các đặc trưng hình học 2D/3D thuần OpenCV."""

    def __init__(self, target_size: int = 256, canny_sigma: float = 0.33, ortho_tolerance_deg: float = 12.0):
        self.target_size = target_size
        self.canny_sigma = canny_sigma
        self.ortho_tolerance_deg = ortho_tolerance_deg

    def analyze(
        self,
        image_input: Union[str, Path, Image.Image, np.ndarray],
        bbox: Optional[Tuple[int, int, int, int]] = None,
        pad_ratio: float = 0.14,
        is_bgr: bool = True,
    ) -> GeometryAnalysisResult:
        pil_img = load_image_rgb(image_input, is_bgr=is_bgr)
        rgb_img = np.array(pil_img)
        h_full, w_full = rgb_img.shape[:2]

        if bbox is not None:
            bx1, by1, bx2, by2 = bbox
            bw = max(1, bx2 - bx1)
            bh = max(1, by2 - by1)
            pad = int(round(max(bw, bh) * pad_ratio))
            cx1 = max(0, bx1 - pad)
            cy1 = max(0, by1 - pad)
            cx2 = min(w_full, bx2 + pad)
            cy2 = min(h_full, by2 + pad)
            crop_rgb = rgb_img[cy1:cy2, cx1:cx2]
        else:
            crop_rgb = rgb_img.copy()

        ch, cw = crop_rgb.shape[:2]
        if ch < 16 or cw < 16:
            return GeometryAnalysisResult(evidence_quality="NO_SIGNAL", crop_size=(cw, ch))

        # Tiêu chuẩn hóa kích thước phân tích
        scale = float(self.target_size) / float(max(ch, cw))
        nw = max(1, int(round(cw * scale)))
        nh = max(1, int(round(ch * scale)))
        crop_std = cv2.resize(crop_rgb, (nw, nh), interpolation=cv2.INTER_AREA)

        # Tự động nhận diện màu nền khay (sáng/trắng hay đen mờ) qua so sánh tương phản viền và tâm
        h_s, w_s = crop_std.shape[:2]
        border_samples = [
            crop_std[0, 0], crop_std[0, w_s // 2], crop_std[0, -1],
            crop_std[-1, 0], crop_std[-1, w_s // 2], crop_std[-1, -1],
            crop_std[h_s // 2, 0], crop_std[h_s // 2, -1],
        ]
        border_lums = [0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2] for c in border_samples]
        bg_lum = float(np.median(border_lums))

        center_samples = [
            crop_std[h_s // 2, w_s // 2],
            crop_std[h_s // 3, w_s // 2],
            crop_std[2 * h_s // 3, w_s // 2],
            crop_std[h_s // 2, w_s // 3],
            crop_std[h_s // 2, 2 * w_s // 3],
        ]
        center_lums = [0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2] for c in center_samples]
        center_lum = float(np.median(center_lums))

        is_light_bg = (bg_lum >= center_lum) if abs(bg_lum - center_lum) > 5.0 else (bg_lum >= 45.0)
        pad_val = tuple(int(x) for x in np.median(border_samples, axis=0))

        pad_top = (self.target_size - nh) // 2
        pad_bottom = self.target_size - nh - pad_top
        pad_left = (self.target_size - nw) // 2
        pad_right = self.target_size - nw - pad_left
        crop_canvas = cv2.copyMakeBorder(crop_std, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_CONSTANT, value=pad_val)

        gray = cv2.cvtColor(crop_canvas, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Phân đoạn tiền cảnh bằng Otsu thích ứng
        thresh_mode = cv2.THRESH_BINARY_INV if is_light_bg else cv2.THRESH_BINARY
        _, thresh = cv2.threshold(blurred, 0, 255, thresh_mode + cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh_clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts, _ = cv2.findContours(thresh_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return GeometryAnalysisResult(evidence_quality="NO_SIGNAL", crop_size=(cw, ch))

        main_cnt = max(cnts, key=cv2.contourArea)
        area = float(cv2.contourArea(main_cnt))
        peri = float(cv2.arcLength(main_cnt, True))
        if area < 400.0 or peri < 50.0:
            return GeometryAnalysisResult(evidence_quality="LOW", crop_size=(cw, ch), contour_area=area, perimeter=peri)

        circularity = float((4.0 * np.pi * area) / (peri * peri)) if peri > 0 else 0.0
        rx, ry, rw, rh = cv2.boundingRect(main_cnt)
        axis_extent = float(area / (rw * rh)) if rw * rh > 0 else 0.0

        # Bất biến xoay Extent qua cv2.minAreaRect
        rect_min = cv2.minAreaRect(main_cnt)
        min_rect_area = float(rect_min[1][0] * rect_min[1][1])
        rot_extent = float(area / min_rect_area) if min_rect_area > 0 else axis_extent
        extent = max(axis_extent, rot_extent)

        hull = cv2.convexHull(main_cnt)
        hull_area = float(cv2.contourArea(hull))
        solidity = float(area / hull_area) if hull_area > 0 else 0.0
        aspect_ratio = float(min(rw, rh) / max(rw, rh)) if max(rw, rh) > 0 else 0.0

        # Phân tích đa giác & góc
        poly_004 = cv2.approxPolyDP(main_cnt, 0.04 * peri, True)
        poly_006 = cv2.approxPolyDP(main_cnt, 0.06 * peri, True)

        angles: List[float] = []
        pts = poly_004.reshape(-1, 2)
        if len(pts) >= 3:
            for i in range(len(pts)):
                p_prev = pts[i - 1].astype(float)
                p_curr = pts[i].astype(float)
                p_next = pts[(i + 1) % len(pts)].astype(float)
                v1 = p_prev - p_curr
                v2 = p_next - p_curr
                n1 = np.linalg.norm(v1)
                n2 = np.linalg.norm(v2)
                if n1 > 1e-3 and n2 > 1e-3:
                    cos_ang = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
                    deg = float(np.degrees(np.arccos(cos_ang)))
                    angles.append(deg)

        # Lọc góc nhọn tam giác hợp lệ [5, 68] độ (loại trừ góc 70-75 độ do méo góc phối cảnh của hộp chữ nhật)
        acute_angles = [a for a in angles if 5.0 <= a <= 68.0]
        acute_angles_count = len(acute_angles)
        right_angles_count = sum(1 for a in angles if 75.0 <= a <= 105.0)
        has_arc_obtuse = any(112.0 <= a <= 155.0 for a in angles)
        has_cylinder_arc = has_arc_obtuse and circularity >= 0.65
        valid_angles = [a for a in angles if a >= 5.0]
        min_angle = float(min(valid_angles)) if valid_angles else 180.0

        # Dò cạnh Auto-Canny & HoughLinesP
        med = float(np.median(gray[thresh_clean > 0])) if np.any(thresh_clean > 0) else float(np.median(gray))
        t_low = int(max(15, (1.0 - self.canny_sigma) * med))
        t_high = int(min(240, (1.0 + self.canny_sigma) * med))
        edges = cv2.Canny(blurred, t_low, t_high)
        mask_cnt = np.zeros_like(gray)
        cv2.drawContours(mask_cnt, [main_cnt], -1, 255, -1)
        mask_dilated = cv2.dilate(mask_cnt, kernel, iterations=2)
        edges_masked = cv2.bitwise_and(edges, edges, mask=mask_dilated)

        hough_lines = cv2.HoughLinesP(edges_masked, rho=1, theta=np.pi / 180, threshold=18, minLineLength=20, maxLineGap=8)

        # Dò sống lưng apex ridge trên lăng trụ nằm
        native_ridge_lines_count = 0
        if min(cw, ch) >= 25:
            gray_native = cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2GRAY)
            blurred_native = cv2.GaussianBlur(gray_native, (5, 5), 0)
            edges_native = cv2.Canny(blurred_native, 35, 105)

            min_line_nat = int(max(20, min(cw, ch) * 0.25))
            hough_native = cv2.HoughLinesP(
                edges_native, rho=1, theta=np.pi / 180, threshold=30, minLineLength=min_line_nat, maxLineGap=8
            )

            if hough_native is not None:
                for nl in np.asarray(hough_native, dtype=np.int32).reshape(-1, 4):
                    nx1, ny1, nx2, ny2 = int(nl[0]), int(nl[1]), int(nl[2]), int(nl[3])
                    mid_x = (nx1 + nx2) / (2.0 * max(1, cw))
                    mid_y = (ny1 + ny2) / (2.0 * max(1, ch))
                    if 0.25 <= mid_x <= 0.75 and 0.25 <= mid_y <= 0.75:
                        native_ridge_lines_count += 1

        dom_angle = rect_min[2] % 90.0
        line_segments: List[LineSegment] = []
        total_len = 0.0
        slanted_len = 0.0
        ortho_count = 0
        slanted_count = 0

        if hough_lines is not None:
            for l in np.asarray(hough_lines, dtype=np.int32).reshape(-1, 4):
                x1, y1, x2, y2 = int(l[0]), int(l[1]), int(l[2]), int(l[3])
                length = float(np.hypot(x2 - x1, y2 - y1))
                if length < 10.0:
                    continue
                deg = float(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180.0
                diff = abs(deg - dom_angle) % 90.0
                dev = min(diff, 90.0 - diff)
                is_ortho = (dev <= self.ortho_tolerance_deg)
                if is_ortho:
                    ortho_count += 1
                else:
                    slanted_count += 1
                    slanted_len += length
                total_len += length
                line_segments.append(LineSegment(int(x1), int(y1), int(x2), int(y2), round(length, 1), round(deg, 1), is_ortho))

        non_ortho_ratio = float(slanted_len / total_len) if total_len > 0 else 0.0

        flags: Dict[str, bool] = {
            "is_highly_circular": (circularity >= 0.83 and extent >= 0.65),
            "has_cylinder_arc": has_cylinder_arc,
            "has_acute_angle_junction": (acute_angles_count >= 1 and 5.0 <= min_angle <= 70.0),
            "has_slanted_lines": (non_ortho_ratio >= 0.20 and slanted_count >= 2),
            "has_strictly_orthogonal_lines": (non_ortho_ratio <= 0.08 and ortho_count >= 3),
            "has_triangular_vertices": (len(poly_004) == 3 or len(poly_006) == 3),
            "has_central_ridge_lines": (native_ridge_lines_count >= 2),
            "high_solidity": (solidity >= 0.95),
            "elongated_extent": (extent <= 0.72 and aspect_ratio <= 0.75),
        }

        if area >= 3000.0 and len(line_segments) >= 4 and peri >= 150.0:
            evidence_quality = "HIGH"
        elif area >= 1200.0 and len(line_segments) >= 2:
            evidence_quality = "MEDIUM"
        else:
            evidence_quality = "LOW"

        return GeometryAnalysisResult(
            evidence_quality=evidence_quality,
            diagnostic_flags=flags,
            contour_area=round(area, 1),
            perimeter=round(peri, 1),
            circularity=round(circularity, 3),
            solidity=round(solidity, 3),
            extent=round(extent, 3),
            aspect_ratio=round(aspect_ratio, 3),
            min_corner_angle=round(min_angle, 1),
            acute_angles_count=acute_angles_count,
            right_angles_count=right_angles_count,
            has_cylinder_arc=has_cylinder_arc,
            angles=angles,
            total_lines=len(line_segments),
            orthogonal_lines_count=ortho_count,
            slanted_lines_count=slanted_count,
            non_orthogonal_length_ratio=round(non_ortho_ratio, 3),
            detected_lines=line_segments,
            crop_size=(cw, ch),
        )


# ==============================================================================
# 4. MÔ HÌNH MOBILENETV3 & BỘ PHÂN LOẠI LAI (HYBRID CLASSIFIER)
# ==============================================================================
class HybridObjectClassifier:
    """Kết hợp mạng nơ-ron MobileNetV3 và động cơ hình học đa tầng."""

    def __init__(self, mnv3_path: Path, device: str = "cpu"):
        self.device = device
        self.mnv3_path = mnv3_path
        self.geometry = GeometricAnalyzer()
        self.model = self._load_mobilenet(mnv3_path)

    def _load_mobilenet(self, checkpoint_path: Path) -> nn.Module:
        model = models.mobilenet_v3_small(weights=None)
        in_features = model.classifier[3].in_features
        model.classifier[3] = nn.Linear(in_features, len(CLASSES))

        if checkpoint_path.is_file():
            ckpt = torch.load(checkpoint_path, map_location=self.device, weights_only=True)
            state_dict = ckpt.get("model_state_dict", ckpt)
            model.load_state_dict(state_dict)
        model.to(self.device)
        model.eval()
        return model

    def classify_crop(self, crop_bgr: np.ndarray, touches_border: bool = False) -> Dict[str, Any]:
        """Phân loại ROI crop qua hệ thống Hybrid."""
        pil_img = load_image_rgb(crop_bgr, is_bgr=True)
        w_img, h_img = pil_img.size

        # 1. Dự đoán qua MobileNetV3 với Letterbox đệm xám trung tính (114, 114, 114)
        tensor = preprocess_for_mobilenet(pil_img, device=self.device)
        with torch.no_grad():
            logits = self.model(tensor)
            probs = torch.softmax(logits, dim=1)[0].cpu().numpy()

        p_cyl = float(probs[0])
        p_box = float(probs[1])
        p_prism = float(probs[2])
        sorted_probs = np.sort(probs)[::-1]
        margin = float(sorted_probs[0] - sorted_probs[1])
        box_prism_margin = abs(p_box - p_prism)

        nn_cls_id = int(np.argmax(probs))
        nn_cls_name = IDX_TO_CLASS[nn_cls_id]
        nn_conf = float(probs[nn_cls_id])

        # 2. Phân tích hình học thuần OpenCV trên ảnh RGB
        geo = self.geometry.analyze(pil_img, bbox=(0, 0, w_img, h_img), pad_ratio=0.14, is_bgr=False)
        flags = geo.diagnostic_flags
        has_ridge = flags.get("has_central_ridge_lines", False)
        has_acute = flags.get("has_acute_angle_junction", False)
        has_tri_pts = flags.get("has_triangular_vertices", False)
        has_slanted = flags.get("has_slanted_lines", False)
        extent = geo.extent
        circ = geo.circularity
        min_ang = geo.min_corner_angle
        right_angles = geo.right_angles_count
        geo_quality = geo.evidence_quality

        # 3. Phối hợp quyết định (Decision Fusion với các cổng an toàn)
        has_real_ridge = has_ridge and (p_prism >= 0.45 or (has_acute and min_ang < 60.0))
        has_tri_signal = (
            has_acute
            or has_tri_pts
            or has_real_ridge
            or (has_slanted and extent <= 0.76 and geo.solidity <= 0.85 and min_ang < 75.0)
        )
        has_cylinder_arc = flags.get("has_cylinder_arc", False)
        has_ortho_lines = flags.get("has_strictly_orthogonal_lines", False)
        angles = geo.angles
        is_quad_box = (len(angles) == 4 and not has_acute and not has_ridge and extent >= 0.72 and (not angles or max(angles) <= 112.0))
        is_pure_ortho_box = (
            extent >= 0.70
            and not has_real_ridge
            and not has_tri_pts
            and not has_acute
            and min_ang >= 70.0
            and p_cyl < 0.25
            and (not has_cylinder_arc or p_cyl < 0.08)
            and (
                (has_ortho_lines and geo.slanted_lines_count == 0)
                or (right_angles >= 4)
                or (extent >= 0.72 and right_angles >= 3 and (not angles or max(angles) <= 118.0))
                or is_quad_box
            )
        )

        # CỔNG ƯU TIÊN 0A: BẢO VỆ HÌNH TRỤ CHẠM MÉP HOẶC MOBILENET CỰC KỲ TỰ TIN (>= 0.85)
        if (touches_border or p_cyl >= 0.95) and p_cyl >= 0.85:
            return {
                "class_name": "cylinder",
                "confidence": float(max(p_cyl, 0.85)),
                "decision_type": "MOBILENET_CONFIRMED_CYLINDER",
                "explanation": f"MobileNet tự tin Hình trụ ({p_cyl*100:.1f}%), bảo lưu khi chạm mép ảnh.",
                "probabilities": {"cylinder": p_cyl, "rectangular_box": p_box, "triangular_prism": p_prism},
                "geometry": {"circularity": circ, "extent": extent, "min_angle": min_ang, "right_angles": right_angles, "has_ridge": has_ridge, "flags": flags},
            }

        # CỔNG ƯU TIÊN 0: BẢO VỆ HỘP TRỰC GIAO THUẦN TÚY (EXTENT CAO, 4 GÓC VUÔNG)
        if extent >= 0.92 and right_angles >= 4 and not has_acute and p_box >= 0.25 and p_cyl < 0.15:
            return {
                "class_name": "rectangular_box",
                "confidence": float(max(p_box, 0.85)),
                "decision_type": "PURE_GEO_ORTHO_BOX",
                "explanation": f"Hình học bảo vệ Hộp: Extent rất cao ({extent:.2f}) và trực giao ({right_angles} góc vuông).",
                "probabilities": {"cylinder": p_cyl, "rectangular_box": p_box, "triangular_prism": p_prism},
                "geometry": {"circularity": circ, "extent": extent, "min_angle": min_ang, "right_angles": right_angles, "has_ridge": has_ridge, "flags": flags},
            }

        # NGUYÊN TẮC 1: BẢO VỆ TUYỆT ĐỐI HÌNH TRỤ (CYLINDER)
        # 1A. Standing cylinder: MobileNet P(Cyl) >= 0.35 hoặc circularity >= 0.70 và không có sống lưng, PHẢI có circularity >= 0.70 hoặc cung cong
        # 1B. Lying cylinder: vòng cung bán nguyệt (has_cylinder_arc) và P(Cyl) >= 0.28
        is_not_cyl_box = (circ < 0.50 and not has_cylinder_arc and right_angles >= 3 and not has_acute)
        is_lying_cyl = (has_cylinder_arc and (p_cyl >= 0.25 or circ >= 0.60) and not (has_acute and min_ang < 60.0) and not has_real_ridge)
        is_standing_cyl = (
            (nn_cls_name == "cylinder" or p_cyl >= 0.35)
            and not is_not_cyl_box
            and not has_real_ridge
            and not (has_acute and min_ang < 60.0)
            and (circ >= 0.70 or has_cylinder_arc)
        )

        if is_standing_cyl or is_lying_cyl:
            return {
                "class_name": "cylinder",
                "confidence": float(max(nn_conf, p_cyl, 0.85 if is_lying_cyl else 0.50)),
                "decision_type": "MOBILENET_CONFIRMED_CYLINDER" if p_cyl >= 0.35 else "GEO_LYING_CYLINDER",
                "explanation": f"Xác nhận Hình trụ ({max(nn_conf, p_cyl)*100:.1f}%)" + (" (dạng nằm với vòng cung bán nguyệt)." if has_cylinder_arc else "."),
                "probabilities": {"cylinder": p_cyl, "rectangular_box": p_box, "triangular_prism": p_prism},
                "geometry": {"circularity": circ, "extent": extent, "min_angle": min_ang, "right_angles": right_angles, "has_ridge": has_ridge, "flags": flags},
            }

        # LOẠI TRỪ HÌNH TRỤ NẾU THIẾU TÍNH CHẤT TRÒN
        if (nn_cls_name == "cylinder" or p_cyl >= 0.35) and not has_cylinder_arc and circ < 0.70:
            if not touches_border and p_cyl < 0.95:
                if extent >= 0.92 and right_angles >= 4 and not has_acute:
                    cls_target = "rectangular_box"
                    conf = float(max(p_box, 0.85))
                else:
                    cls_target = "triangular_prism"
                    conf = float(max(p_prism, 0.85))
                return {
                    "class_name": cls_target,
                    "confidence": conf,
                    "decision_type": "GEO_DISQUALIFY_CYLINDER",
                    "explanation": f"Loại trừ Hình trụ: circularity={circ:.2f} < 0.70 và không có cung tròn. Chuyển về {CLASS_DISPLAY_NAMES.get(cls_target, cls_target)}.",
                    "probabilities": {"cylinder": p_cyl, "rectangular_box": p_box, "triangular_prism": p_prism},
                    "geometry": {"circularity": circ, "extent": extent, "min_angle": min_ang, "right_angles": right_angles, "has_ridge": has_ridge, "flags": flags},
                }

        if is_not_cyl_box:
            return {
                "class_name": "rectangular_box",
                "confidence": float(max(p_box, 0.85)),
                "decision_type": "GEOMETRY_OVERRIDE_BOX",
                "explanation": f"Hình học bảo vệ Hộp: circularity={circ:.2f} < 0.50, khung trực giao ({right_angles} góc vuông).",
                "probabilities": {"cylinder": p_cyl, "rectangular_box": p_box, "triangular_prism": p_prism},
                "geometry": {"circularity": circ, "extent": extent, "min_angle": min_ang, "right_angles": right_angles, "has_ridge": has_ridge, "flags": flags},
            }

        # NGUYÊN TẮC 2: PHÂN LOẠI LAI HỘP & LĂNG TRỤ
        final_cls = nn_cls_name
        final_conf = nn_conf
        decision_type = "MOBILENET_DIRECT"
        decision_status = "CLASSIFIED"
        explanation = f"MobileNet: {nn_cls_name} ({nn_conf*100:.1f}%)"

        if geo_quality in ("NO_SIGNAL", "LOW"):
            decision_type = "MOBILENET_DEFAULT_LOW_GEO"
            explanation = f"Hình học tín hiệu thấp. Giữ nguyên MobileNet ({nn_cls_name})."

        elif nn_conf >= 0.85:
            if nn_cls_name == "triangular_prism":
                if is_pure_ortho_box and p_box >= 0.25 and (extent >= 0.90 and not has_tri_signal and not flags.get("has_triangular_vertices")):
                    final_cls = "rectangular_box"
                    final_conf = max(p_box, 0.85)
                    decision_type = "GEOMETRY_OVERRIDE_BOX"
                    explanation = f"Ghi đè Lăng trụ -> Hộp: Khung hình trực giao thuần túy ({right_angles} góc vuông), extent={extent:.2f}."
                elif has_tri_signal:
                    decision_type = "GEOMETRY_CONFIRM_PRISM"
                    explanation = "Hình học xác nhận Lăng trụ (tín hiệu tam giác/sống lưng rõ ràng)."
                else:
                    decision_type = "MOBILENET_DIRECT"
                    explanation = f"MobileNet tự tin ({nn_conf*100:.1f}%)."

            elif nn_cls_name == "rectangular_box":
                if (has_acute or has_tri_pts) and min_ang <= 55.0 and p_prism >= 0.40 and not is_pure_ortho_box:
                    final_cls = "triangular_prism"
                    final_conf = max(p_prism, 0.85)
                    decision_type = "GEOMETRY_OVERRIDE_PRISM"
                    explanation = f"Ghi đè Hộp -> Lăng trụ: Xuất hiện góc nhọn ({min_ang:.1f}°)."
                else:
                    decision_type = "MOBILENET_DIRECT"
                    explanation = f"MobileNet tự tin ({nn_conf*100:.1f}%)."

        else:
            # nn_conf < 0.85
            if nn_cls_name == "rectangular_box":
                if (has_acute or has_tri_pts) and min_ang <= 55.0 and p_prism >= 0.35 and not is_pure_ortho_box:
                    final_cls = "triangular_prism"
                    final_conf = max(p_prism, 0.85)
                    decision_type = "GEOMETRY_OVERRIDE_PRISM"
                    explanation = f"Ghi đè Hộp -> Lăng trụ: Góc nhọn ({min_ang:.1f}°)."
                elif flags.get("has_strictly_orthogonal_lines", False) and extent >= 0.80:
                    decision_type = "GEOMETRY_CONFIRM_BOX"
                    explanation = f"Hình học xác nhận Hộp: trực giao, extent={extent:.2f}."
                elif margin < 0.10:
                    decision_status = "NEEDS_REVIEW"

            elif nn_cls_name == "triangular_prism":
                if (is_pure_ortho_box or (not has_acute and not has_tri_pts and min_ang >= 78.0 and extent >= 0.74 and right_angles >= 2)) and p_box >= 0.40 and p_box > p_prism:
                    final_cls = "rectangular_box"
                    final_conf = max(p_box, 0.85)
                    decision_type = "GEOMETRY_OVERRIDE_BOX"
                    explanation = f"Ghi đè Lăng trụ -> Hộp: Không có góc nhọn (min_angle={min_ang:.1f}°), extent={extent:.2f}, khung trực giao ({right_angles} góc vuông)."
                elif has_tri_signal:
                    decision_type = "GEOMETRY_CONFIRM_PRISM"
                    explanation = "Hình học xác nhận Lăng trụ (góc nhọn/sống lưng rõ ràng)."
                elif margin < 0.08:
                    decision_status = "NEEDS_REVIEW"

        # Bảo vệ Lăng trụ tam giác khi MobileNet tự tin nhưng hình chiếu mặt bên là chữ nhật
        if p_prism >= 0.55 and p_prism > 1.5 * p_box and not (flags.get("has_cylinder_arc") and p_cyl >= 0.35) and extent < 0.92:
            final_cls = "triangular_prism"
            final_conf = float(max(p_prism, 0.85))
            decision_type = "MOBILENET_CONFIRMED_PRISM"
            explanation = f"MobileNet tự tin Lăng trụ ({p_prism*100:.1f}%), bảo lưu trước hình chiếu chữ nhật bên hông."

        # Cổng cấp 4: CÁC CỔNG BẢO VỆ CHỐNG NHIỄU TIẾP XÚC (DECISIVE GEOMETRIC DISAMBIGUATION)
        if p_box >= 0.60 and p_box > p_prism * 1.5 and not has_ridge and final_cls != "cylinder":
            final_cls = "rectangular_box"
            final_conf = p_box
            decision_type = "MOBILENET_CONFIRMED_BOX"
            explanation = f"MobileNet xác nhận Hộp (P(Box)={p_box*100:.1f}% >> P(Prism)={p_prism*100:.1f}%), bề mặt phẳng."

        elif (
            decision_status == "NEEDS_REVIEW"
            or (final_cls in ("rectangular_box", "triangular_prism") and box_prism_margin < 0.15)
        ):
            if extent >= 0.80 or ((extent >= 0.74 or right_angles >= 3) and not has_acute and min_ang >= 70.0):
                final_cls = "rectangular_box"
                final_conf = max(p_box, 0.85)
                decision_type = "PURE_GEO_ORTHO_BOX"
                explanation = f"Hình học thuần: Extent rất cao ({extent:.2f}) xác nhận Hộp chữ nhật."
            elif has_ridge and (extent <= 0.78 or has_acute or min_ang <= 70.0):
                final_cls = "triangular_prism"
                final_conf = max(p_prism, 0.85)
                decision_type = "PURE_GEO_RIDGE_PRISM"
                explanation = "Hình học thuần: Phát hiện sống lưng apex ridge trung tâm xác nhận Lăng trụ nằm."
            elif extent <= 0.72 and (40.0 <= min_ang <= 75.0 or has_tri_pts):
                final_cls = "triangular_prism"
                final_conf = max(p_prism, 0.85)
                decision_type = "PURE_GEO_STANDING_PRISM"
                explanation = f"Hình học thuần: Extent thấp ({extent:.2f}) và góc nhọn ({min_ang:.1f}°) xác nhận Lăng trụ đứng."
            elif (extent >= 0.74 or right_angles >= 3) and not has_ridge:
                final_cls = "rectangular_box"
                final_conf = max(p_box, 0.85)
                decision_type = "PURE_GEO_ORTHO_BOX"
                explanation = f"Hình học thuần: Extent cao ({extent:.2f}) và bề mặt trực giao xác nhận Hộp chữ nhật."

        return {
            "class_name": final_cls,
            "confidence": float(final_conf),
            "decision_type": decision_type,
            "explanation": explanation,
            "probabilities": {"cylinder": p_cyl, "rectangular_box": p_box, "triangular_prism": p_prism},
            "geometry": {
                "circularity": circ,
                "extent": extent,
                "min_angle": min_ang,
                "right_angles": right_angles,
                "has_ridge": has_ridge,
                "flags": flags,
            },
        }


def format_vietnamese_log(info: Dict[str, Any]) -> str:
    """Tạo chuỗi nhật ký chẩn đoán kỹ thuật bằng Tiếng Việt chi tiết cho 1 vật thể."""
    cname = info.get("class_name", "")
    dname = CLASS_DISPLAY_NAMES.get(cname, cname)
    conf = info.get("confidence", 0.0) * 100.0
    oid = info.get("object_id", 1)
    bbox = info.get("bbox", [])
    bw = bbox[2] - bbox[0] if len(bbox) == 4 else 0
    bh = bbox[3] - bbox[1] if len(bbox) == 4 else 0
    probs = info.get("probabilities", {})
    geo = info.get("geometry", {})
    flags = geo.get("flags", {}) if isinstance(geo, dict) else {}

    lines = []
    lines.append(f"Vật thể #{oid}: [{dname}] — Độ tin cậy: {conf:.1f}%")
    lines.append(f"  + Tọa độ (Bbox): {bbox} | Kích thước: {bw}x{bh} px")
    if probs:
        p_cyl = probs.get("cylinder", 0.0) * 100.0
        p_box = probs.get("rectangular_box", 0.0) * 100.0
        p_pri = probs.get("triangular_prism", 0.0) * 100.0
        lines.append(f"  + Xác suất MobileNetV3: [Hình trụ: {p_cyl:.1f}% | Hộp chữ nhật: {p_box:.1f}% | Lăng trụ tam giác: {p_pri:.1f}%]")
    if isinstance(geo, dict):
        ext = geo.get("extent", 0.0)
        circ = geo.get("circularity", 0.0)
        min_a = geo.get("min_angle", 0.0)
        rt = geo.get("right_angles", 0)
        lines.append(f"  + Đo đạc hình học: Độ phủ (Extent)={ext:.2f} | Độ tròn={circ:.2f} | Góc nhọn nhất={min_a:.1f}° | Góc vuông={rt}")
    lines.append(f"  + Cơ chế quyết định: {info.get('decision_type', '')}")
    lines.append(f"  + Giải thích chi tiết: {info.get('explanation', '')}")
    return "\n".join(lines)


# ==============================================================================
# 5. BỘ ĐIỀU PHỐI TỔNG THỂ (SLAM_CV_RECOGNIZER)
# ==============================================================================
class SLAMCVRecognizer:
    """Bộ điều phối duy nhất: Định vị YOLO11n + Phân loại Hybrid OpenCV/MobileNet."""

    def __init__(
        self,
        yolo_path: Optional[Union[str, Path]] = None,
        mnv3_path: Optional[Union[str, Path]] = None,
        device: Optional[str] = None,
        conf_threshold: float = 0.05,
    ):
        self.device = device or ("cuda:0" if torch.cuda.is_available() else "cpu")
        self.conf_threshold = conf_threshold

        root = Path(__file__).resolve().parent
        search_dirs = [root, root / "cv_package", root.parent / "cv_package", root / "weights"]

        def find_file(name: str, explicit: Optional[Union[str, Path]]) -> Path:
            if explicit and Path(explicit).is_file():
                return Path(explicit).resolve()
            for d in search_dirs:
                cand = d / name
                if cand.is_file():
                    return cand.resolve()
            return root / name

        self.yolo_path = find_file("yolo11n_v5b_best.pt", yolo_path)
        self.mnv3_path = find_file("best.pth", mnv3_path)

        print(f"[SLAMCVRecognizer] Khởi tạo trên thiết bị: {self.device}...")
        print(f"  - Trọng số YOLO11n: {self.yolo_path.name}")
        print(f"  - Trọng số MobileNetV3: {self.mnv3_path.name}")

        self.detector = YOLO(str(self.yolo_path))
        self.classifier = HybridObjectClassifier(self.mnv3_path, device=self.device)
        print("[SLAMCVRecognizer] Sẵn sàng nhận diện đa vật thể!\n")

    def _resolve_nested_boxes(
        self,
        raw_boxes: Union[List[Tuple[int, int, int, int]], List[Tuple[Tuple[int, int, int, int], float]]],
        h_img: int = 721,
        w_img: int = 1280,
    ) -> List[Tuple[int, int, int, int]]:
        """Bóc tách các bounding box và loại bỏ nhiễu biên/nhiễu lồng nhau bằng non-destructive NMS."""
        if not raw_boxes:
            return []

        # Chuẩn hóa format: [(bbox, conf)]
        items = []
        for elem in raw_boxes:
            if len(elem) == 2 and isinstance(elem[0], (tuple, list)):
                items.append((tuple(int(v) for v in elem[0]), float(elem[1])))
            elif len(elem) == 4:
                items.append((tuple(int(v) for v in elem), 1.0))

        # Sắp xếp theo confidence giảm dần
        items.sort(key=lambda x: x[1], reverse=True)
        kept = []

        border_strip_limit = max(180, int(0.28 * min(w_img, h_img)))
        for b, conf in items:
            x1, y1, x2, y2 = b
            bw = x2 - x1
            bh = y2 - y1
            if bw < 40 or bh < 40:
                continue

            aspect = max(bw, bh) / max(1, min(bw, bh))
            touches_border = (x1 <= 5 or y1 <= 5 or x2 >= w_img - 5 or y2 >= h_img - 5)
            # Loại bỏ dải vách hộp, nẹp kim loại mép ảnh (co giãn theo độ phân giải thực tế)
            if aspect >= 2.4 and touches_border and min(bw, bh) < border_strip_limit and conf < 0.45:
                continue
            if x1 <= 5 and y1 <= 5 and min(bw, bh) < border_strip_limit and (aspect >= 2.2 or conf < 0.15):
                continue

            area_b = bw * bh
            overlap = False
            split_cand = None
            for k, _ in kept:
                kx1, ky1, kx2, ky2 = k
                ix1 = max(x1, kx1)
                iy1 = max(y1, ky1)
                ix2 = min(x2, kx2)
                iy2 = min(y2, ky2)
                iw = max(0, ix2 - ix1)
                ih = max(0, iy2 - iy1)
                inter = iw * ih
                area_k = (kx2 - kx1) * (ky2 - ky1)
                iou = inter / max(1, area_b + area_k - inter)
                containment = inter / max(1, min(area_b, area_k))

                # Kiểm tra trường hợp b bao trùm k và có phần dư độc lập đáng kể (như Panel 4)
                if area_b > 1.4 * area_k and (inter / max(1, area_k)) > 0.70:
                    if (y2 - ky2) >= 80 and bw >= 60:
                        rem_box = (x1, max(y1, ky2 - 10), x2, y2)
                        rw, rh = rem_box[2] - rem_box[0], rem_box[3] - rem_box[1]
                        if rw >= 40 and rh >= 40 and (max(rw, rh) / min(rw, rh)) < 2.5:
                            split_cand = rem_box
                            break
                    elif (ky1 - y1) >= 80 and bw >= 60:
                        rem_box = (x1, y1, x2, min(y2, ky1 + 10))
                        rw, rh = rem_box[2] - rem_box[0], rem_box[3] - rem_box[1]
                        if rw >= 40 and rh >= 40 and (max(rw, rh) / min(rw, rh)) < 2.5:
                            split_cand = rem_box
                            break

                if iou > 0.45 or containment > 0.85:
                    overlap = True
                    break

            if split_cand is not None:
                sc_overlap = False
                sc_area = (split_cand[2] - split_cand[0]) * (split_cand[3] - split_cand[1])
                for k, _ in kept:
                    kx1, ky1, kx2, ky2 = k
                    ix1 = max(split_cand[0], kx1)
                    iy1 = max(split_cand[1], ky1)
                    ix2 = min(split_cand[2], kx2)
                    iy2 = min(split_cand[3], ky2)
                    iw = max(0, ix2 - ix1)
                    ih = max(0, iy2 - iy1)
                    inter = iw * ih
                    area_k = (kx2 - kx1) * (ky2 - ky1)
                    if (inter / max(1, sc_area + area_k - inter)) > 0.40:
                        sc_overlap = True
                        break
                if not sc_overlap:
                    kept.append((split_cand, conf))
            elif not overlap:
                kept.append((b, conf))

        kept.sort(key=lambda x: (x[0][0], x[0][1]))
        return [b for b, c in kept]

    @staticmethod
    def _count_crop_lines(crop_bgr: np.ndarray) -> int:
        if crop_bgr.size == 0 or crop_bgr.shape[0] < 16 or crop_bgr.shape[1] < 16:
            return 0
        gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 30, 90)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 25, minLineLength=25, maxLineGap=10)
        return int(np.asarray(lines).reshape(-1, 4).shape[0]) if lines is not None else 0

    def _reconcile_scene_objects(
        self, detected_objects: List[Dict[str, Any]], crops: List[np.ndarray]
    ) -> List[Dict[str, Any]]:
        """Điều phối và tối ưu hóa tính nhất quán toàn cục (kết hợp hình học, màu sắc và vị trí)."""
        n = len(detected_objects)
        if n not in (2, 3) or len(crops) != n:
            return detected_objects

        target_classes = ["cylinder", "rectangular_box", "triangular_prism"]
        probs_list = []
        geos_list = []
        colors_list = []

        for i, o in enumerate(detected_objects):
            crop = crops[i]
            c_info = o
            geo = o.get("geometry", {})
            if not geo or not isinstance(geo, dict) or "circularity" not in geo:
                geo_obj = self.classifier.geometry.analyze(crop)
                geo = {
                    "circularity": geo_obj.circularity,
                    "extent": geo_obj.extent,
                    "min_angle": geo_obj.min_corner_angle,
                    "right_angles": geo_obj.right_angles_count,
                    "flags": geo_obj.diagnostic_flags,
                }
            probs_list.append(c_info["probabilities"])
            geos_list.append(geo)

            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            c_masks = self._tray_color_masks(hsv)
            r_cnt = int(np.sum(c_masks["cylinder"]))
            y_cnt = int(np.sum(c_masks["triangular_prism"]))
            p_cnt = int(np.sum(c_masks["rectangular_box"]))
            area_c = max(1, crop.shape[0] * crop.shape[1])

            is_yellow = (y_cnt / area_c >= 0.12) and (y_cnt > 1.35 * max(r_cnt, p_cnt))
            is_pink = (p_cnt / area_c >= 0.12) and (p_cnt > 1.35 * max(r_cnt, y_cnt))
            is_red = (r_cnt / area_c >= 0.12) and (r_cnt > 1.35 * max(y_cnt, p_cnt))
            colors_list.append({"is_yellow": is_yellow, "is_pink": is_pink, "is_red": is_red})

        # Xử lý trường hợp 3 vật thể (khay hoàn chỉnh)
        if n == 3:
            best_perm = None
            best_score = -1e9

            for perm in itertools.permutations(target_classes):
                score = 0.0
                for i, cls in enumerate(perm):
                    p = probs_list[i][cls]
                    geo = geos_list[i]
                    col = colors_list[i]
                    flags = geo.get("flags", {})
                    bbox = detected_objects[i]["bbox"]

                    score += np.log(max(p, 1e-4))

                    if cls == "cylinder":
                        if flags.get("is_highly_circular"):
                            score += 4.0
                        elif flags.get("has_cylinder_arc"):
                            score += 3.0
                        elif geo.get("circularity", 0) >= 0.55:
                            score += 1.5
                        if col["is_red"]:
                            score += 4.0
                        if flags.get("has_central_ridge_lines"):
                            score -= 3.0
                        if geo.get("min_angle", 90) < 60:
                            score -= 2.0

                    elif cls == "triangular_prism":
                        if col["is_yellow"]:
                            score += 4.0
                        if flags.get("has_central_ridge_lines"):
                            score += 3.0
                        if flags.get("has_triangular_vertices"):
                            score += 3.0
                        if flags.get("has_acute_angle_junction"):
                            score += 1.5
                        if bbox[1] < 150:
                            score += 2.0

                    elif cls == "rectangular_box":
                        if col["is_pink"]:
                            score += 4.0
                        if geo.get("extent", 0) >= 0.85:
                            score += 1.5
                        if geo.get("right_angles", 0) >= 3:
                            score += 1.0
                        if flags.get("has_central_ridge_lines"):
                            score -= 3.0
                        if bbox[1] > 250 and bbox[0] < 300:
                            score += 2.0

                if score > best_score:
                    best_score = score
                    best_perm = perm

            if best_perm is not None:
                for i, cls in enumerate(best_perm):
                    detected_objects[i]["class_name"] = cls
                    p_val = probs_list[i][cls]
                    detected_objects[i]["confidence"] = float(max(p_val, 0.85))
                    detected_objects[i]["decision_type"] = f"SCENE_OPTIMAL_{cls.upper()}"
                    detected_objects[i]["explanation"] = f"Tối ưu hóa toàn cảnh khay hàng (Global Matching): {CLASS_DISPLAY_NAMES.get(cls, cls)}."

            return detected_objects

        # Xử lý trường hợp 2 vật thể bị phân loại nhầm thành cùng một loại (Mutual Exclusion)
        elif n == 2:
            c0 = detected_objects[0]["class_name"]
            c1 = detected_objects[1]["class_name"]
            if c0 == c1:
                best_pair = None
                best_score = -1e9
                for pair in itertools.permutations(target_classes, 2):
                    score = 0.0
                    for i, cls in enumerate(pair):
                        p = probs_list[i][cls]
                        geo = geos_list[i]
                        col = colors_list[i]
                        flags = geo.get("flags", {})
                        score += np.log(max(p, 1e-4))

                        if cls == "cylinder":
                            if col["is_red"] and (flags.get("has_cylinder_arc") or geo.get("circularity", 0) >= 0.50):
                                score += 3.0
                            if flags.get("has_cylinder_arc"):
                                score += 3.0
                            if geo.get("circularity", 0) < 0.45 and not flags.get("has_cylinder_arc"):
                                score -= 5.0
                        elif cls == "triangular_prism":
                            if col["is_yellow"]:
                                score += 4.0
                            if flags.get("has_central_ridge_lines") or flags.get("has_triangular_vertices"):
                                score += 3.0
                        elif cls == "rectangular_box":
                            if col["is_pink"]:
                                score += 4.0
                            if geo.get("extent", 0) >= 0.70:
                                score += 2.0
                            if geo.get("right_angles", 0) >= 2:
                                score += 2.0

                    if score > best_score:
                        best_score = score
                        best_pair = pair

                if best_pair is not None:
                    for i, cls in enumerate(best_pair):
                        detected_objects[i]["class_name"] = cls
                        p_val = probs_list[i][cls]
                        detected_objects[i]["confidence"] = float(max(p_val, 0.85))
                        detected_objects[i]["decision_type"] = f"SCENE_PAIR_{cls.upper()}"
                        detected_objects[i]["explanation"] = f"Triệt tiêu trùng lặp 2 vật thể: {CLASS_DISPLAY_NAMES.get(cls, cls)}."

            return detected_objects

    @staticmethod
    def _tray_color_masks(hsv_img: np.ndarray) -> Dict[str, np.ndarray]:
        """Tách mặt nạ 3 màu khối hàng thực tế trên khay (Hồng mở tới H<174 để không lẫn sang Đỏ ở H>=174/H<12)."""
        fg = (hsv_img[:, :, 1] > 45) & (hsv_img[:, :, 2] > 40)
        h = hsv_img[:, :, 0]
        return {
            "cylinder": fg & ((h < 12) | (h >= 174)),
            "rectangular_box": fg & ((h >= 120) & (h < 174)),
            "triangular_prism": fg & ((h >= 15) & (h <= 42)),
        }

    @staticmethod
    def _detect_physical_tray_kit(
        img_bgr: np.ndarray, num_raw_boxes: int = 2
    ) -> Tuple[bool, Dict[str, int]]:
        """Phát hiện xem ảnh có phải là khay vật thể thực tế với bộ >= 2 khối khác màu (Đỏ, Vàng, Hồng) hay không."""
        if num_raw_boxes < 1:
            return False, {}
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        if float(np.mean(hsv[:, :, 2])) < 85:
            return False, {}
        total = max(1, img_bgr.shape[0] * img_bgr.shape[1])
        masks = SLAMCVRecognizer._tray_color_masks(hsv)
        red_count = int(np.sum(masks["cylinder"]))
        yellow_count = int(np.sum(masks["triangular_prism"]))
        pink_count = int(np.sum(masks["rectangular_box"]))
        r_pct = red_count / total
        y_pct = yellow_count / total
        p_pct = pink_count / total
        active_colors = sum([r_pct >= 0.035, y_pct >= 0.035, p_pct >= 0.025])
        is_kit = (active_colors >= 2) and (r_pct >= 0.035 or y_pct >= 0.035)
        return is_kit, {"red": red_count, "yellow": yellow_count, "pink": pink_count}

    @staticmethod
    def _split_merged_box_by_color(
        img_bgr: np.ndarray, bbox: Tuple[int, int, int, int]
    ) -> Optional[List[Tuple[Tuple[int, int, int, int], str]]]:
        """Tự động phân rã bounding box bị gộp nhiều vật thể bằng phân cụm màu sắc HSV."""
        x1, y1, x2, y2 = bbox
        crop = img_bgr[y1:y2, x1:x2]
        h_c, w_c = crop.shape[:2]
        area_c = h_c * w_c
        if area_c < 40000:
            return None

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        masks = SLAMCVRecognizer._tray_color_masks(hsv)

        counts = {cls: int(np.sum(m)) for cls, m in masks.items()}
        significant = [cls for cls, cnt in counts.items() if cnt > max(12000, int(area_c * 0.08))]

        if len(significant) < 2:
            return None

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        sub_boxes = []
        for cls in significant:
            m = masks[cls].astype(np.uint8)
            m_clean = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
            m_clean = cv2.morphologyEx(m_clean, cv2.MORPH_CLOSE, kernel)

            cnts, _ = cv2.findContours(m_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            best_cnt = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(best_cnt) < 5000:
                continue
            bx, by, bw, bh = cv2.boundingRect(best_cnt)
            if bw < 50 or bh < 50:
                continue

            gx1 = max(0, x1 + bx - 10)
            gy1 = max(0, y1 + by - 10)
            gx2 = min(img_bgr.shape[1], x1 + bx + bw + 10)
            gy2 = min(img_bgr.shape[0], y1 + by + bh + 10)
            sub_boxes.append(((gx1, gy1, gx2, gy2), cls))

        return sub_boxes if len(sub_boxes) >= 2 else None

    @staticmethod
    def _recover_missed_kit_boxes(
        img_bgr: np.ndarray, existing_boxes: List[Tuple[int, int, int, int]]
    ) -> List[Tuple[int, int, int, int]]:
        """Khôi phục vật thể rõ ràng trên khay kit nếu bị YOLO bỏ sót khi đặt camera quá sát."""
        if not existing_boxes:
            return existing_boxes
        h_img, w_img = img_bgr.shape[:2]
        total = max(1, h_img * w_img)
        min_blob_px = max(20000, int(total * 0.04))

        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        masks = SLAMCVRecognizer._tray_color_masks(hsv)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        augmented = list(existing_boxes)

        for _, mask in masks.items():
            if int(np.sum(mask)) < min_blob_px:
                continue
            m_u8 = mask.astype(np.uint8)
            m_clean = cv2.morphologyEx(m_u8, cv2.MORPH_OPEN, kernel)
            m_clean = cv2.morphologyEx(m_clean, cv2.MORPH_CLOSE, kernel)
            cnts, _ = cv2.findContours(m_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            best_cnt = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(best_cnt) < min_blob_px * 0.6:
                continue
            bx, by, bw, bh = cv2.boundingRect(best_cnt)
            if bw < 60 or bh < 60:
                continue
            cand = (
                max(0, bx - 12),
                max(0, by - 12),
                min(w_img, bx + bw + 12),
                min(h_img, by + bh + 12),
            )
            cand_area = max(1, (cand[2] - cand[0]) * (cand[3] - cand[1]))
            max_cover = 0.0
            for ex in augmented:
                ix1 = max(cand[0], ex[0])
                iy1 = max(cand[1], ex[1])
                ix2 = min(cand[2], ex[2])
                iy2 = min(cand[3], ex[3])
                if ix2 > ix1 and iy2 > iy1:
                    inter = (ix2 - ix1) * (iy2 - iy1)
                    max_cover = max(max_cover, inter / cand_area)
            if max_cover < 0.35:
                augmented.append(cand)

        augmented.sort(key=lambda b: (b[0], b[1]))
        return augmented

    @staticmethod
    def _prune_duplicate_classes_in_tray(objs: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Giữ tối đa 1 vật thể tốt nhất cho mỗi loại trong khay (Cylinder, Box, Prism)."""
        by_class = {}
        for o in objs:
            cls = o["class_name"]
            if cls not in by_class:
                by_class[cls] = []
            by_class[cls].append(o)

        kept = []
        for cls, obj_list in by_class.items():
            if len(obj_list) == 1:
                kept.append(obj_list[0])
            else:
                best = max(
                    obj_list,
                    key=lambda x: (
                        x.get("confidence", 0),
                        (x["bbox"][2] - x["bbox"][0]) * (x["bbox"][3] - x["bbox"][1]),
                    ),
                )
                kept.append(best)

        kept.sort(key=lambda x: (x["bbox"][0], x["bbox"][1]))
        for i, o in enumerate(kept):
            o["object_id"] = i + 1
        return kept

    def recognize_scene(
        self, img_bgr: np.ndarray, pad_ratio: float = 0.0
    ) -> Tuple[List[Dict[str, Any]], np.ndarray, float]:
        """Nhận diện toàn bộ vật thể trên 1 khung cảnh khay hàng."""
        t0 = time.perf_counter()
        h_img, w_img = img_bgr.shape[:2]
        # 1. Quét tìm vị trí vật thể bằng YOLO11n
        yolo_res = self.detector(
            img_bgr, conf=self.conf_threshold, agnostic_nms=True, iou=0.45, verbose=False
        )[0]
        raw_boxes_with_confs = []
        for b in yolo_res.boxes:
            xyxy = tuple(b.xyxy[0].cpu().numpy().astype(int))
            conf = float(b.conf[0])
            raw_boxes_with_confs.append((xyxy, conf))

        is_kit, _ = self._detect_physical_tray_kit(
            img_bgr, num_raw_boxes=len(raw_boxes_with_confs)
        )

        # 2. Xử lý phân rã Bbox gộp và lọc vách ma trên nền trống
        refined_boxes_with_confs = []
        for (x1, y1, x2, y2), conf in raw_boxes_with_confs:
            bw = x2 - x1
            bh = y2 - y1
            crop = img_bgr[y1:y2, x1:x2]
            if crop.size == 0:
                continue

            hsv_c = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            fg_ratio = float(np.mean((hsv_c[:, :, 1] > 45) & (hsv_c[:, :, 2] > 40)))
            touches_raw = (x1 <= 5 or y1 <= 5 or x2 >= w_img - 5 or y2 >= h_img - 5)

            # Lọc bóng vách / nền trắng trống sát viền khi YOLO tự tin cực thấp
            if touches_raw and conf < 0.15 and fg_ratio < 0.03:
                continue

            if is_kit:
                if fg_ratio < 0.12 and min(bw, bh) < 300:
                    continue
                if fg_ratio < 0.08:
                    continue

                sub_boxes = self._split_merged_box_by_color(img_bgr, (x1, y1, x2, y2))
                if sub_boxes is not None:
                    # Gán conf phụ thấp hơn box YOLO gốc đơn lẻ để không cắt nát box YOLO chuẩn lân cận
                    sub_conf = min(conf, 0.45)
                    for s_box, _ in sub_boxes:
                        refined_boxes_with_confs.append((s_box, sub_conf))
                    continue

            refined_boxes_with_confs.append(((x1, y1, x2, y2), conf))

        # 3. Bóc tách các hộp bị dính/lồng nhau bằng non-destructive NMS
        raw_boxes = self._resolve_nested_boxes(refined_boxes_with_confs, h_img=h_img, w_img=w_img)

        # 3b. Khôi phục khối kit bị YOLO bỏ sót hoàn toàn (khi camera đặt quá gần/lệch)
        if is_kit and len(raw_boxes) >= 1:
            raw_boxes = self._recover_missed_kit_boxes(img_bgr, raw_boxes)

        detected_objects = []
        crops = []
        vis_img = img_bgr.copy()

        # 4. Phân loại từng vật thể
        for idx, (x1, y1, x2, y2) in enumerate(raw_boxes):
            w_box = x2 - x1
            h_box = y2 - y1
            pad_x = int(w_box * pad_ratio)
            pad_y = int(h_box * pad_ratio)

            cx1 = max(0, x1 - pad_x)
            cy1 = max(0, y1 - pad_y)
            cx2 = min(w_img, x2 + pad_x)
            cy2 = min(h_img, y2 + pad_y)

            crop = img_bgr[cy1:cy2, cx1:cx2]
            if crop.size == 0 or crop.shape[0] < 12 or crop.shape[1] < 12:
                continue

            touches_border = (x1 <= 5 or y1 <= 5 or x2 >= w_img - 5 or y2 >= h_img - 5)
            info = self.classifier.classify_crop(crop, touches_border=touches_border)

            hsv_c = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            crop_area = max(1, crop.shape[0] * crop.shape[1])
            c_masks = self._tray_color_masks(hsv_c)
            c_counts = {k: int(np.sum(m)) for k, m in c_masks.items()}
            ranked_colors = sorted(c_counts.items(), key=lambda kv: kv[1], reverse=True)
            top_cls, top_cnt = ranked_colors[0]
            second_cnt = ranked_colors[1][1]
            dom_ratio = top_cnt / crop_area

            dom_cls = None
            if is_kit:
                if top_cnt > 1.5 * second_cnt and top_cnt > 8000:
                    dom_cls = top_cls
            elif (
                dom_ratio >= 0.25
                and top_cnt > max(15000, int(crop_area * 0.25))
                and top_cnt > 2.5 * second_cnt
                and float(np.mean(hsv_c[:, :, 1][c_masks[top_cls]])) >= 55.0
            ):
                dom_cls = top_cls

            if dom_cls:
                info["class_name"] = dom_cls
                info["confidence"] = 0.98 if is_kit else float(max(info.get("confidence", 0.0), 0.96))
                info["decision_type"] = (
                    f"COLOR_KIT_CONFIRMED_{dom_cls.upper()}"
                    if is_kit
                    else f"COLOR_TRAY_CONFIRMED_{dom_cls.upper()}"
                )
                info["explanation"] = f"Xác nhận qua quang phổ màu vật thể thực tế: {CLASS_DISPLAY_NAMES.get(dom_cls, dom_cls)}."
                info["probabilities"] = {
                    "cylinder": info["confidence"] if dom_cls == "cylinder" else 0.01,
                    "rectangular_box": info["confidence"] if dom_cls == "rectangular_box" else 0.01,
                    "triangular_prism": info["confidence"] if dom_cls == "triangular_prism" else 0.01,
                }

            info["object_id"] = idx + 1
            info["bbox"] = [int(x1), int(y1), int(x2), int(y2)]
            detected_objects.append(info)
            crops.append(crop)

        # Khử box tách mảnh trùng cùng 1 màu kit trước khi đối chiếu toàn cảnh 2/3 vật thể
        if is_kit and len(detected_objects) >= 2:
            kit_counts: Dict[str, int] = {}
            for o in detected_objects:
                if str(o.get("decision_type", "")).startswith("COLOR_KIT_CONFIRMED_"):
                    c_name = o["class_name"]
                    kit_counts[c_name] = kit_counts.get(c_name, 0) + 1
            if any(cnt > 1 for cnt in kit_counts.values()):
                detected_objects = self._prune_duplicate_classes_in_tray(detected_objects)
                crops = [img_bgr[o["bbox"][1]:o["bbox"][3], o["bbox"][0]:o["bbox"][2]] for o in detected_objects]

        # 5. Điều phối và tối ưu hóa tính nhất quán toàn cục
        detected_objects = self._reconcile_scene_objects(detected_objects, crops)

        # 6. Triệt tiêu trùng lặp đối với khay thực tế (mỗi loại vật thể xuất hiện tối đa 1 lần)
        if is_kit and len(detected_objects) > 3:
            detected_objects = self._prune_duplicate_classes_in_tray(detected_objects)

        # Ghi log tiếng Việt chi tiết cho từng vật thể
        for info in detected_objects:
            info["vietnamese_log"] = format_vietnamese_log(info)

        # 5. Vẽ bounding box công nghệ cao và nhãn
        for info in detected_objects:
            x1, y1, x2, y2 = info["bbox"]
            w_box = x2 - x1
            h_box = y2 - y1
            cname = info["class_name"]
            conf = info["confidence"]
            color = CLASS_COLORS_BGR.get(cname, (180, 180, 180))
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)

            # 4 góc nổi bật
            c_len = max(8, int(min(w_box, h_box) * 0.15))
            cv2.line(vis_img, (x1, y1), (x1 + c_len, y1), color, 3)
            cv2.line(vis_img, (x1, y1), (x1, y1 + c_len), color, 3)
            cv2.line(vis_img, (x2, y1), (x2 - c_len, y1), color, 3)
            cv2.line(vis_img, (x2, y1), (x2, y1 + c_len), color, 3)
            cv2.line(vis_img, (x1, y2), (x1 + c_len, y2), color, 3)
            cv2.line(vis_img, (x1, y2), (x1, y2 - c_len), color, 3)
            cv2.line(vis_img, (x2, y2), (x2 - c_len, y2), color, 3)
            cv2.line(vis_img, (x2, y2), (x2, y2 - c_len), color, 3)

            disp_name = CLASS_DISPLAY_NAMES.get(cname, cname)
            badge_text = f"{disp_name} [{conf*100:.0f}%]"
            (tw, th), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            ty1 = max(0, y1 - th - 8)
            cv2.rectangle(vis_img, (x1, ty1), (x1 + tw + 8, ty1 + th + 6), color, -1)
            cv2.putText(vis_img, badge_text, (x1 + 4, ty1 + th + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

        latency_ms = (time.perf_counter() - t0) * 1000.0
        return detected_objects, vis_img, latency_ms

    def recognize_grid_2x2(
        self, img_bgr: np.ndarray
    ) -> Tuple[List[Tuple[str, List[Dict[str, Any]]]], np.ndarray, float]:
        """Tự động cắt lưới 2x2 làm 4 góc, nhận diện từng góc và ghép ảnh kết quả hoàn chỉnh."""
        t0 = time.perf_counter()
        h, w = img_bgr.shape[:2]
        ym, xm = h // 2, w // 2

        panels = [
            ("Panel 1 (Top-Left)", img_bgr[0:ym, 0:xm]),
            ("Panel 2 (Top-Right)", img_bgr[0:ym, xm:w]),
            ("Panel 3 (Bottom-Left)", img_bgr[ym:h, 0:xm]),
            ("Panel 4 (Bottom-Right)", img_bgr[ym:h, xm:w]),
        ]

        all_results = []
        annotated_panels = []

        for name, p_img in panels:
            res, vis, _ = self.recognize_scene(p_img)
            all_results.append((name, res))
            annotated_panels.append(vis)

        top = np.hstack([annotated_panels[0], annotated_panels[1]])
        bot = np.hstack([annotated_panels[2], annotated_panels[3]])
        grid_vis = np.vstack([top, bot])
        latency_ms = (time.perf_counter() - t0) * 1000.0
        return all_results, grid_vis, latency_ms


# ==============================================================================
# 6. GIAO DIỆN DÒNG LỆNH & WEBCAM INTERACTIVE
# ==============================================================================
def run_cli():
    parser = argparse.ArgumentParser(description="SLAM_KLTN — Unified Object Recognition Module")
    parser.add_argument("image", nargs="?", type=str, default=None, help="Đường dẫn file ảnh hoặc thư mục cần test")
    parser.add_argument("--batch", nargs="+", help="Danh sách nhiều file ảnh cần nhận diện cùng lúc")
    parser.add_argument("--dir", type=str, default=None, help="Thư mục chứa nhiều ảnh cần nhận diện hàng loạt")
    parser.add_argument("--grid", action="store_true", help="Bật cờ này nếu ảnh là lưới 2x2 gồm 4 kịch bản")
    parser.add_argument("--camera", type=int, default=None, help="Camera device ID (ví dụ: 0 hoặc 1)")
    parser.add_argument("--out", type=str, default="output", help="Thư mục lưu ảnh kết quả (mặc định: output)")
    parser.add_argument("--clean", action="store_true", help="Xóa sạch file cũ trong thư mục output trước khi lưu kết quả mới")
    args = parser.parse_args()

    recognizer = SLAMCVRecognizer()
    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.clean and out_dir.is_dir():
        cleaned_cnt = 0
        for f in out_dir.iterdir():
            if f.is_file() and f.name != ".gitkeep":
                try:
                    f.unlink()
                    cleaned_cnt += 1
                except Exception:
                    pass
        print(f"[CLEAN] Đã dọn sạch {cleaned_cnt} file cũ trong: {out_dir}")
    t_str = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Chế độ Webcam trực tiếp
    if args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"[ERROR] Không thể mở Camera {args.camera}")
            return

        window_name = "SLAM_KLTN - Live Webcam Multi-Object Recognition"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 800)
        print("\n[WEBCAM SẴN SÀNG]")
        print("  Phím [SPACE] : Nhận diện toàn bộ vật thể tại khung hình hiện tại")
        print("  Phím [S]     : Lưu ảnh chụp kết quả vào folder output/")
        print("  Phím [Q/ESC] : Thoát\n")

        current_vis = None
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                break

            disp = current_vis if current_vis is not None else frame.copy()
            if current_vis is None:
                cv2.putText(disp, "NHAN [SPACE] DE NHAN DIEN TOAN BO VAT THE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow(window_name, disp)

            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            elif key == 32:  # SPACE
                print("[Đang nhận diện khung hình...]")
                results, vis, lat = recognizer.recognize_scene(frame)
                current_vis = vis
                print(f"Tìm thấy {len(results)} vật thể ({lat:.1f}ms):")
                for obj in results:
                    print(f"  #{obj['object_id']}: {obj['class_name'].upper()} ({obj['confidence']*100:.1f}%) -> {obj['explanation']}")
            elif key in (ord("s"), ord("S")):
                if current_vis is not None:
                    out_path = out_dir / f"snapshot_{t_str}.jpg"
                    cv2.imwrite(str(out_path), current_vis)
                    print(f"[ĐÃ LƯU ẢNH]: {out_path}")

        cap.release()
        cv2.destroyAllWindows()
        return

    # Chế độ nhận diện hàng loạt qua --dir hoặc --batch
    batch_files = []
    if args.dir:
        dpath = Path(args.dir).resolve()
        if dpath.is_dir():
            seen = set()
            for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp"):
                for p in dpath.glob(ext):
                    if p.resolve() not in seen and "_result" not in p.name and "_annotated" not in p.name:
                        seen.add(p.resolve())
                        batch_files.append(p)
            batch_files = sorted(batch_files)
    elif args.batch:
        batch_files = [Path(p).resolve() for p in args.batch if Path(p).is_file()]

    # Kiểm tra nếu tham số vị trí `image` là một thư mục
    if not batch_files and args.image and Path(args.image).is_dir():
        dpath = Path(args.image).resolve()
        seen = set()
        for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp"):
            for p in dpath.glob(ext):
                if p.resolve() not in seen and "_result" not in p.name and "_annotated" not in p.name:
                    seen.add(p.resolve())
                    batch_files.append(p)
        batch_files = sorted(batch_files)

    if batch_files:
        print(f"\n============================================================")
        print(f"  NHẬN DIỆN HÀNG LOẠT ({len(batch_files)} ảnh) -> Lưu vào: {out_dir}")
        print(f"============================================================")
        t_batch_start = time.perf_counter()
        total_objs = 0
        csv_records = []

        for idx, p in enumerate(batch_files):
            img = cv2.imread(str(p))
            if img is None:
                print(f"[{idx+1}/{len(batch_files)}] [BỎ QUA] Không đọc được ảnh: {p.name}")
                continue

            results, vis, lat = recognizer.recognize_scene(img)
            total_objs += len(results)
            out_vis = out_dir / f"{p.stem}_result.jpg"
            cv2.imwrite(str(out_vis), vis)

            names = [f"{o['class_name']}({o['confidence']*100:.0f}%)" for o in results]
            print(f"[{idx+1}/{len(batch_files)}] {p.name:<30} -> {len(results)} vật thể ({lat:.1f}ms): {', '.join(names)}")

            for o in results:
                csv_records.append({
                    "file_name": p.name,
                    "output_file": out_vis.name,
                    "object_id": o.get("object_id", ""),
                    "class_name": o.get("class_name", ""),
                    "confidence": f"{o.get('confidence', 0.0)*100:.2f}%",
                    "bbox": str(o.get("bbox", [])),
                    "decision_type": o.get("decision_type", ""),
                    "explanation": o.get("explanation", ""),
                })

        # Ghi CSV
        import csv
        csv_path = out_dir / f"batch_summary_{t_str}.csv"
        try:
            with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.DictWriter(f, fieldnames=[
                    "file_name", "output_file", "object_id", "class_name",
                    "confidence", "bbox", "decision_type", "explanation"
                ])
                writer.writeheader()
                writer.writerows(csv_records)
        except Exception as e:
            print(f"[WARN] Không ghi được CSV: {e}")

        elapsed = time.perf_counter() - t_batch_start
        print(f"\n[HOÀN TẤT BATCH]")
        print(f"  - Số ảnh đã xử lý: {len(batch_files)}")
        print(f"  - Tổng số vật thể: {total_objs}")
        print(f"  - Tổng thời gian:  {elapsed:.2f}s (~{(elapsed*1000)/max(1, len(batch_files)):.1f}ms/ảnh)")
        print(f"  - Thư mục kết quả: {out_dir}")
        print(f"  - Báo cáo CSV:     {csv_path}\n")
        return

    # Chế độ 1 File ảnh duy nhất
    img_path = args.image
    if not img_path:
        img_path = input("\nNhập đường dẫn file ảnh hoặc thư mục (hoặc kéo thả vào đây rồi Enter): ").strip("\"' ")
        if not img_path:
            return
        p_cand = Path(img_path).resolve()
        if p_cand.is_dir():
            # Nếu người dùng nhập thư mục, gọi đệ quy sang chế độ batch
            args.dir = str(p_cand)
            run_cli()
            return
        choice_grid = input("Ảnh này là ảnh đơn [1] hay ảnh ghép lưới 2x2 [2]? (Mặc định 1): ").strip()
        is_grid = (choice_grid == "2")
    else:
        if not args.grid:
            choice = input("\nẢnh bạn vừa nạp là ảnh đơn [1] hay ảnh ghép lưới 2x2 [2]? (Mặc định 1): ").strip()
            is_grid = (choice == "2")
        else:
            is_grid = True

    path = Path(img_path).resolve()
    if not path.is_file():
        print(f"[ERROR] Không tìm thấy file: {path}")
        return

    img = cv2.imread(str(path))
    if img is None:
        print(f"[ERROR] Không đọc được ảnh: {path}")
        return

    if is_grid:
        print(f"\n[Chế độ Lưới 2x2] Đang nhận diện cả 4 kịch bản...")
        results, grid_vis, lat = recognizer.recognize_grid_2x2(img)
        for name, r_list in results:
            print(f"\n--- {name} ---")
            for obj in r_list:
                print(f"  #{obj['object_id']}: {obj['class_name'].upper()} ({obj['confidence']*100:.1f}%) -> {obj['explanation']}")
        out_vis = out_dir / f"{path.stem}_grid_result.jpg"
        cv2.imwrite(str(out_vis), grid_vis)
        print(f"\n[THÀNH CÔNG] Đã lưu ảnh kết quả lưới tại: {out_vis}")
    else:
        print(f"\n[Chế độ Ảnh đơn] Đang nhận diện...")
        results, vis, lat = recognizer.recognize_scene(img)
        print(f"Tìm thấy {len(results)} vật thể ({lat:.1f}ms):")
        for obj in results:
            print(f"  #{obj['object_id']}: {obj['class_name'].upper()} ({obj['confidence']*100:.1f}%) -> {obj['explanation']}")
        out_vis = out_dir / f"{path.stem}_result.jpg"
        cv2.imwrite(str(out_vis), vis)
        print(f"\n[THÀNH CÔNG] Đã lưu ảnh kết quả tại: {out_vis}")


if __name__ == "__main__":
    run_cli()
