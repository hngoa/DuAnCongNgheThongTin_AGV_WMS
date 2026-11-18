"""SLAM_KLTN — OpenCV Geometric Analyzer for Tray Objects.

Extracts rich 2D/3D-projected geometric features from images or ROI crops:
- Foreground segmentation against matte black tray (Otsu adaptive threshold)
- Contour shape descriptors (area, perimeter, circularity, solidity, extent, aspect ratio)
- Multi-scale polygon approximation (vertex counts, corner angles, acute angle detection)
- Edge and Line segment detection (Auto-Canny, HoughLinesP, line orientations, orthogonality)
- Candidate shape indicators (circular arcs, non-orthogonal slanted lines, rectilinear grids)
- Diagnostic flags and Geometry Evidence Quality scoring (HIGH, MEDIUM, LOW, NO_SIGNAL)
- Clean visual overlay for explainability and debugging.

Note: Strictly avoids naive 2D vertex counting as a direct 3D classifier.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
from PIL import Image, ImageOps


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
    # Status and quality
    evidence_quality: str  # HIGH, MEDIUM, LOW, NO_SIGNAL
    diagnostic_flags: Dict[str, bool] = field(default_factory=dict)
    
    # Contour features
    contour_area: float = 0.0
    perimeter: float = 0.0
    circularity: float = 0.0
    solidity: float = 0.0
    extent: float = 0.0
    aspect_ratio: float = 0.0
    
    # Polygon approximation
    poly_vertices_002: int = 0
    poly_vertices_004: int = 0
    poly_vertices_006: int = 0
    min_corner_angle: float = 180.0
    acute_angles_count: int = 0
    right_angles_count: int = 0
    
    # Line segments
    total_lines: int = 0
    orthogonal_lines_count: int = 0
    slanted_lines_count: int = 0
    non_orthogonal_length_ratio: float = 0.0
    detected_lines: List[LineSegment] = field(default_factory=list)
    
    # Candidate votes from pure geometry
    geometry_candidate: str = "UNKNOWN"  # cylinder, rectangular_box, triangular_prism, UNKNOWN
    candidate_scores: Dict[str, float] = field(default_factory=dict)
    
    # Geometry details
    crop_size: Tuple[int, int] = (0, 0)
    raw_contour: Optional[np.ndarray] = None


class GeometricAnalyzer:
    """Analyzes geometric properties of objects on the SLAM_KLTN black tray."""

    def __init__(
        self,
        target_size: int = 256,
        canny_sigma: float = 0.33,
        ortho_tolerance_deg: float = 12.0,
    ):
        self.target_size = target_size
        self.canny_sigma = canny_sigma
        self.ortho_tolerance_deg = ortho_tolerance_deg

    def analyze(
        self,
        image_input: Union[str, Path, Image.Image, np.ndarray],
        bbox: Optional[Tuple[int, int, int, int]] = None,
        pad_ratio: float = 0.14,
    ) -> GeometryAnalysisResult:
        """Extract geometric features from an image and optional bounding box."""
        # 1. Load image as RGB
        if isinstance(image_input, (str, Path)):
            p = Path(image_input)
            if not p.is_file():
                return GeometryAnalysisResult(evidence_quality="NO_SIGNAL", diagnostic_flags={"file_not_found": True})
            with Image.open(p) as pil_img:
                pil_img = ImageOps.exif_transpose(pil_img)
                rgb_img = np.array(pil_img.convert("RGB"))
        elif isinstance(image_input, Image.Image):
            pil_img = ImageOps.exif_transpose(image_input)
            rgb_img = np.array(pil_img.convert("RGB"))
        elif isinstance(image_input, np.ndarray):
            if image_input.ndim == 2:
                rgb_img = cv2.cvtColor(image_input, cv2.COLOR_GRAY2RGB)
            elif image_input.shape[2] == 4:
                rgb_img = cv2.cvtColor(image_input, cv2.COLOR_RGBA2RGB)
            else:
                rgb_img = image_input.copy()
        else:
            return GeometryAnalysisResult(evidence_quality="NO_SIGNAL", diagnostic_flags={"invalid_input": True})

        h_full, w_full = rgb_img.shape[:2]

        # 2. Extract ROI crop
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
            return GeometryAnalysisResult(
                evidence_quality="NO_SIGNAL",
                crop_size=(cw, ch),
                diagnostic_flags={"crop_too_small": True}
            )

        # 3. Standardize scale for consistent geometry thresholds
        scale = float(self.target_size) / float(max(ch, cw))
        nw = max(1, int(round(cw * scale)))
        nh = max(1, int(round(ch * scale)))
        crop_std = cv2.resize(crop_rgb, (nw, nh), interpolation=cv2.INTER_AREA)

        # Estimate background brightness from the border pixels of crop_std
        border_pixels = np.concatenate([
            crop_std[0, :, :], crop_std[-1, :, :],
            crop_std[:, 0, :], crop_std[:, -1, :]
        ], axis=0)
        bg_color = np.median(border_pixels, axis=0).astype(int)
        bg_gray = float(0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2])
        is_bright_bg = (bg_gray > 120.0)

        # Pad with matched background border color (neutral black on dark tray, paper color on light tray)
        pad_val = tuple(int(x) for x in bg_color) if is_bright_bg else (0, 0, 0)
        pad_top = (self.target_size - nh) // 2
        pad_bottom = self.target_size - nh - pad_top
        pad_left = (self.target_size - nw) // 2
        pad_right = self.target_size - nw - pad_left
        crop_canvas = cv2.copyMakeBorder(
            crop_std, pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT, value=pad_val
        )

        gray = cv2.cvtColor(crop_canvas, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 4. Foreground Segmentation (Adaptive against Black Tray or Bright White Surface)
        thresh_type = (cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU) if is_bright_bg else (cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, thresh = cv2.threshold(blurred, 0, 255, thresh_type)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh_clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts, _ = cv2.findContours(thresh_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return GeometryAnalysisResult(
                evidence_quality="NO_SIGNAL",
                crop_size=(cw, ch),
                diagnostic_flags={"no_contours_found": True}
            )

        # Filter out tiny noise contours, take largest
        main_cnt = max(cnts, key=cv2.contourArea)
        area = float(cv2.contourArea(main_cnt))
        peri = float(cv2.arcLength(main_cnt, True))

        if area < 400.0 or peri < 50.0:
            return GeometryAnalysisResult(
                evidence_quality="LOW",
                crop_size=(cw, ch),
                contour_area=area,
                perimeter=peri,
                diagnostic_flags={"area_too_small": True}
            )

        # 5. Basic Shape Descriptors
        circularity = float((4.0 * np.pi * area) / (peri * peri)) if peri > 0 else 0.0
        rx, ry, rw, rh = cv2.boundingRect(main_cnt)
        extent = float(area / (rw * rh)) if rw * rh > 0 else 0.0
        hull = cv2.convexHull(main_cnt)
        hull_area = float(cv2.contourArea(hull))
        solidity = float(area / hull_area) if hull_area > 0 else 0.0
        aspect_ratio = float(min(rw, rh) / max(rw, rh)) if max(rw, rh) > 0 else 0.0

        # 6. Multi-scale Polygon Approximation & Angles
        poly_002 = cv2.approxPolyDP(main_cnt, 0.02 * peri, True)
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

        min_angle = float(min(angles)) if angles else 180.0
        acute_angles_count = sum(1 for a in angles if a < 68.0)
        right_angles_count = sum(1 for a in angles if 75.0 <= a <= 105.0)

        # 7. Line Segment Detection via Auto-Canny & HoughLinesP
        # Performed both at standardized canvas and native resolution for apex ridge detection
        med = float(np.median(gray[thresh_clean > 0])) if np.any(thresh_clean > 0) else float(np.median(gray))
        t_low = int(max(15, (1.0 - self.canny_sigma) * med))
        t_high = int(min(240, (1.0 + self.canny_sigma) * med))
        edges = cv2.Canny(blurred, t_low, t_high)

        # Mask edges by dilated contour to avoid extreme border noise
        mask_cnt = np.zeros_like(gray)
        cv2.drawContours(mask_cnt, [main_cnt], -1, 255, -1)
        mask_dilated = cv2.dilate(mask_cnt, kernel, iterations=2)
        edges_masked = cv2.bitwise_and(edges, edges, mask=mask_dilated)

        hough_lines = cv2.HoughLinesP(
            edges_masked,
            rho=1,
            theta=np.pi / 180,
            threshold=18,
            minLineLength=20,
            maxLineGap=8
        )

        # Native Resolution Apex Ridge Detection
        gray_native = cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2GRAY)
        blurred_native = cv2.GaussianBlur(gray_native, (7, 7), 0)
        med_native = float(np.median(gray_native))
        t_low_nat = int(max(15, 0.50 * med_native))
        t_high_nat = int(min(240, 1.30 * med_native))
        edges_native = cv2.Canny(blurred_native, t_low_nat, t_high_nat)

        min_line_nat = int(max(20, min(cw, ch) * 0.08))
        max_gap_nat = int(max(5, min(cw, ch) * 0.03))
        hough_native = cv2.HoughLinesP(
            edges_native,
            rho=1,
            theta=np.pi / 180,
            threshold=30,
            minLineLength=min_line_nat,
            maxLineGap=max_gap_nat
        )

        native_ridge_lines_count = 0
        long_axis_vertical = (ch >= cw)
        if hough_native is not None:
            for nl in hough_native:
                line_pts = np.asarray(nl).flatten()
                if len(line_pts) < 4:
                    continue
                nx1, ny1, nx2, ny2 = int(line_pts[0]), int(line_pts[1]), int(line_pts[2]), int(line_pts[3])
                nlen = float(np.hypot(nx2 - nx1, ny2 - ny1))
                nang = float(np.degrees(np.arctan2(ny2 - ny1, nx2 - nx1))) % 180.0
                if long_axis_vertical:
                    mid_x = (nx1 + nx2) / (2.0 * max(1, cw))
                    if 68.0 <= nang <= 112.0 and 0.18 <= mid_x <= 0.82:
                        native_ridge_lines_count += 1
                else:
                    mid_y = (ny1 + ny2) / (2.0 * max(1, ch))
                    if (nang <= 22.0 or nang >= 158.0) and 0.18 <= mid_y <= 0.82:
                        native_ridge_lines_count += 1

        # Determine dominant orientation from minAreaRect
        rect = cv2.minAreaRect(main_cnt)
        dom_angle = rect[2] % 90.0

        line_segments: List[LineSegment] = []
        total_len = 0.0
        slanted_len = 0.0
        ortho_count = 0
        slanted_count = 0

        if hough_lines is not None:
            for l in hough_lines:
                line_pts = np.asarray(l).flatten()
                if len(line_pts) < 4:
                    continue
                x1, y1, x2, y2 = int(line_pts[0]), int(line_pts[1]), int(line_pts[2]), int(line_pts[3])
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

                line_segments.append(
                    LineSegment(
                        x1=int(x1), y1=int(y1), x2=int(x2), y2=int(y2),
                        length=round(length, 1),
                        angle_deg=round(deg, 1),
                        is_orthogonal=is_ortho
                    )
                )

        non_ortho_ratio = float(slanted_len / total_len) if total_len > 0 else 0.0

        # 8. Diagnostic Flags (rotation-invariant rectangular fill complements axis-aligned extent)
        rect_w, rect_h = float(rect[1][0]), float(rect[1][1])
        rect_area = max(1.0, rect_w * rect_h)
        rect_extent = min(1.0, float(area / rect_area))
        flags: Dict[str, bool] = {
            "is_highly_circular": (circularity >= 0.83 and extent >= 0.65),
            "has_acute_angle_junction": (acute_angles_count >= 1 and min_angle < 68.0),
            "has_slanted_lines": (non_ortho_ratio >= 0.20 and slanted_count >= 2),
            "has_strictly_orthogonal_lines": (non_ortho_ratio <= 0.08 and ortho_count >= 3),
            "has_triangular_vertices": (len(poly_004) == 3 or len(poly_006) == 3),
            "has_central_ridge_lines": (native_ridge_lines_count >= 2),
            "high_solidity": (solidity >= 0.95),
            "elongated_extent": (extent <= 0.72 and aspect_ratio <= 0.75),
            "is_rectangular_fill": (
                rect_extent >= 0.82
                and circularity < 0.74
                and acute_angles_count == 0
                and right_angles_count >= 2
            ),
        }

        # 9. Evidence Quality Scoring
        if area >= 3000.0 and len(line_segments) >= 4 and peri >= 150.0:
            evidence_quality = "HIGH"
        elif area >= 1200.0 and len(line_segments) >= 2:
            evidence_quality = "MEDIUM"
        else:
            evidence_quality = "LOW"

        # 10. Candidate Scores from Pure Geometry
        # Scores sum to ~1.0
        score_cyl = 0.1
        score_box = 0.45
        score_prism = 0.45

        # Strong cylinder indicators
        if flags["is_highly_circular"]:
            score_cyl += 0.75
            score_box -= 0.35
            score_prism -= 0.40
        elif circularity >= 0.78:
            score_cyl += 0.30

        # Strong triangular prism indicators
        if flags["has_triangular_vertices"] or flags["has_acute_angle_junction"]:
            score_prism += 0.40
            score_box -= 0.30
        if flags["has_central_ridge_lines"]:
            score_prism += 0.45
            score_box -= 0.35
        if flags["has_slanted_lines"]:
            score_prism += 0.25
            score_box -= 0.20
        if flags["elongated_extent"] and not flags["is_highly_circular"]:
            score_prism += 0.15

        # Strong rectangular box indicators
        if flags["has_strictly_orthogonal_lines"] and right_angles_count >= 2:
            score_box += 0.30
            score_prism -= 0.20
        if (extent >= 0.80 or rect_extent >= 0.84) and not flags["has_acute_angle_junction"] and not flags["has_slanted_lines"]:
            score_box += 0.20
            score_prism -= 0.15

        # Normalize scores to probabilities
        raw_scores = np.array([max(0.01, score_cyl), max(0.01, score_box), max(0.01, score_prism)])
        probs = raw_scores / np.sum(raw_scores)
        candidate_scores = {
            "cylinder": round(float(probs[0]), 3),
            "rectangular_box": round(float(probs[1]), 3),
            "triangular_prism": round(float(probs[2]), 3),
        }

        best_cls = max(candidate_scores, key=candidate_scores.get)
        if candidate_scores[best_cls] < 0.50:
            geometry_candidate = "UNKNOWN"
        else:
            geometry_candidate = best_cls

        return GeometryAnalysisResult(
            evidence_quality=evidence_quality,
            diagnostic_flags=flags,
            contour_area=round(area, 1),
            perimeter=round(peri, 1),
            circularity=round(circularity, 3),
            solidity=round(solidity, 3),
            extent=round(extent, 3),
            aspect_ratio=round(aspect_ratio, 3),
            poly_vertices_002=len(poly_002),
            poly_vertices_004=len(poly_004),
            poly_vertices_006=len(poly_006),
            min_corner_angle=round(min_angle, 1),
            acute_angles_count=acute_angles_count,
            right_angles_count=right_angles_count,
            total_lines=len(line_segments),
            orthogonal_lines_count=ortho_count,
            slanted_lines_count=slanted_count,
            non_orthogonal_length_ratio=round(non_ortho_ratio, 3),
            detected_lines=line_segments,
            geometry_candidate=geometry_candidate,
            candidate_scores=candidate_scores,
            crop_size=(cw, ch),
            raw_contour=main_cnt
        )

    def draw_visualization(
        self,
        image_input: Union[str, Path, Image.Image, np.ndarray],
        result: GeometryAnalysisResult,
        bbox: Optional[Tuple[int, int, int, int]] = None,
        title: str = "",
    ) -> np.ndarray:
        """Renders diagnostic overlay onto the standardized crop for explainability."""
        # Standardize crop to self.target_size canvas
        if isinstance(image_input, (str, Path)):
            with Image.open(image_input) as pil_img:
                pil_img = ImageOps.exif_transpose(pil_img)
                rgb = np.array(pil_img.convert("RGB"))
        elif isinstance(image_input, Image.Image):
            rgb = np.array(ImageOps.exif_transpose(image_input).convert("RGB"))
        else:
            rgb = image_input.copy()

        h_full, w_full = rgb.shape[:2]
        if bbox is not None:
            bx1, by1, bx2, by2 = bbox
            bw, bh = bx2 - bx1, by2 - by1
            pad = int(round(max(bw, bh) * 0.14))
            cx1 = max(0, bx1 - pad)
            cy1 = max(0, by1 - pad)
            cx2 = min(w_full, bx2 + pad)
            cy2 = min(h_full, by2 + pad)
            crop = rgb[cy1:cy2, cx1:cx2]
        else:
            crop = rgb.copy()

        ch, cw = crop.shape[:2]
        scale = float(self.target_size) / float(max(ch, cw))
        nw, nh = max(1, int(round(cw * scale))), max(1, int(round(ch * scale)))
        crop_std = cv2.resize(crop, (nw, nh), interpolation=cv2.INTER_AREA)

        pad_top = (self.target_size - nh) // 2
        pad_bottom = self.target_size - nh - pad_top
        pad_left = (self.target_size - nw) // 2
        pad_right = self.target_size - nw - pad_left
        canvas = cv2.copyMakeBorder(
            crop_std, pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT, value=(20, 20, 20)
        )

        # Convert RGB to BGR for OpenCV drawing
        vis = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)

        # Draw contour
        if result.raw_contour is not None:
            cv2.drawContours(vis, [result.raw_contour], -1, (0, 255, 0), 2)

        # Draw lines: green/yellow for orthogonal, magenta for slanted
        for seg in result.detected_lines:
            color = (0, 255, 255) if seg.is_orthogonal else (255, 0, 255)
            thickness = 2 if not seg.is_orthogonal else 1
            cv2.line(vis, (seg.x1, seg.y1), (seg.x2, seg.y2), color, thickness)

        # Overlay text banner
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (self.target_size, 58), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, self.target_size - 40), (self.target_size, self.target_size), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.75, vis, 0.25, 0, vis)

        header = f"{title} | Quality: {result.evidence_quality} | Cand: {result.geometry_candidate}"
        cv2.putText(vis, header, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
        
        metrics_line = (
            f"Circ: {result.circularity:.2f} | Ext: {result.extent:.2f} | "
            f"V04: {result.poly_vertices_004} | MinAng: {result.min_corner_angle:.0f}°"
        )
        cv2.putText(vis, metrics_line, (6, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 255, 200), 1, cv2.LINE_AA)

        lines_info = (
            f"Lines: {result.total_lines} (Ortho: {result.orthogonal_lines_count}, Slanted: {result.slanted_lines_count}) | "
            f"SlantRatio: {result.non_orthogonal_length_ratio:.2f}"
        )
        cv2.putText(vis, lines_info, (6, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 255), 1, cv2.LINE_AA)

        scores_str = (
            f"P(Cyl)={result.candidate_scores.get('cylinder', 0):.2f} "
            f"P(Box)={result.candidate_scores.get('rectangular_box', 0):.2f} "
            f"P(Prism)={result.candidate_scores.get('triangular_prism', 0):.2f}"
        )
        cv2.putText(vis, scores_str, (6, self.target_size - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1, cv2.LINE_AA)

        return vis
