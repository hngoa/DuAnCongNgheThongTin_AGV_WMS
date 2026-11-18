"""SLAM_KLTN — Hybrid Classifier: MobileNetV3-Small + OpenCV ROI & Geometric Verification (No YOLO).

Fusion Architecture:
1. Autonomous ROI Localization & Empty Tray Detection via OpenCV (no YOLO dependency).
2. Base classification from MobileNetV3-Small Improved (best.pth).
3. Geometric feature extraction from GeometricAnalyzer (contour, circularity, ridge lines, acute angles).
4. Explainable rule-based fusion with strict safety gates & corroboration:
   - High MobileNet confidence (>=0.85) defaults to MobileNet unless severe contradiction.
   - Low geometry evidence quality (LOW, NO_SIGNAL) never overrides MobileNet.
   - Corroborated Prism / Box rescue: resolves Box <-> Prism tilt ambiguities cleanly.
   - Uncorroborated contradictions emit NEEDS_REVIEW rather than forced classification.
   - Multiple objects detected emit NEEDS_REVIEW in single-item full_scene mode.
   - Empty trays (dark, gray, or bright paper) emit NO_OBJECT.
   - Inverted or out-of-bounds bounding boxes emit INVALID_IMAGE.
   - Strictly calibrated probabilities (no hardcoded 0.80 confidence).
   - Clean separation of Neural Network Probabilities, Geometry Evidence, and Final Decision.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
from PIL import Image
import torch
import torch.nn as nn
from torchvision import models

try:
    from .geometry_analyzer import GeometricAnalyzer, GeometryAnalysisResult
    from .preprocessing import (
        IMAGE_SIZE,
        NORM_MEAN,
        NORM_STD,
        PAD_RGB,
        is_blank_or_textureless,
        load_image_rgb,
        preprocess_for_mobilenet,
        validate_bbox,
    )
    from .roi_extraction import (
        ObjectDetector,
        detect_tray_foreground,
        is_empty_tray,
    )
except (ImportError, ValueError):
    from geometry_analyzer import GeometricAnalyzer, GeometryAnalysisResult
    from preprocessing import (
        IMAGE_SIZE,
        NORM_MEAN,
        NORM_STD,
        PAD_RGB,
        is_blank_or_textureless,
        load_image_rgb,
        preprocess_for_mobilenet,
        validate_bbox,
    )
    from roi_extraction import (
        ObjectDetector,
        detect_tray_foreground,
        is_empty_tray,
    )

CLASSES = ("cylinder", "rectangular_box", "triangular_prism")
IDX_TO_CLASS = {0: "cylinder", 1: "rectangular_box", 2: "triangular_prism"}
CLASS_TO_IDX = {v: k for k, v in IDX_TO_CLASS.items()}

DECISION_STATUS_CLASSIFIED = "CLASSIFIED"
DECISION_STATUS_NEEDS_REVIEW = "NEEDS_REVIEW"
DECISION_STATUS_NO_OBJECT = "NO_OBJECT"
DECISION_STATUS_INVALID_IMAGE = "INVALID_IMAGE"


@dataclass
class HybridPrediction:
    """Standardized prediction result for hybrid classification."""
    decision_status: str  # CLASSIFIED, NEEDS_REVIEW, NO_OBJECT, INVALID_IMAGE
    predicted_class: Optional[str]
    predicted_class_id: Optional[int]
    confidence: float
    decision_type: str
    is_override: bool
    explanation: str
    inference_latency_ms: float
    detected_bbox: Optional[Tuple[int, int, int, int]] = None
    input_mode: str = "auto"
    num_detected_objects: int = 1

    # Strictly Separated Output Components
    neural_network_probabilities: Dict[str, float] = field(default_factory=dict)
    geometry_evidence: Dict[str, Any] = field(default_factory=dict)
    final_decision: Dict[str, Any] = field(default_factory=dict)

    # Backward compatibility aliases
    class_probabilities: Dict[str, float] = field(default_factory=dict)
    mobilenet_class_id: Optional[int] = None
    mobilenet_class_name: Optional[str] = None
    mobilenet_confidence: float = 0.0
    mobilenet_probabilities: Dict[str, float] = field(default_factory=dict)
    geometry_quality: str = "NO_SIGNAL"
    geometry_candidate: str = "UNKNOWN"
    geometry_scores: Dict[str, float] = field(default_factory=dict)
    diagnostic_flags: Dict[str, bool] = field(default_factory=dict)
    needs_review: bool = False
    geometry_result: Optional[GeometryAnalysisResult] = None

    @property
    def final_class_name(self) -> str:
        return self.predicted_class or "UNKNOWN"

    @property
    def final_class_id(self) -> int:
        return self.predicted_class_id if self.predicted_class_id is not None else -1

    @property
    def latency_ms(self) -> float:
        return self.inference_latency_ms

    def to_dict(self) -> Dict[str, Any]:
        """Serialize result to a dictionary suitable for JSON export."""
        d = {
            "decision_status": self.decision_status,
            "predicted_class": self.predicted_class,
            "predicted_class_id": self.predicted_class_id,
            "confidence": self.confidence,
            "decision_type": self.decision_type,
            "is_override": self.is_override,
            "explanation": self.explanation,
            "detected_bbox": list(self.detected_bbox) if self.detected_bbox is not None else None,
            "input_mode": self.input_mode,
            "num_detected_objects": self.num_detected_objects,
            "inference_latency_ms": self.inference_latency_ms,
            "neural_network_probabilities": self.neural_network_probabilities,
            "geometry_evidence": self.geometry_evidence,
            "final_decision": self.final_decision,
            "mobilenet_baseline": {
                "class_name": self.mobilenet_class_name,
                "class_id": self.mobilenet_class_id,
                "confidence": self.mobilenet_confidence,
                "probabilities": self.mobilenet_probabilities,
            },
        }
        return d


class HybridClassifier:
    """Combines OpenCV object localization, MobileNetV3-Small (best.pth), and OpenCV geometry (no YOLO)."""

    def __init__(
        self,
        checkpoint_path: Optional[Union[str, Path]] = None,
        yolo_path: Optional[Union[str, Path]] = None,
        device: Optional[Union[str, torch.device]] = None,
        confidence_threshold_high: float = 0.85,
        uncertainty_margin: float = 0.20,
        require_yolo: bool = False,
        allow_opencv_fallback: bool = True,
    ):
        self.device = torch.device(device) if device is not None else torch.device("cpu")
        pkg_dir = Path(__file__).resolve().parent

        if checkpoint_path is None:
            self.checkpoint_path = pkg_dir / "best.pth"
        else:
            self.checkpoint_path = Path(checkpoint_path)

        # YOLO is explicitly disabled; keep attributes for signature compatibility only
        self.yolo_path = None
        self.confidence_threshold_high = confidence_threshold_high
        self.uncertainty_margin = uncertainty_margin
        self.require_yolo = False
        self.allow_opencv_fallback = True

        # 1. Load MobileNetV3-Small (best.pth) once
        self.model = self._load_mobilenet(self.checkpoint_path, self.device)
        # 2. Initialize Geometric Analyzer
        self.geom_analyzer = GeometricAnalyzer()
        # 3. Initialize OpenCV Detector for autonomous full-scene ROI localization
        self.detector = ObjectDetector(
            require_yolo=False,
            allow_opencv_fallback=True,
        )

    def _load_mobilenet(self, path: Path, device: torch.device) -> nn.Module:
        """Loads MobileNetV3-Small with weights_only=True and strict=True."""
        if not path.is_file():
            raise FileNotFoundError(f"Checkpoint not found at: {path}")
        model = models.mobilenet_v3_small(weights=None)
        in_features = model.classifier[3].in_features
        model.classifier[3] = nn.Linear(in_features, len(CLASSES))

        state = torch.load(path, map_location=device, weights_only=True)
        if isinstance(state, dict) and "model_state_dict" in state:
            state = state["model_state_dict"]
        model.load_state_dict(state, strict=True)
        model.to(device)
        model.eval()
        return model

    @torch.inference_mode()
    def predict(
        self,
        image_input: Union[str, Path, Image.Image, np.ndarray],
        bbox: Optional[Tuple[int, int, int, int]] = None,
        input_mode: str = "auto",  # "auto", "full_scene", "pre_cropped_roi"
        pad_ratio: float = 0.14,
        is_bgr: bool = False,
    ) -> HybridPrediction:
        """Run hybrid inference combining autonomous ROI detection, MobileNetV3, and Geometric verification."""
        t0 = time.perf_counter()

        # Step 0: Validate input image
        try:
            pil_img = load_image_rgb(image_input, is_bgr=is_bgr)
        except Exception as e:
            latency_ms = (time.perf_counter() - t0) * 1000.0
            return HybridPrediction(
                decision_status=DECISION_STATUS_INVALID_IMAGE,
                predicted_class=None,
                predicted_class_id=None,
                confidence=0.0,
                class_probabilities={c: 0.0 for c in CLASSES},
                geometry_evidence={},
                decision_type="INVALID_IMAGE",
                is_override=False,
                explanation=f"Cannot load or decode image: {e}",
                inference_latency_ms=round(latency_ms, 2),
                needs_review=False,
                input_mode=input_mode,
            )

        w_img, h_img = pil_img.size

        # Step 0A: Validate explicit bbox if provided
        effective_bbox = bbox
        if effective_bbox is not None:
            try:
                effective_bbox = validate_bbox(effective_bbox, w_img, h_img)
            except ValueError as e:
                latency_ms = (time.perf_counter() - t0) * 1000.0
                return HybridPrediction(
                    decision_status=DECISION_STATUS_INVALID_IMAGE,
                    predicted_class=None,
                    predicted_class_id=None,
                    confidence=0.0,
                    class_probabilities={c: 0.0 for c in CLASSES},
                    geometry_evidence={},
                    decision_type="INVALID_BBOX",
                    is_override=False,
                    explanation=f"Invalid bounding box: {e}",
                    inference_latency_ms=round(latency_ms, 2),
                    needs_review=False,
                    input_mode=input_mode,
                )

        # Step 0B: Autonomous ROI Extraction when bbox is not provided
        num_detected_objects = 1
        if effective_bbox is None:
            # Check for completely blank / textureless image (empty bright or dark tray)
            if is_blank_or_textureless(pil_img):
                latency_ms = (time.perf_counter() - t0) * 1000.0
                return HybridPrediction(
                    decision_status=DECISION_STATUS_NO_OBJECT,
                    predicted_class=None,
                    predicted_class_id=None,
                    confidence=0.0,
                    class_probabilities={c: 0.0 for c in CLASSES},
                    geometry_evidence={"empty_tray_detected": True},
                    decision_type="NO_OBJECT",
                    is_override=False,
                    explanation="Empty tray detected: image is uniform/textureless with no cargo object.",
                    inference_latency_ms=round(latency_ms, 2),
                    needs_review=False,
                    input_mode=input_mode,
                    num_detected_objects=0,
                )

            # Determine whether to run full_scene detection
            should_run_detector = (input_mode == "full_scene") or (
                input_mode == "auto" and max(w_img, h_img) > 300
            )

            if should_run_detector:
                detected_box, num_dets, all_boxes = self.detector.detect(pil_img)
                num_detected_objects = num_dets

                # Station A Safety Rule: Multiple objects detected on tray -> NEEDS_REVIEW
                if num_dets > 1:
                    latency_ms = (time.perf_counter() - t0) * 1000.0
                    return HybridPrediction(
                        decision_status=DECISION_STATUS_NEEDS_REVIEW,
                        predicted_class=None,
                        predicted_class_id=None,
                        confidence=0.0,
                        class_probabilities={c: 0.0 for c in CLASSES},
                        geometry_evidence={"detected_boxes": all_boxes, "num_objects": num_dets},
                        decision_type="MULTIPLE_OBJECTS_DETECTED",
                        is_override=False,
                        explanation=f"Multiple objects detected on tray ({num_dets} candidates found). Station A constraint requires exactly 1 object.",
                        inference_latency_ms=round(latency_ms, 2),
                        needs_review=True,
                        detected_bbox=detected_box,
                        input_mode=input_mode,
                        num_detected_objects=num_dets,
                    )

                if detected_box is not None:
                    effective_bbox = detected_box
                else:
                    # No object localized on full image
                    latency_ms = (time.perf_counter() - t0) * 1000.0
                    return HybridPrediction(
                        decision_status=DECISION_STATUS_NO_OBJECT,
                        predicted_class=None,
                        predicted_class_id=None,
                        confidence=0.0,
                        class_probabilities={c: 0.0 for c in CLASSES},
                        geometry_evidence={"empty_tray_detected": True},
                        decision_type="NO_OBJECT",
                        is_override=False,
                        explanation="Empty tray detected: no cargo object localized on the tray surface.",
                        inference_latency_ms=round(latency_ms, 2),
                        needs_review=False,
                        input_mode=input_mode,
                        num_detected_objects=0,
                    )
            else:
                # Pre-cropped ROI mode (e.g. 224x224 crop)
                effective_bbox = (0, 0, w_img, h_img)

        # Step 1: Run MobileNetV3 Inference on effective ROI
        try:
            tensor = preprocess_for_mobilenet(
                pil_img, bbox=effective_bbox, pad_ratio=pad_ratio, is_bgr=False
            ).to(self.device)
            logits = self.model(tensor)
            probs = torch.softmax(logits, dim=1)[0].cpu().numpy()
        except Exception as e:
            latency_ms = (time.perf_counter() - t0) * 1000.0
            return HybridPrediction(
                decision_status=DECISION_STATUS_INVALID_IMAGE,
                predicted_class=None,
                predicted_class_id=None,
                confidence=0.0,
                class_probabilities={c: 0.0 for c in CLASSES},
                geometry_evidence={},
                decision_type="PREPROCESSING_FAILURE",
                is_override=False,
                explanation=f"Neural network inference failure: {e}",
                inference_latency_ms=round(latency_ms, 2),
                needs_review=False,
                detected_bbox=effective_bbox,
                input_mode=input_mode,
            )

        nn_cls_id = int(np.argmax(probs))
        nn_cls_name = IDX_TO_CLASS[nn_cls_id]
        nn_conf = float(probs[nn_cls_id])

        # Sort probs to compute confidence margin
        sorted_probs = np.sort(probs)[::-1]
        margin = float(sorted_probs[0] - sorted_probs[1])

        nn_probs_dict = {
            IDX_TO_CLASS[i]: round(float(probs[i]), 4) for i in range(len(CLASSES))
        }

        # Step 2: Run Geometric Analysis on effective ROI
        geom_res = self.geom_analyzer.analyze(pil_img, bbox=effective_bbox, pad_ratio=pad_ratio)
        geo_quality = geom_res.evidence_quality
        geo_cand = geom_res.geometry_candidate
        geo_scores = geom_res.candidate_scores
        flags = geom_res.diagnostic_flags

        geom_evidence_dict = {
            "evidence_quality": geo_quality,
            "geometry_candidate": geo_cand,
            "circularity": round(float(geom_res.circularity), 3),
            "solidity": round(float(geom_res.solidity), 3),
            "extent": round(float(geom_res.extent), 3),
            "aspect_ratio": round(float(geom_res.aspect_ratio), 3),
            "min_corner_angle_deg": round(float(geom_res.min_corner_angle), 1),
            "acute_angles_count": int(geom_res.acute_angles_count),
            "non_orthogonal_line_ratio": round(float(geom_res.non_orthogonal_length_ratio), 3),
            "has_central_ridge_lines": flags.get("has_central_ridge_lines", False),
            "diagnostic_flags": flags,
            "candidate_scores": geo_scores,
        }

        # Step 3: Decision Logic Fusion with Corroboration Safety
        final_id = nn_cls_id
        final_name = nn_cls_name
        decision_type = "MOBILENET_DIRECT"
        decision_status = DECISION_STATUS_CLASSIFIED
        needs_review = False
        is_override = False
        explanation = ""

        # Gate 1: Geometry has No Signal or Low Quality -> Rely strictly on MobileNet
        if geo_quality in ("NO_SIGNAL", "LOW"):
            decision_type = "MOBILENET_DEFAULT_LOW_GEO"
            decision_status = DECISION_STATUS_CLASSIFIED
            explanation = (
                f"Geometry evidence is {geo_quality}. Retaining MobileNet prediction "
                f"({nn_cls_name}, conf={nn_conf:.3f})."
            )

        # Gate 2: MobileNet is highly confident (conf >= 0.85)
        elif nn_conf >= self.confidence_threshold_high:
            if nn_cls_name == "cylinder":
                p_box_nn = float(probs[1])
                p_prism_nn = float(probs[2])
                can_override_high_cyl = (nn_conf < 0.92 and geo_quality in ("HIGH", "MEDIUM"))
                if (
                    can_override_high_cyl
                    and p_box_nn >= 0.05
                    and flags.get("has_strictly_orthogonal_lines", False)
                    and geom_res.extent >= 0.72
                    and geom_res.circularity < 0.65
                ):
                    final_id = 1
                    final_name = "rectangular_box"
                    decision_type = "GEOMETRY_OVERRIDE_BOX"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    is_override = True
                    explanation = (
                        f"Override Cylinder -> Box: High orthogonal rectangular extent ({geom_res.extent:.2f}) "
                        f"overrides Cylinder with low circularity ({geom_res.circularity:.2f})."
                    )
                elif (
                    can_override_high_cyl
                    and p_prism_nn >= 0.05
                    and (flags.get("has_acute_angle_junction") or flags.get("has_triangular_vertices"))
                    and geom_res.min_corner_angle <= 58.0
                    and geom_res.circularity < 0.65
                ):
                    final_id = 2
                    final_name = "triangular_prism"
                    decision_type = "GEOMETRY_OVERRIDE_PRISM"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    is_override = True
                    explanation = (
                        f"Override Cylinder -> Prism: Acute angle ({geom_res.min_corner_angle:.1f} deg) "
                        f"overrides Cylinder with low circularity ({geom_res.circularity:.2f})."
                    )
                elif geom_res.circularity < 0.45 and nn_conf < 0.95:
                    decision_type = "NEEDS_REVIEW"
                    decision_status = DECISION_STATUS_NEEDS_REVIEW
                    needs_review = True
                    explanation = (
                        f"Contradiction: MobileNet predicted cylinder (conf={nn_conf:.3f}) but "
                        f"circularity is very low ({geom_res.circularity:.2f})."
                    )
                else:
                    decision_type = "MOBILENET_DIRECT"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    explanation = (
                        f"High MobileNet confidence ({nn_conf:.3f} >= {self.confidence_threshold_high}). "
                        f"No severe contradiction."
                    )
            elif (
                nn_cls_name == "rectangular_box"
                and flags.get("has_acute_angle_junction")
                and geom_res.min_corner_angle < 50.0
            ):
                decision_type = "NEEDS_REVIEW"
                decision_status = DECISION_STATUS_NEEDS_REVIEW
                needs_review = True
                explanation = (
                    f"Contradiction: MobileNet predicted box (conf={nn_conf:.3f}) but geometry "
                    f"found sharp acute angle ({geom_res.min_corner_angle:.1f} deg)."
                )
            elif (
                nn_cls_name == "triangular_prism"
                and nn_conf < 0.90
                and geo_quality in ("HIGH", "MEDIUM")
                and float(probs[1]) >= 0.05
                and flags.get("has_strictly_orthogonal_lines", False)
                and (flags.get("is_rectangular_fill", False) or geom_res.extent >= 0.88)
                and geom_res.acute_angles_count == 0
            ):
                final_id = 1
                final_name = "rectangular_box"
                decision_type = "GEOMETRY_OVERRIDE_BOX"
                decision_status = DECISION_STATUS_CLASSIFIED
                is_override = True
                explanation = (
                    f"Override Prism -> Box: Orthogonal rectangular contour (extent={geom_res.extent:.2f}, "
                    f"0 acute angles) overrides moderate Prism ({nn_conf:.3f})."
                )
            else:
                decision_type = "MOBILENET_DIRECT"
                decision_status = DECISION_STATUS_CLASSIFIED
                explanation = (
                    f"High MobileNet confidence ({nn_conf:.3f} >= {self.confidence_threshold_high}). "
                    f"No severe contradiction."
                )

        # Gate 3: MobileNet is uncertain or moderately confident (conf < 0.85)
        else:
            # Sub-case 3A: Targeted Prism Rescue with Strict Corroboration
            if nn_cls_name == "rectangular_box":
                has_acute = flags.get("has_acute_angle_junction", False)
                has_tri = flags.get("has_triangular_vertices", False)
                has_slanted = flags.get("has_slanted_lines", False)
                has_ridge = flags.get("has_central_ridge_lines", False)
                p_prism_nn = float(probs[2])

                # Corroboration condition 1: Acute angle + NN corroboration
                is_strictly_orthogonal = flags.get("has_strictly_orthogonal_lines", False)
                high_box_extent = (geom_res.extent >= 0.70) or flags.get("is_rectangular_fill", False)
                if (has_acute or has_tri) and geo_quality in ("HIGH", "MEDIUM") and not is_strictly_orthogonal and not high_box_extent:
                    if p_prism_nn >= 0.35:
                        final_id = 2
                        final_name = "triangular_prism"
                        decision_type = "GEOMETRY_OVERRIDE_PRISM"
                        decision_status = DECISION_STATUS_CLASSIFIED
                        is_override = True
                        explanation = (
                            f"Override Box -> Prism: Acute angle ({geom_res.min_corner_angle:.1f} deg) "
                            f"corroborated by neural network probability (P(Prism)={p_prism_nn:.3f})."
                        )
                    elif p_prism_nn < 0.10 and nn_conf > 0.75:
                        # Contradiction: MobileNet is confident of Box (>75%), acute angle is likely noise/tape
                        decision_type = "NEEDS_REVIEW"
                        decision_status = DECISION_STATUS_NEEDS_REVIEW
                        needs_review = True
                        explanation = (
                            f"Contradiction: MobileNet predicted Box ({nn_conf:.3f}, P(Prism)={p_prism_nn:.3f}) "
                            f"but geometry found uncorroborated acute angle ({geom_res.min_corner_angle:.1f} deg)."
                        )
                    else:
                        decision_type = "NEEDS_REVIEW"
                        decision_status = DECISION_STATUS_NEEDS_REVIEW
                        needs_review = True
                        explanation = (
                            f"Uncorroborated geometry: Acute angle detected ({geom_res.min_corner_angle:.1f} deg) "
                            f"but NN Prism probability ({p_prism_nn:.3f}) is borderline."
                        )
                elif is_strictly_orthogonal or high_box_extent:
                    decision_type = "MOBILENET_CONFIRM_BOX_GEO"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    explanation = (
                        f"Orthogonal rectilinear geometry (extent={geom_res.extent:.2f}) "
                        f"confirms MobileNet Box prediction ({nn_conf:.3f})."
                    )

                # Corroboration condition 2: Longitudinal Apex Ridge
                elif has_ridge and p_prism_nn >= 0.25 and margin < self.uncertainty_margin:
                    final_id = 2
                    final_name = "triangular_prism"
                    decision_type = "GEOMETRY_OVERRIDE_PRISM"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    is_override = True
                    explanation = (
                        f"Override Box -> Prism: Longitudinal apex ridge line detected "
                        f"corroborated by neural network probability (P(Prism)={p_prism_nn:.3f}, margin={margin:.3f})."
                    )

                # Corroboration condition 3: Slanted lines
                elif (
                    has_slanted
                    and p_prism_nn >= 0.20
                    and margin < self.uncertainty_margin
                    and geom_res.non_orthogonal_length_ratio > 0.20
                ):
                    final_id = 2
                    final_name = "triangular_prism"
                    decision_type = "GEOMETRY_OVERRIDE_PRISM"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    is_override = True
                    explanation = (
                        f"Override Box -> Prism: Slanted lines detected ({geom_res.non_orthogonal_length_ratio:.2f}) "
                        f"with close NN margin ({margin:.3f})."
                    )

                elif margin < 0.10:
                    decision_type = "NEEDS_REVIEW"
                    decision_status = DECISION_STATUS_NEEDS_REVIEW
                    needs_review = True
                    explanation = (
                        f"Borderline Box-Prism ambiguity: NN margin is only {margin:.3f} "
                        f"(P(Box)={nn_conf:.3f} vs P(Prism)={p_prism_nn:.3f}) and 2D geometry cannot rule out flat prism."
                    )
                elif flags.get("has_strictly_orthogonal_lines", False) and geom_res.extent >= 0.80:
                    decision_type = "GEOMETRY_CONFIRM_BOX"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    explanation = (
                        f"Geometry confirms Rectangular Box (strictly orthogonal lines, "
                        f"extent={geom_res.extent:.2f}, margin={margin:.3f})."
                    )
                else:
                    decision_type = "MOBILENET_DIRECT"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    explanation = f"MobileNet predicted Box ({nn_conf:.3f}), geometry has no decisive override."

            # Sub-case 3B: MobileNet predicted Cylinder
            elif nn_cls_name == "cylinder":
                p_box_nn = float(probs[1])
                p_prism_nn = float(probs[2])
                # Rescue 1: Strong Box Geometry (strictly orthogonal lines + high rectangular extent)
                if (
                    flags.get("has_strictly_orthogonal_lines", False)
                    and geo_quality in ("HIGH", "MEDIUM")
                    and geom_res.extent >= 0.72
                    and geom_res.circularity < 0.68
                    and (p_box_nn >= 0.20 or nn_conf < 0.65)
                ):
                    final_id = 1
                    final_name = "rectangular_box"
                    decision_type = "GEOMETRY_OVERRIDE_BOX"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    is_override = True
                    explanation = (
                        f"Override Cylinder -> Box: Strongly orthogonal rectangular geometry (extent={geom_res.extent:.2f}, "
                        f"circularity={geom_res.circularity:.2f} < 0.68, P(Box)={p_box_nn:.3f})."
                    )
                # Rescue 2: Strong Prism Geometry corroborated by NN (symmetric with Sub-case 3A)
                elif (
                    (flags.get("has_acute_angle_junction") or flags.get("has_triangular_vertices"))
                    and geo_quality in ("HIGH", "MEDIUM")
                    and geom_res.min_corner_angle <= 58.0
                    and geom_res.circularity < 0.68
                    and p_prism_nn >= 0.30
                ):
                    final_id = 2
                    final_name = "triangular_prism"
                    decision_type = "GEOMETRY_OVERRIDE_PRISM"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    is_override = True
                    explanation = (
                        f"Override Cylinder -> Prism: Acute angle detected ({geom_res.min_corner_angle:.1f} deg) "
                        f"with non-circular geometry ({geom_res.circularity:.2f}, P(Prism)={p_prism_nn:.3f})."
                    )
                elif flags.get("is_highly_circular", False) or geom_res.circularity >= 0.75:
                    decision_type = "GEOMETRY_CONFIRM_CYLINDER"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    explanation = f"Geometry confirms Cylinder (circularity={geom_res.circularity:.2f} >= 0.75)."
                elif geom_res.circularity < 0.60 and (
                    flags.get("has_acute_angle_junction") or flags.get("has_strictly_orthogonal_lines")
                ):
                    decision_type = "NEEDS_REVIEW"
                    decision_status = DECISION_STATUS_NEEDS_REVIEW
                    needs_review = True
                    explanation = (
                        f"Conflict: MobileNet predicted Cylinder (conf={nn_conf:.3f}) but "
                        f"circularity is low ({geom_res.circularity:.2f})."
                    )
                else:
                    decision_type = "MOBILENET_DIRECT"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    explanation = f"MobileNet predicted Cylinder ({nn_conf:.3f}), geometry consistent."

            # Sub-case 3C: MobileNet predicted Triangular Prism
            elif nn_cls_name == "triangular_prism":
                p_box_nn = float(probs[1])
                rect_fill = flags.get("is_rectangular_fill", False)
                if (
                    flags.get("has_strictly_orthogonal_lines", False)
                    and geom_res.acute_angles_count == 0
                    and (geom_res.extent >= 0.88 or rect_fill)
                    and (nn_conf < 0.65 or ((geom_res.extent >= 0.92 or rect_fill) and nn_conf < 0.82 and p_box_nn >= 0.12))
                ):
                    final_id = 1
                    final_name = "rectangular_box"
                    decision_type = "GEOMETRY_OVERRIDE_BOX"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    is_override = True
                    explanation = (
                        f"Override Prism -> Box: Strictly orthogonal rectilinear boundary with high extent "
                        f"({geom_res.extent:.2f}), 0 acute angles, and weak NN Prism conf ({nn_conf:.3f})."
                    )
                elif flags.get("has_acute_angle_junction") or flags.get("has_slanted_lines"):
                    decision_type = "GEOMETRY_CONFIRM_PRISM"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    explanation = "Geometry confirms Prism (acute angle or slanted lines present)."
                elif margin < 0.08:
                    decision_type = "NEEDS_REVIEW"
                    decision_status = DECISION_STATUS_NEEDS_REVIEW
                    needs_review = True
                    explanation = f"Borderline Prism prediction (margin={margin:.3f}) with neutral geometry."
                else:
                    decision_type = "MOBILENET_DIRECT"
                    decision_status = DECISION_STATUS_CLASSIFIED
                    explanation = f"MobileNet predicted Prism ({nn_conf:.3f}), accepted."

        latency_ms = (time.perf_counter() - t0) * 1000.0

        # Calibrated output confidence (honest probability, no hardcoding)
        final_conf = round(float(probs[final_id]), 4) if decision_status == DECISION_STATUS_CLASSIFIED else 0.0

        final_decision_dict = {
            "decision_status": decision_status,
            "predicted_class": final_name if decision_status == DECISION_STATUS_CLASSIFIED else None,
            "predicted_class_id": final_id if decision_status == DECISION_STATUS_CLASSIFIED else None,
            "confidence": final_conf,
            "decision_type": decision_type,
            "is_override": is_override,
            "explanation": explanation,
        }

        return HybridPrediction(
            decision_status=decision_status,
            predicted_class=final_name if decision_status == DECISION_STATUS_CLASSIFIED else None,
            predicted_class_id=final_id if decision_status == DECISION_STATUS_CLASSIFIED else None,
            confidence=final_conf,
            decision_type=decision_type,
            is_override=is_override,
            explanation=explanation,
            inference_latency_ms=round(latency_ms, 2),
            detected_bbox=effective_bbox,
            input_mode=input_mode,
            num_detected_objects=num_detected_objects,
            neural_network_probabilities=nn_probs_dict,
            geometry_evidence=geom_evidence_dict,
            final_decision=final_decision_dict,
            class_probabilities=nn_probs_dict,
            mobilenet_class_id=nn_cls_id,
            mobilenet_class_name=nn_cls_name,
            mobilenet_confidence=round(nn_conf, 4),
            mobilenet_probabilities=nn_probs_dict,
            geometry_quality=geo_quality,
            geometry_candidate=geo_cand,
            geometry_scores=geo_scores,
            diagnostic_flags=flags,
            needs_review=needs_review,
            geometry_result=geom_res,
        )
