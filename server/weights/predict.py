"""SLAM_KLTN — Hybrid CV Inference CLI & Python API.

Usage Examples:
    # Autonomous full-scene prediction on Station A Android image
    python predict.py --image path/to/image.jpg

    # Force full_scene or pre_cropped_roi mode
    python predict.py --image path/to/image.jpg --mode full_scene
    python predict.py --image path/to/image.jpg --mode pre_cropped_roi

    # Single image with explicit bounding box
    python predict.py --image path/to/image.jpg --bbox 100,50,450,400

    # Batch prediction on a directory
    python predict.py --dir path/to/folder/ --save-json results.json

    # Python API:
    from predict import HybridPredictor
    predictor = HybridPredictor()
    result = predictor.predict("image.jpg")
    print(result.decision_status, result.predicted_class, result.confidence)
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
from PIL import Image

try:
    from .hybrid_classifier import HybridClassifier, HybridPrediction
except (ImportError, ValueError):
    from hybrid_classifier import HybridClassifier, HybridPrediction


class HybridPredictor:
    """Production-ready inference wrapper combining OpenCV ROI localization,
    MobileNetV3-Small classification (best.pth), and OpenCV geometric verification (no YOLO).
    """

    def __init__(
        self,
        checkpoint_path: Optional[Union[str, Path]] = None,
        yolo_path: Optional[Union[str, Path]] = None,
        device: Optional[str] = None,
        confidence_threshold_high: float = 0.85,
        uncertainty_margin: float = 0.20,
        require_yolo: bool = False,
        allow_opencv_fallback: bool = True,
    ):
        pkg_dir = Path(__file__).resolve().parent
        if checkpoint_path is None:
            checkpoint_path = pkg_dir / "best.pth"

        self.classifier = HybridClassifier(
            checkpoint_path=checkpoint_path,
            yolo_path=None,
            device=device,
            confidence_threshold_high=confidence_threshold_high,
            uncertainty_margin=uncertainty_margin,
            require_yolo=False,
            allow_opencv_fallback=True,
        )

    def predict(
        self,
        image_input: Union[str, Path, Image.Image, np.ndarray],
        bbox: Optional[Tuple[int, int, int, int]] = None,
        input_mode: str = "auto",
        pad_ratio: float = 0.14,
        is_bgr: bool = False,
    ) -> HybridPrediction:
        """Run single image hybrid prediction with autonomous ROI localization."""
        return self.classifier.predict(
            image_input=image_input,
            bbox=bbox,
            input_mode=input_mode,
            pad_ratio=pad_ratio,
            is_bgr=is_bgr,
        )

    def predict_batch(
        self,
        image_inputs: List[Union[str, Path, Image.Image, np.ndarray]],
        input_mode: str = "auto",
        pad_ratio: float = 0.14,
        is_bgr: bool = False,
    ) -> List[HybridPrediction]:
        """Run batch inference over a list of images (reuses loaded models in memory)."""
        results = []
        for img in image_inputs:
            results.append(
                self.predict(img, bbox=None, input_mode=input_mode, pad_ratio=pad_ratio, is_bgr=is_bgr)
            )
        return results


def parse_bbox_string(bbox_str: str) -> Tuple[int, int, int, int]:
    """Parse comma-separated 'x1,y1,x2,y2' string into integer tuple."""
    parts = [int(p.strip()) for p in bbox_str.split(",")]
    if len(parts) != 4:
        raise ValueError("Bounding box must have exactly 4 values: x1,y1,x2,y2")
    return tuple(parts)  # type: ignore


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SLAM_KLTN Hybrid Computer Vision Inference CLI",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--image", type=str, help="Path to single image file")
    group.add_argument("--dir", type=str, help="Directory containing images to evaluate")

    parser.add_argument(
        "--mode",
        type=str,
        choices=["auto", "full_scene", "pre_cropped_roi"],
        default="auto",
        help="Input mode: 'full_scene' (enforces detection), 'pre_cropped_roi' (bypasses detection), or 'auto'",
    )
    parser.add_argument(
        "--bbox",
        type=str,
        default=None,
        help="Optional bounding box in format 'x1,y1,x2,y2' (forces ROI crop)",
    )
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="Path to best.pth weights (defaults to best.pth in script folder)",
    )
    parser.add_argument(
        "--yolo-weights",
        type=str,
        default=None,
        help="Path to yolo11n_v5b_best.pt (defaults to file in script folder)",
    )
    parser.add_argument(
        "--allow-fallback",
        action="store_true",
        help="Explicitly permit OpenCV heuristic fallback when YOLO cannot be loaded",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Compute device ('cpu' or 'cuda')",
    )
    parser.add_argument(
        "--save-json",
        type=str,
        default=None,
        help="Save prediction output to JSON file",
    )

    args = parser.parse_args()

    t_start = time.perf_counter()
    try:
        predictor = HybridPredictor(
            checkpoint_path=args.checkpoint,
            yolo_path=args.yolo_weights,
            device=args.device,
            require_yolo=not args.allow_fallback,
            allow_opencv_fallback=args.allow_fallback,
        )
    except Exception as e:
        print(f"Error initializing predictor: {e}", file=sys.stderr)
        return 1

    parsed_bbox = None
    if args.bbox:
        try:
            parsed_bbox = parse_bbox_string(args.bbox)
        except Exception as e:
            print(f"Invalid --bbox argument: {e}", file=sys.stderr)
            return 1

    # Single Image Mode
    if args.image:
        img_path = Path(args.image)
        if not img_path.is_file():
            print(f"Error: image file not found: {img_path}", file=sys.stderr)
            return 1

        pred = predictor.predict(img_path, bbox=parsed_bbox, input_mode=args.mode)
        print("=" * 60)
        print(f"File:            {img_path.name}")
        print(f"Decision Status: {pred.decision_status}")
        print(f"Predicted Class: {pred.predicted_class} (ID: {pred.predicted_class_id})")
        print(f"Confidence:      {pred.confidence:.4f}")
        print(f"Decision Type:   {pred.decision_type} (Override: {pred.is_override})")
        print(f"Detected BBox:   {pred.detected_bbox}")
        print(f"Input Mode:      {pred.input_mode} (Objects Found: {pred.num_detected_objects})")
        print(f"Inference Latency: {pred.inference_latency_ms:.1f} ms")
        print("-" * 60)
        print("1. NEURAL NETWORK PROBABILITIES:")
        for cls_name, prob in pred.neural_network_probabilities.items():
            print(f"   {cls_name:<18}: {prob:.4f}")
        print("2. GEOMETRY EVIDENCE:")
        print(f"   Candidate: {pred.geometry_evidence.get('geometry_candidate')}, "
              f"Quality: {pred.geometry_evidence.get('evidence_quality')}, "
              f"Circularity: {pred.geometry_evidence.get('circularity')}, "
              f"Min Angle: {pred.geometry_evidence.get('min_corner_angle_deg')} deg")
        print("3. FINAL DECISION:")
        print(f"   Status: {pred.decision_status} | Reason: {pred.explanation}")
        print("=" * 60)

        if args.save_json:
            out_p = Path(args.save_json)
            with open(out_p, "w", encoding="utf-8") as f:
                json.dump({"image": str(img_path), "result": pred.to_dict()}, f, indent=2)
            print(f"Results saved to {out_p}")
        return 0

    # Directory Batch Mode
    if args.dir:
        dir_path = Path(args.dir)
        if not dir_path.is_dir():
            print(f"Error: directory not found: {dir_path}", file=sys.stderr)
            return 1

        exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
        image_files = sorted([p for p in dir_path.iterdir() if p.suffix.lower() in exts])
        if not image_files:
            print(f"No image files found in {dir_path}", file=sys.stderr)
            return 1

        print(f"Found {len(image_files)} images in {dir_path}. Running autonomous inference (mode={args.mode})...")
        batch_results = []
        for i, p in enumerate(image_files, 1):
            pred = predictor.predict(p, bbox=None, input_mode=args.mode)
            batch_results.append({"filename": p.name, "result": pred.to_dict()})
            print(f"[{i:03d}/{len(image_files):03d}] {p.name:<28} "
                  f"status={pred.decision_status:<14} class={str(pred.predicted_class):<16} "
                  f"conf={pred.confidence:.3f} bbox={str(pred.detected_bbox):<20} latency={pred.inference_latency_ms:.1f}ms")

        status_counts = {}
        for r in batch_results:
            st = r["result"]["decision_status"]
            status_counts[st] = status_counts.get(st, 0) + 1

        print("\n" + "=" * 60)
        print("BATCH SUMMARY:")
        print(f"Total Images: {len(image_files)}")
        for st, cnt in sorted(status_counts.items()):
            pct = 100.0 * cnt / len(image_files)
            print(f"  {st:<16}: {cnt:3d} ({pct:.1f}%)")
        print("=" * 60)

        if args.save_json:
            out_p = Path(args.save_json)
            with open(out_p, "w", encoding="utf-8") as f:
                json.dump({
                    "directory": str(dir_path),
                    "total_images": len(image_files),
                    "status_summary": status_counts,
                    "items": batch_results,
                }, f, indent=2)
            print(f"Results saved to {out_p}")
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
