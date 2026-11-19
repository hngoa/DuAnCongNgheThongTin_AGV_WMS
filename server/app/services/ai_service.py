"""
===================================================================================
DỊCH VỤ NHẬN DIỆN SẢN PHẨM — CV VER3 ADAPTER (AGV_VER2)
SỬ DỤNG KIẾN TRÚC CV VER3 (SLAMCVRecognizer):
  - Stage 1: YOLO11n (server/weights/yolo11n_v5b_best.pt) — định vị đa vật thể
  - Stage 2: MobileNetV3-Small (server/weights/best.pth) + OpenCV GeometricAnalyzer
  - Stage 3: Hợp nhất ngữ cảnh toàn khung (2-3 vật thể) & Cổng an toàn WMS
  - Ánh xạ 3 lớp (class_names.json):
      * cylinder         -> PKG_MECH (Chi Tiết Bánh Răng & Khớp Nối, Kho Kệ 2 C)
      * rectangular_box  -> PKG_ELEC (Bo Mạch Vi Điều Khiển, Kho Kệ 1 B)
      * triangular_prism -> PKG_TOOL (Bộ Dụng Cụ Đo & Kiểm Thử, Kho Kệ 3 D)
  - Cổng an toàn (Safety Gates):
      * NO_OBJECT        -> Khay trống, từ chối tạo đơn rỗng
      * NEEDS_REVIEW     -> Tự tin thấp / Lớp không hợp lệ
      * CLASSIFIED       -> Nhận diện chính xác, thông qua điều phối WMS
===================================================================================
"""

from typing import Dict, Any, Optional, Tuple, List
from pathlib import Path
import os
import sys
import io
import re
import traceback

try:
    import numpy as np
except ImportError:
    np = None  # Cho phép import/chạy test ở môi trường không cài ML dependencies

from PIL import Image, ImageDraw, ImageFont, ImageOps

# Đường dẫn thư mục lưu weights trong server
CURRENT_DIR = Path(__file__).resolve().parent
SERVER_DIR = CURRENT_DIR.parent.parent
WEIGHTS_DIR = SERVER_DIR / "weights"

# Đảm bảo đường dẫn weights có trong sys.path để import slam_cv_recognizer.py
if str(WEIGHTS_DIR) not in sys.path and WEIGHTS_DIR.is_dir():
    sys.path.insert(0, str(WEIGHTS_DIR))

# Ánh xạ mặc định từ nhãn Computer Vision (class_names.json / best.pth) sang Mã hàng và Phân khu Kho WMS
# Khi AIProductClassifierService gắn với workflow_manager, metadata & target_node được tra động từ SQLite (products + allocation_rules).
CLASS_TO_WMS_MAPPING = {
    "cylinder": {
        "class_id": 0,
        "product_code": "PKG_MECH",
        "product_name": "Chi Tiết Bánh Răng & Khớp Nối",
        "category": "MECHANICAL",
        "target_node": "C",
        "short_vi": "Hình trụ (Cơ khí)",
        "bbox_color": (56, 189, 248),  # Xanh lam
    },
    "rectangular_box": {
        "class_id": 1,
        "product_code": "PKG_ELEC",
        "product_name": "Bo Mạch Vi Điều Khiển",
        "category": "ELECTRONIC",
        "target_node": "B",
        "short_vi": "Hộp chữ nhật (Điện tử)",
        "bbox_color": (16, 185, 129),  # Xanh lá
    },
    "triangular_prism": {
        "class_id": 2,
        "product_code": "PKG_TOOL",
        "product_name": "Bộ Dụng Cụ Đo & Kiểm Thử",
        "category": "TOOL",
        "target_node": "D",
        "short_vi": "Lăng trụ tam giác (Dụng cụ)",
        "bbox_color": (245, 158, 11),  # Cam vàng
    },
}


class AIProductClassifierService:
    def __init__(
        self,
        model_path: Optional[Path] = None,
        workflow_manager: Any = None,
        yolo_path: Optional[Path] = None,
    ):
        self.weights_dir = WEIGHTS_DIR
        self.checkpoint_path = Path(model_path) if model_path else (self.weights_dir / "best.pth")
        self.yolo_path = Path(yolo_path) if yolo_path else (self.checkpoint_path.parent / "yolo11n_v5b_best.pt")
        self.workflow_manager = workflow_manager
        self.recognizer = None
        self.is_placeholder_mode = True
        self.engine_name = "Placeholder_Mode"
        self.init_error = ""
        self._init_engine()

    def _init_engine(self):
        """Khởi tạo engine CV Ver3: YOLO11n (yolo11n_v5b_best.pt) + MobileNetV3-Small (best.pth) + GeometricAnalyzer."""
        weights_parent = str(self.weights_dir)
        if weights_parent not in sys.path and self.weights_dir.is_dir():
            sys.path.insert(0, weights_parent)

        if self.yolo_path.is_file() and self.checkpoint_path.is_file():
            try:
                os.environ.setdefault("YOLO_AUTOINSTALL", "false")
                from slam_cv_recognizer import SLAMCVRecognizer
                self.recognizer = SLAMCVRecognizer(
                    yolo_path=self.yolo_path,
                    mnv3_path=self.checkpoint_path,
                    device="cpu",
                )
                self.is_placeholder_mode = False
                self.engine_name = "CV_Ver3_YOLO11n_MobileNetV3_Geometry"
                self.init_error = ""
                print(
                    f"🧠 [AI Service] ĐÃ NẠP CV VER3 (SLAMCVRecognizer): "
                    f"{self.yolo_path.name} + {self.checkpoint_path.name}"
                )
                print(f"   (Thư mục weights: {self.weights_dir})")
                return
            except Exception as e:
                print(f"⚠️ [AI Service] Không thể nạp CV Ver3 SLAMCVRecognizer ({e}).")
                print("   Kích hoạt chế độ Cổng chờ thông minh (Heuristic Fallback Mode).")
                self.is_placeholder_mode = True
                self.engine_name = "Heuristic_Fallback_Mode"
                self.init_error = str(e)
        else:
            missing = []
            if not self.yolo_path.is_file():
                missing.append(self.yolo_path.name)
            if not self.checkpoint_path.is_file():
                missing.append(self.checkpoint_path.name)
            self.is_placeholder_mode = True
            self.engine_name = "Placeholder_Mode"
            self.init_error = f"Thiếu trọng số CV Ver3: {', '.join(missing)}"
            print(
                f"ℹ️ [AI Service] Đang ở chế độ CỔNG CHỜ (Placeholder Mode) - "
                f"Thiếu trọng số CV Ver3: {', '.join(missing)}."
            )
            print("   (Đặt yolo11n_v5b_best.pt và best.pth vào: server/weights/)")

    @staticmethod
    def _evaluate_ver3_detection(obj: Any) -> Tuple[Optional[str], float, str, str]:
        """Đánh giá kết quả từng đối tượng từ SLAMCVRecognizer.recognize_scene theo Safety Gates WMS."""
        if isinstance(obj, dict):
            dec_type = str(obj.get("decision_type", "") or "")
            dec_status = str(obj.get("decision_status", "") or "")
            cls_name = obj.get("class_name", obj.get("predicted_class", None))
            conf = round(float(obj.get("confidence", 0.0) or 0.0), 3)
        else:
            dec_type = str(getattr(obj, "decision_type", "") or "")
            dec_status = str(getattr(obj, "decision_status", "") or "")
            cls_name = getattr(obj, "class_name", getattr(obj, "predicted_class", None))
            conf = round(float(getattr(obj, "confidence", 0.0) or 0.0), 3)

        if dec_status == "NEEDS_REVIEW" or dec_type == "NEEDS_REVIEW":
            return None, 0.0, "NEEDS_REVIEW", dec_type or "NEEDS_REVIEW"

        if cls_name in CLASS_TO_WMS_MAPPING and conf >= 0.50:
            return cls_name, conf, "CLASSIFIED", dec_type or "CV_VER3_CLASSIFIED"

        return None, 0.0, "NEEDS_REVIEW", dec_type or "LOW_CONFIDENCE_OR_UNKNOWN"

    def _resolve_wms_metadata(self, cls_key: str, from_node: str = "A") -> Dict[str, Any]:
        """Tra cứu danh mục sản phẩm (products) & trạm kho đích (allocation_rules) động từ DB."""
        default_info = CLASS_TO_WMS_MAPPING[cls_key]
        info = dict(default_info)
        wf = getattr(self, "workflow_manager", None)
        if wf is None:
            return info

        try:
            prod_repo = getattr(wf, "prod_repo", None)
            if prod_repo is not None:
                db_prod = None
                if hasattr(prod_repo, "get_by_class_id"):
                    db_prod = prod_repo.get_by_class_id(default_info["class_id"])
                if db_prod is None:
                    db_prod = prod_repo.get_by_code(default_info["product_code"])
                if db_prod:
                    info["product_code"] = db_prod["product_code"]
                    info["product_name"] = db_prod["product_name"]
                    info["category"] = db_prod["category"]
                    shape_vi = {
                        "cylinder": "Hình trụ",
                        "rectangular_box": "Hộp chữ nhật",
                        "triangular_prism": "Lăng trụ",
                    }.get(cls_key, cls_key)
                    if db_prod["product_name"] != default_info["product_name"]:
                        info["short_vi"] = f"{db_prod['product_code']} ({db_prod['product_name']})"
                    else:
                        info["short_vi"] = f"{db_prod['product_code']} ({shape_vi})"

            alloc_repo = getattr(wf, "alloc_repo", None)
            if alloc_repo is not None:
                active_map = wf.map_repo.get_active_map() if getattr(wf, "map_repo", None) else None
                map_id = active_map["map_id"] if active_map else "SABAN_154_151"
                router = wf.get_router(map_id) if hasattr(wf, "get_router") else None
                agv_id = getattr(wf, "agv_id", "AGV_Ver2")
                alloc = alloc_repo.allocate_nearest_target_node(
                    map_id,
                    info["product_code"],
                    info["category"],
                    from_node=from_node,
                    router=router,
                    current_agv_id=agv_id,
                )
                if alloc is None:
                    alloc = alloc_repo.allocate_target_node(map_id, info["product_code"], info["category"])
                if alloc and alloc.get("target_node_id"):
                    info["target_node"] = alloc["target_node_id"]
        except Exception:
            pass
        return info

    def _resolve_fallback_token(self, tok: str, from_node: str = "A") -> Tuple[str, Dict[str, Any]]:
        """Giải mã product_hint trong chế độ Heuristic/Placeholder, ưu tiên tra DB trước."""
        wf = getattr(self, "workflow_manager", None)
        class_id_to_shape = {0: "cylinder", 1: "rectangular_box", 2: "triangular_prism"}
        if wf is not None and getattr(wf, "prod_repo", None) is not None:
            try:
                db_prod = wf.prod_repo.get_by_code(tok)
                if db_prod:
                    shape_key = class_id_to_shape.get(int(db_prod.get("yolo_class_id", 1)), "rectangular_box")
                    info = dict(CLASS_TO_WMS_MAPPING[shape_key])
                    info["product_code"] = db_prod["product_code"]
                    info["product_name"] = db_prod["product_name"]
                    info["category"] = db_prod["category"]
                    info["short_vi"] = db_prod["product_name"]
                    if getattr(wf, "alloc_repo", None) is not None:
                        active_map = wf.map_repo.get_active_map() if getattr(wf, "map_repo", None) else None
                        map_id = active_map["map_id"] if active_map else "SABAN_154_151"
                        router = wf.get_router(map_id) if hasattr(wf, "get_router") else None
                        alloc = wf.alloc_repo.allocate_nearest_target_node(
                            map_id, info["product_code"], info["category"],
                            from_node=from_node, router=router, current_agv_id=getattr(wf, "agv_id", "AGV_Ver2")
                        ) or wf.alloc_repo.allocate_target_node(map_id, info["product_code"], info["category"])
                        if alloc and alloc.get("target_node_id"):
                            info["target_node"] = alloc["target_node_id"]
                    return shape_key, info
            except Exception:
                pass

        if "MECH" in tok or "CO KHI" in tok or "SP02" in tok or "CYL" in tok:
            cls_key = "cylinder"
        elif "TOOL" in tok or "DUNG CU" in tok or "SP03" in tok or "PRISM" in tok:
            cls_key = "triangular_prism"
        else:
            cls_key = "rectangular_box"
        return cls_key, self._resolve_wms_metadata(cls_key, from_node=from_node)

    def classify_image(
        self,
        image_bytes: bytes,
        product_hint: str = "",
        from_node: str = "A",
        unload_station: str = "",
        expected_product_codes: Optional[List[str]] = None,
        planned_targets: Optional[Dict[str, str]] = None,
    ) -> Dict[str, Any]:
        """
        Giao diện phân loại chuẩn hóa:
          - Đầu vào: luồng byte ảnh JPEG từ điện thoại + mã hàng gợi ý từ App Android
          - Đầu ra: dict chuẩn {product_code, product_name, confidence, bbox, engine, decision_status}
        """
        # Thử nạp ảnh bằng PIL và chuẩn hóa hướng xoay EXIF trước khi đưa vào detector/visualizer
        pil_img: Optional[Image.Image] = None
        is_real_image = False
        if isinstance(image_bytes, (bytes, bytearray)) and len(image_bytes) >= 128:
            try:
                with Image.open(io.BytesIO(image_bytes)) as raw_img:
                    transposed = ImageOps.exif_transpose(raw_img) or raw_img
                    pil_img = transposed.convert("RGB")
                # Ảnh thật của điện thoại thường có kích thước tối thiểu >= 64x64
                if pil_img.size[0] >= 64 and pil_img.size[1] >= 64:
                    is_real_image = True
            except Exception:
                pil_img = None
                is_real_image = False

        # =====================================================================
        # TRƯỜNG HỢP 1: ĐÃ NẠP MÔ HÌNH CV VER3 (SLAMCVRecognizer) VÀ ẢNH HỢP LỆ
        # =====================================================================
        if not self.is_placeholder_mode and self.recognizer is not None and is_real_image and pil_img is not None:
            try:
                # 1. Chuẩn hóa ảnh PIL RGB -> OpenCV BGR ndarray đúng quy chuẩn CV Ver3
                if np is not None:
                    rgb_np = np.array(pil_img, dtype=np.uint8)
                    img_bgr = rgb_np[:, :, ::-1].copy()
                else:
                    img_bgr = pil_img

                # 2. Chạy pipeline CV Ver3: YOLO11n + MobileNetV3-Small + GeometricAnalyzer + Scene Reconcile
                detected_objects, _, latency_ms = self.recognizer.recognize_scene(img_bgr)
                detected_objects = list(detected_objects or [])
                lat_ms = round(float(latency_ms or 0.0), 2)

                if not detected_objects:
                    print("📸 [AI Service] CV Ver3 phát hiện KHAY TRỐNG (NO_OBJECT) tại trạm.")
                    return {
                        "product_code": "NONE",
                        "product_name": "Khay trống (Không có hàng)",
                        "confidence": 0.0,
                        "bbox": None,
                        "decision_status": "NO_OBJECT",
                        "detected_items": [],
                        "total_detected": 0,
                        "explanation": "Khay trống không có kiện hàng",
                        "engine": self.engine_name,
                        "latency_ms": lat_ms,
                        "success": False,
                    }

                # 3. Ánh xạ toàn bộ vật thể từ recognize_scene qua Safety Gates & DB WMS
                all_items: List[Dict[str, Any]] = []
                classified_items: List[Dict[str, Any]] = []
                review_items: List[Dict[str, Any]] = []

                for idx, obj in enumerate(detected_objects):
                    raw_bbox = obj.get("bbox", [0, 0, pil_img.size[0], pil_img.size[1]]) if isinstance(obj, dict) else getattr(obj, "bbox", [0, 0, pil_img.size[0], pil_img.size[1]])
                    if hasattr(raw_bbox, "tolist"):
                        raw_bbox = raw_bbox.tolist()
                    clean_box = [int(raw_bbox[0]), int(raw_bbox[1]), int(raw_bbox[2]), int(raw_bbox[3])] if len(raw_bbox) >= 4 else [0, 0, pil_img.size[0], pil_img.size[1]]

                    final_cls, final_conf, item_status, fusion_rule = self._evaluate_ver3_detection(obj)
                    explanation = obj.get("explanation", "") if isinstance(obj, dict) else getattr(obj, "explanation", "")
                    vietnamese_log = obj.get("vietnamese_log", "") if isinstance(obj, dict) else getattr(obj, "vietnamese_log", "")
                    probs = obj.get("probabilities", {}) if isinstance(obj, dict) else getattr(obj, "probabilities", {})
                    geom = obj.get("geometry", {}) if isinstance(obj, dict) else getattr(obj, "geometry", {})
                    obj_id = int(obj.get("object_id", idx + 1)) if isinstance(obj, dict) else int(getattr(obj, "object_id", idx + 1))

                    if item_status == "CLASSIFIED" and final_cls in CLASS_TO_WMS_MAPPING:
                        wms_info = self._resolve_wms_metadata(final_cls, from_node=from_node)
                        if planned_targets and wms_info["product_code"] in planned_targets:
                            wms_info["target_node"] = planned_targets[wms_info["product_code"]]
                        item_dict = {
                            "object_id": obj_id,
                            "product_code": wms_info["product_code"],
                            "product_name": wms_info["product_name"],
                            "category": wms_info["category"],
                            "target_station": wms_info["target_node"],
                            "target_node": wms_info["target_node"],
                            "predicted_class": final_cls,
                            "confidence": final_conf,
                            "decision_status": "CLASSIFIED",
                            "decision_type": fusion_rule,
                            "fusion_rule": fusion_rule,
                            "explanation": explanation,
                            "vietnamese_log": vietnamese_log,
                            "probabilities": probs,
                            "geometry": geom,
                            "bbox": clean_box,
                            "bbox_color": wms_info["bbox_color"],
                            "short_vi": wms_info["short_vi"],
                        }
                        all_items.append(item_dict)
                        classified_items.append(item_dict)
                    else:
                        rev_dict = {
                            "object_id": obj_id,
                            "product_code": "NEEDS_REVIEW",
                            "product_name": "Cần kiểm tra lại (Không rõ)",
                            "category": "UNKNOWN",
                            "target_station": "?",
                            "target_node": "?",
                            "predicted_class": None,
                            "confidence": 0.0,
                            "decision_status": "NEEDS_REVIEW",
                            "decision_type": fusion_rule,
                            "fusion_rule": fusion_rule,
                            "explanation": explanation,
                            "vietnamese_log": vietnamese_log,
                            "probabilities": probs,
                            "geometry": geom,
                            "bbox": clean_box,
                            "bbox_color": (239, 68, 68),
                            "short_vi": "Cần kiểm tra lại",
                        }
                        all_items.append(rev_dict)
                        review_items.append(rev_dict)

                annotated_bytes = self._generate_multi_annotated_jpeg(
                    pil_img,
                    all_items,
                    unload_station=unload_station,
                    expected_product_codes=expected_product_codes,
                )

                if not classified_items:
                    print(f"⚠️ [AI Service] Phát hiện {len(review_items)} vùng nhưng đều NEEDS_REVIEW (từ chối đoán bừa).")
                    return {
                        "product_code": "NEEDS_REVIEW",
                        "product_name": "Cần kiểm tra lại (AI chưa đủ chắc chắn)",
                        "predicted_class": None,
                        "confidence": 0.0,
                        "bbox": review_items[0]["bbox"] if review_items else None,
                        "decision_status": "NEEDS_REVIEW",
                        "detected_items": [],
                        "review_items": review_items,
                        "total_detected": 0,
                        "explanation": f"CV Ver3 phát hiện {len(review_items)} vùng vật thể nhưng chưa đạt ngưỡng tin cậy an toàn (NEEDS_REVIEW).",
                        "engine": self.engine_name,
                        "latency_ms": lat_ms,
                        "annotated_bytes": annotated_bytes,
                        "success": False,
                    }

                primary_item = classified_items[0]
                print(
                    f"🧠 [AI Service] CV Ver3 phát hiện {len(classified_items)} sản phẩm "
                    f"({lat_ms:.1f}ms): {[i['product_code'] for i in classified_items]}"
                )

                return {
                    "product_code": primary_item["product_code"],
                    "product_name": primary_item["product_name"],
                    "predicted_class": primary_item["predicted_class"],
                    "category": primary_item["category"],
                    "target_station": primary_item["target_station"],
                    "target_node": primary_item["target_station"],
                    "confidence": primary_item["confidence"],
                    "bbox": primary_item["bbox"],
                    "decision_status": "CLASSIFIED",
                    "decision_type": primary_item["decision_type"],
                    "explanation": f"CV Ver3 đã nhận diện {len(classified_items)} sản phẩm trên khay xe ({lat_ms:.1f}ms).",
                    "vietnamese_log": primary_item.get("vietnamese_log", ""),
                    "detected_items": classified_items,
                    "review_items": review_items,
                    "total_detected": len(classified_items),
                    "engine": self.engine_name,
                    "latency_ms": lat_ms,
                    "annotated_bytes": annotated_bytes,
                    "success": True,
                }
            except Exception as e:
                traceback.print_exc()
                print(f"⚠️ [AI Service] Lỗi khi suy luận CV Ver3: {e}. Từ chối phân loại giả để đảm bảo an toàn!")
                err_annotated = self._generate_multi_annotated_jpeg(pil_img, [])
                return {
                    "product_code": "NEEDS_REVIEW",
                    "product_name": "Lỗi suy luận CV Ver3 (NEEDS_REVIEW)",
                    "predicted_class": None,
                    "confidence": 0.0,
                    "bbox": None,
                    "decision_status": "NEEDS_REVIEW",
                    "detected_items": [],
                    "review_items": [],
                    "total_detected": 0,
                    "explanation": f"CV Ver3 gặp lỗi khi xử lý ảnh ({e}). Từ chối đoán giả để bảo vệ điều phối WMS.",
                    "engine": self.engine_name,
                    "annotated_bytes": err_annotated,
                    "success": False,
                }

        # =====================================================================
        # TRƯỜNG HỢP 2: HEURISTIC / CHẾ ĐỘ CỔNG CHỜ (CHỈ DÙNG KHI TEST HOẶC CÓ HINT TƯỜNG MINH)
        # =====================================================================
        hint = str(product_hint).strip().upper()

        # Tuyệt đối KHÔNG bịa PKG_ELEC 95% cho ảnh camera thật nếu không có hint và CV Ver3 chưa nạp!
        if is_real_image and not hint:
            reason = getattr(self, "init_error", "") or "CV Ver3 (SLAMCVRecognizer) chưa được nạp"
            print(f"🚨 [AI Service] Từ chối phân loại giả cho ảnh thật ({self.engine_name}: {reason}).")
            clean_jpeg = self._generate_multi_annotated_jpeg(pil_img, []) if pil_img is not None else b""
            return {
                "product_code": "NEEDS_REVIEW",
                "product_name": "CV Ver3 chưa sẵn sàng (Kiểm tra môi trường Python)",
                "predicted_class": None,
                "confidence": 0.0,
                "bbox": None,
                "decision_status": "NEEDS_REVIEW",
                "detected_items": [],
                "review_items": [],
                "total_detected": 0,
                "explanation": f"Từ chối phát BBox giả 95% trên ảnh camera thật vì {reason}.",
                "engine": self.engine_name,
                "annotated_bytes": clean_jpeg,
                "success": False,
            }

        if hint in ("NONE", "EMPTY", "NO_OBJECT", "TRONG", "KHAY_TRONG"):
            return {
                "product_code": "NONE",
                "product_name": "Khay trống (Không có hàng)",
                "confidence": 0.0,
                "bbox": None,
                "decision_status": "NO_OBJECT",
                "detected_items": [],
                "total_detected": 0,
                "explanation": "Khay trống không có kiện hàng",
                "engine": self.engine_name,
                "success": False,
            }

        # Tách danh sách hint nếu chứa nhiều sản phẩm (dấu phẩy, cộng, chấm phẩy)
        tokens = [t.strip() for t in re.split(r"[,+;]+", hint) if t.strip()]
        if not tokens or tokens == [""]:
            tokens = ["PKG_ELEC"]

        detected_items = []
        w_img, h_img = (pil_img.size if is_real_image and pil_img else (640, 480))
        num_toks = len(tokens)

        for idx, tok in enumerate(tokens):
            if "ALL" in tok:
                for k in ("rectangular_box", "cylinder", "triangular_prism"):
                    info = self._resolve_wms_metadata(k, from_node=from_node)
                    detected_items.append({
                        "product_code": info["product_code"],
                        "product_name": info["product_name"],
                        "category": info["category"],
                        "target_station": info["target_node"],
                        "target_node": info["target_node"],
                        "predicted_class": k,
                        "confidence": 0.95,
                        "bbox": [50 + len(detected_items) * 180, 100, 200 + len(detected_items) * 180, 400],
                        "bbox_color": info["bbox_color"],
                        "short_vi": info["short_vi"],
                    })
                break

            cls_key, info = self._resolve_fallback_token(tok, from_node=from_node)
            slot_w = w_img // max(1, num_toks)
            bx1 = int(idx * slot_w + slot_w * 0.1)
            by1 = int(h_img * 0.2)
            bx2 = int((idx + 1) * slot_w - slot_w * 0.1)
            by2 = int(h_img * 0.8)

            detected_items.append({
                "product_code": info["product_code"],
                "product_name": info["product_name"],
                "category": info["category"],
                "target_station": info["target_node"],
                "target_node": info["target_node"],
                "predicted_class": cls_key,
                "confidence": 0.95,
                "bbox": [bx1, by1, bx2, by2],
                "bbox_color": info["bbox_color"],
                "short_vi": info["short_vi"],
            })

        if planned_targets:
            for it in detected_items:
                if it["product_code"] in planned_targets:
                    it["target_node"] = planned_targets[it["product_code"]]
                    it["target_station"] = planned_targets[it["product_code"]]

        primary_item = detected_items[0]
        annotated_bytes = None
        if is_real_image and pil_img:
            annotated_bytes = self._generate_multi_annotated_jpeg(
                pil_img,
                detected_items,
                unload_station=unload_station,
                expected_product_codes=expected_product_codes,
            )

        print(f"📸 [AI Service] Phân loại {len(detected_items)} sản phẩm: {[i['product_code'] for i in detected_items]} | Engine: {self.engine_name}")

        return {
            "product_code": primary_item["product_code"],
            "product_name": primary_item["product_name"],
            "category": primary_item["category"],
            "target_station": primary_item["target_station"],
            "target_node": primary_item["target_station"],
            "confidence": 0.950,
            "bbox": primary_item["bbox"],
            "decision_status": "CLASSIFIED",
            "detected_items": detected_items,
            "total_detected": len(detected_items),
            "explanation": f"Đã nhận diện {len(detected_items)} sản phẩm theo quy tắc phân loại WMS",
            "engine": self.engine_name,
            "annotated_bytes": annotated_bytes,
            "success": True,
        }

    @staticmethod
    def _format_item_banner(
        idx: int,
        item: Dict[str, Any],
        unload_station: str = "",
        expected_product_codes: Optional[List[str]] = None,
    ) -> Tuple[str, bool]:
        """Tạo nội dung nhãn động theo mã hàng thực tế từ DB và ngữ cảnh Shot 1 / Shot 2."""
        obj_id = int(item.get("object_id") or (idx + 1))
        p_code = str(item.get("product_code", "") or "").strip()
        raw_short = str(item.get("short_vi") or item.get("product_name") or p_code).strip()
        conf = float(item.get("confidence", 0.95))
        target_node = str(item.get("target_node") or item.get("target_station") or "").strip()

        if p_code and p_code not in ("NONE", "NEEDS_REVIEW") and not raw_short.startswith(p_code):
            label_core = f"{p_code} ({raw_short})" if raw_short else p_code
        else:
            label_core = raw_short or p_code

        if item.get("decision_status") == "NEEDS_REVIEW" or not target_node or target_node == "?":
            return f" #{obj_id} {label_core} [NEEDS_REVIEW] ", False

        unload_st = str(unload_station or "").strip().upper()
        if unload_st:
            exp_set = {str(c).strip().upper() for c in (expected_product_codes or []) if str(c).strip()}
            is_target_here = (target_node.upper() == unload_st) or (p_code.upper() in exp_set)
            if is_target_here:
                return f" #{obj_id} DỠ @ KHO {unload_st}: {label_core} [{conf * 100:.0f}%] ", True
            return f" #{obj_id} {label_core} (Giữ -> Kho {target_node}) [{conf * 100:.0f}%] ", False

        return f" #{obj_id} {label_core} -> Kho {target_node} [{conf * 100:.0f}%] ", True

    @staticmethod
    def _select_label_rect(
        idx: int,
        norm_boxes: List[Tuple[int, int, int, int]],
        placed_rects: List[Tuple[int, int, int, int]],
        box_w: int,
        box_h: int,
        img_w: int,
        img_h: int,
    ) -> Tuple[int, int, int, int]:
        """Chọn vị trí đặt nhãn thông minh, bám vào bên trong khung vật thể và tránh đè nhãn/vật khác."""
        x1, y1, x2, y2 = norm_boxes[idx]
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)
        margin = 6

        raw_candidates: List[Tuple[int, int]] = []
        if bw >= box_w + 8 and bh >= box_h + 12:
            raw_candidates.extend([
                (x1 + margin, y1 + margin),                         # 1. Góc trên bên trong khung
                (x1 + margin, y2 - box_h - margin),                 # 2. Góc dưới bên trong khung
                (x1 + (bw - box_w) // 2, y1 + margin),              # 3. Giữa-trên bên trong khung
                (x1 + (bw - box_w) // 2, y2 - box_h - margin),      # 4. Giữa-dưới bên trong khung
                (x1 + (bw - box_w) // 2, y1 + (bh - box_h) // 2),   # 5. Chính giữa khung
            ])
        raw_candidates.extend([
            (x1 + (bw - box_w) // 2, y1 + margin),
            (x1 + (bw - box_w) // 2, y2 - box_h - margin),
            (x1, y1 - box_h - 4),
            (x1, y2 + 4),
            (x1 + margin, y1 + margin),
        ])

        def _inter_area(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> int:
            ix1 = max(a[0], b[0])
            iy1 = max(a[1], b[1])
            ix2 = min(a[2], b[2])
            iy2 = min(a[3], b[3])
            return max(0, ix2 - ix1) * max(0, iy2 - iy1)

        label_area = max(1, box_w * box_h)
        best_rect = (4, 4, min(img_w, 4 + box_w), min(img_h, 4 + box_h))
        best_score = float("inf")

        for p_idx, (cx, cy) in enumerate(raw_candidates):
            bx1 = max(4, min(cx, max(4, img_w - box_w - 4)))
            by1 = max(4, min(cy, max(4, img_h - box_h - 4)))
            bx2 = min(img_w - 2, bx1 + box_w)
            by2 = min(img_h - 2, by1 + box_h)
            cand = (bx1, by1, bx2, by2)

            placed_overlap = sum(_inter_area(cand, pr) for pr in placed_rects)
            other_overlap = sum(
                _inter_area(cand, ob) for j, ob in enumerate(norm_boxes) if j != idx
            )
            own_frac = _inter_area(cand, norm_boxes[idx]) / float(label_area)
            score = (
                placed_overlap * 1000.0
                + other_overlap * 40.0
                + (1.0 - own_frac) * 40000.0
                + p_idx * 400.0
            )
            if score < best_score:
                best_score = score
                best_rect = cand

        return best_rect

    def _generate_multi_annotated_jpeg(
        self,
        pil_img: Image.Image,
        items: List[Dict[str, Any]],
        unload_station: str = "",
        expected_product_codes: Optional[List[str]] = None,
    ) -> bytes:
        """Vẽ toàn bộ Bounding Box và nhãn động (tránh chồng chéo, phân biệt Shot 1 / Shot 2)."""
        try:
            img_out = pil_img.copy()
            draw = ImageDraw.Draw(img_out)
            w, h = img_out.size

            base_font_size = max(13, int(min(w, h) * 0.022))

            def _load_font(sz: int):
                try:
                    return ImageFont.truetype("arial.ttf", sz)
                except Exception:
                    return ImageFont.load_default()

            line_width = max(3, int(min(w, h) * 0.006))

            norm_boxes: List[Tuple[int, int, int, int]] = []
            valid_items: List[Dict[str, Any]] = []
            for item in items:
                bbox = item.get("bbox")
                if not bbox or len(bbox) < 4:
                    continue
                x1, y1, x2, y2 = bbox
                x1 = max(0, min(w - 1, int(x1)))
                y1 = max(0, min(h - 1, int(y1)))
                x2 = max(x1 + 1, min(w, int(x2)))
                y2 = max(y1 + 1, min(h, int(y2)))
                norm_boxes.append((x1, y1, x2, y2))
                valid_items.append(item)

            if not valid_items:
                buf = io.BytesIO()
                img_out.save(buf, format="JPEG", quality=85)
                return buf.getvalue()

            banners: List[Tuple[str, bool, Tuple[int, int, int]]] = []
            # Pass 1: Vẽ toàn bộ khung BBox trước để nhãn ở Pass 2 luôn nổi rõ và không bị khung khác đè lên
            for idx, item in enumerate(valid_items):
                x1, y1, x2, y2 = norm_boxes[idx]
                banner_text, is_primary_focus = self._format_item_banner(
                    idx,
                    item,
                    unload_station=unload_station,
                    expected_product_codes=expected_product_codes,
                )
                color_rgb = tuple(item.get("bbox_color", (16, 185, 129)))
                if unload_station and not is_primary_focus and item.get("decision_status") != "NEEDS_REVIEW":
                    color_rgb = (100, 116, 139)  # Xám nhạt cho kiện giữ lại đi trạm khác
                    draw_w = max(2, line_width - 2)
                else:
                    draw_w = line_width + (2 if (unload_station and is_primary_focus) else 0)

                for i in range(draw_w):
                    draw.rectangle([max(0, x1 - i), max(0, y1 - i), min(w - 1, x2 + i), min(h - 1, y2 + i)], outline=color_rgb)
                banners.append((banner_text, is_primary_focus, color_rgb))

            # Pass 2: Đặt nhãn thông minh chống đè chữ & bám vào đúng vật thể
            placed_rects: List[Tuple[int, int, int, int]] = []
            for idx, (banner_text, _focus, color_rgb) in enumerate(banners):
                x1, y1, x2, y2 = norm_boxes[idx]
                bw = max(1, x2 - x1)
                chosen_font = _load_font(base_font_size)
                chosen_sz = base_font_size

                for scale in (1.0, 0.85, 0.72, 0.62):
                    sz = max(12, int(round(base_font_size * scale)))
                    f_try = _load_font(sz)
                    try:
                        tb = draw.textbbox((0, 0), banner_text, font=f_try)
                        tw, th = tb[2] - tb[0], tb[3] - tb[1]
                    except Exception:
                        tw, th = len(banner_text) * max(6, sz // 2), sz + 4
                    chosen_font, chosen_sz = f_try, sz
                    if tw + 16 <= max(bw - 8, int(w * 0.45)) or scale == 0.62:
                        break

                box_w = min(w - 8, tw + 10)
                box_h = th + 10
                bx1, by1, bx2, by2 = self._select_label_rect(
                    idx, norm_boxes, placed_rects, box_w, box_h, w, h
                )
                placed_rects.append((bx1, by1, bx2, by2))

                lum = 0.299 * color_rgb[0] + 0.587 * color_rgb[1] + 0.114 * color_rgb[2]
                text_fill = (15, 23, 42) if lum >= 155 else (255, 255, 255)
                draw.rectangle([bx1, by1, bx2, by2], fill=color_rgb, outline=(15, 23, 42), width=2)
                draw.text((bx1 + 5, by1 + 4), banner_text, fill=text_fill, font=chosen_font)

            buf = io.BytesIO()
            img_out.save(buf, format="JPEG", quality=85)
            return buf.getvalue()
        except Exception:
            return b""
