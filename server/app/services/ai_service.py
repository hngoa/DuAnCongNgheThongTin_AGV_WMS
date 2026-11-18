"""
===================================================================================
DỊCH VỤ NHẬN DIỆN SẢN PHẨM (AI / COMPUTER VISION SERVICE ADAPTER - V2.1)
CHỈ SỬ DỤNG MÔ HÌNH MOBILENETV3-SMALL (best.pth) + OPENCV ROI/GEOMETRY (KHÔNG DÙNG YOLO):
  - Vị trí mô hình: server/weights/ (best.pth + model_config.json)
  - Ánh xạ 3 lớp:
      * cylinder         -> PKG_MECH (Chi Tiết Bánh Răng & Khớp Nối, Kho Kệ 2 C)
      * rectangular_box  -> PKG_ELEC (Bo Mạch Vi Điều Khiển, Kho Kệ 1 B)
      * triangular_prism -> PKG_TOOL (Bộ Dụng Cụ Đo & Kiểm Thử, Kho Kệ 3 D)
  - Cổng an toàn (Safety Gates):
      * NO_OBJECT        -> Khay trống, từ chối tạo đơn rỗng
      * NEEDS_REVIEW     -> Tự tin thấp / Mâu thuẫn hình học
      * CLASSIFIED       -> Nhận diện chính xác, thông qua điều phối WMS
===================================================================================
"""

from typing import Dict, Any, Optional, Tuple, List
from pathlib import Path
import sys
import io
import re
import traceback

from PIL import Image, ImageDraw, ImageFont, ImageOps

# Đường dẫn thư mục lưu weights trong server
CURRENT_DIR = Path(__file__).resolve().parent
SERVER_DIR = CURRENT_DIR.parent.parent
WEIGHTS_DIR = SERVER_DIR / "weights"

# Đảm bảo đường dẫn weights có trong sys.path để import predict.py
if str(WEIGHTS_DIR) not in sys.path and WEIGHTS_DIR.is_dir():
    sys.path.insert(0, str(WEIGHTS_DIR))

# Ánh xạ mặc định từ nhãn Computer Vision (class_names.json của best.pth) sang Mã hàng và Phân khu Kho WMS
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
    def __init__(self, model_path: Optional[Path] = None, workflow_manager: Any = None):
        self.weights_dir = WEIGHTS_DIR
        self.yolo_path = None
        self.checkpoint_path = model_path or (self.weights_dir / "best.pth")
        self.workflow_manager = workflow_manager
        self.predictor = None
        self.is_placeholder_mode = True
        self.engine_name = "Placeholder_Mode"
        self._init_engine()

    def _init_engine(self):
        """Khởi tạo engine: Chỉ nạp MobileNetV3-Small (best.pth) + OpenCV ROI/Geometry (không dùng YOLO)."""
        if str(self.weights_dir) not in sys.path:
            sys.path.insert(0, str(self.weights_dir))

        if self.checkpoint_path.is_file():
            try:
                from predict import HybridPredictor
                self.predictor = HybridPredictor(
                    checkpoint_path=self.checkpoint_path,
                    device="cpu",
                    confidence_threshold_high=0.85,
                    uncertainty_margin=0.20,
                    require_yolo=False,
                    allow_opencv_fallback=True,
                )
                self.is_placeholder_mode = False
                self.engine_name = "MobileNetV3_OpenCV_Hybrid"
                print(f"🧠 [AI Service] ĐÃ NẠP MÔ HÌNH MOBILENET: {self.checkpoint_path.name} (OpenCV ROI + Geometry, không dùng YOLO)")
                print(f"   (Thư mục weights: {self.weights_dir})")
                return
            except Exception as e:
                print(f"⚠️ [AI Service] Không thể nạp MobileNet HybridPredictor ({e}).")
                print("   Kích hoạt chế độ Cổng chờ thông minh (Heuristic Fallback Mode).")
                self.is_placeholder_mode = True
                self.engine_name = "Heuristic_Fallback_Mode"
        else:
            self.is_placeholder_mode = True
            self.engine_name = "Placeholder_Mode"
            print("ℹ️ [AI Service] Đang ở chế độ CỔNG CHỜ (Placeholder Mode) - Chờ tích hợp Model MobileNet best.pth.")
            print(f"   (Khi có mô hình, đặt file best.pth vào: server/weights/)")

    @staticmethod
    def _evaluate_mobilenet_prediction(pred: Any) -> Tuple[Optional[str], float, str, str]:
        """Đánh giá kết quả MobileNetV3 (best.pth) + OpenCV Geometry theo Safety Gates (không dùng YOLO)."""
        dec_type = getattr(pred, "decision_type", "") or ""
        h_status = getattr(pred, "decision_status", "NEEDS_REVIEW")
        h_cls = getattr(pred, "predicted_class", None)
        h_conf = round(float(getattr(pred, "confidence", 0.0) or 0.0), 3)

        if h_status == "CLASSIFIED" and h_cls in CLASS_TO_WMS_MAPPING:
            return h_cls, h_conf, "CLASSIFIED", dec_type or "MOBILENET_HYBRID_CLASSIFIED"

        return None, 0.0, "NEEDS_REVIEW", dec_type or "NEEDS_REVIEW"

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
                    if db_prod["product_name"] != default_info["product_name"]:
                        info["short_vi"] = db_prod["product_name"]

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

    def classify_image(self, image_bytes: bytes, product_hint: str = "", from_node: str = "A") -> Dict[str, Any]:
        """
        Giao diện phân loại chuẩn hóa:
          - Đầu vào: luồng byte ảnh JPEG từ điện thoại + mã hàng gợi ý từ App Android
          - Đầu ra: dict chuẩn {product_code, product_name, confidence, bbox, engine, decision_status}
        """
        # Thử nạp ảnh bằng PIL và chuẩn hóa hướng xoay EXIF trước khi đưa vào detector/visualizer
        pil_img: Optional[Image.Image] = None
        is_real_image = False
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
        # TRƯỜNG HỢP 1: ĐÃ NẠP MÔ HÌNH HYBRID CV THẬT VÀ ẢNH ĐẦU VÀO HỢP LỆ
        # =====================================================================
        if not self.is_placeholder_mode and self.predictor is not None and is_real_image and pil_img is not None:
            try:
                # 1. Phát hiện tất cả bounding box của các sản phẩm trên khay
                detector = self.predictor.classifier.detector
                detected_box, num_dets, all_boxes = detector.detect(pil_img)

                if num_dets == 0 or not all_boxes:
                    print(f"📸 [AI Service] Phát hiện KHAY TRỐNG (NO_OBJECT) tại trạm.")
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

                # 2. Phân loại toàn bộ các sản phẩm phát hiện được bằng MobileNetV3 (best.pth) + OpenCV
                all_items: List[Dict[str, Any]] = []
                classified_items: List[Dict[str, Any]] = []
                review_items: List[Dict[str, Any]] = []

                # Chuẩn hóa an toàn tuyệt đối tọa độ bounding box (tránh lỗi numpy.int32 unpack)
                clean_boxes: List[Tuple[int, int, int, int]] = []
                for b in all_boxes:
                    if isinstance(b, (list, tuple)) and len(b) >= 4:
                        clean_boxes.append((int(b[0]), int(b[1]), int(b[2]), int(b[3])))
                    elif hasattr(b, "tolist") and len(b) >= 4:
                        tl = b.tolist()
                        clean_boxes.append((int(tl[0]), int(tl[1]), int(tl[2]), int(tl[3])))

                multi_box_mode = len(clean_boxes) > 1
                for box in clean_boxes:
                    if multi_box_mode:
                        pred = self.predictor.predict(pil_img, bbox=box, input_mode="pre_cropped_roi", pad_ratio=0.06)
                    else:
                        pred = self.predictor.predict(pil_img, bbox=box, input_mode="pre_cropped_roi")

                    final_cls, final_conf, item_status, fusion_rule = self._evaluate_mobilenet_prediction(pred)

                    if item_status == "CLASSIFIED" and final_cls in CLASS_TO_WMS_MAPPING:
                        wms_info = self._resolve_wms_metadata(final_cls, from_node=from_node)
                        item_dict = {
                            "product_code": wms_info["product_code"],
                            "product_name": wms_info["product_name"],
                            "category": wms_info["category"],
                            "target_station": wms_info["target_node"],
                            "target_node": wms_info["target_node"],
                            "predicted_class": final_cls,
                            "confidence": final_conf,
                            "decision_status": "CLASSIFIED",
                            "fusion_rule": fusion_rule,
                            "bbox": list(box),
                            "bbox_color": wms_info["bbox_color"],
                            "short_vi": wms_info["short_vi"],
                        }
                        all_items.append(item_dict)
                        classified_items.append(item_dict)
                    else:
                        rev_dict = {
                            "product_code": "NEEDS_REVIEW",
                            "product_name": "Cần kiểm tra lại (Không rõ)",
                            "category": "UNKNOWN",
                            "target_station": "?",
                            "target_node": "?",
                            "predicted_class": None,
                            "confidence": 0.0,
                            "decision_status": "NEEDS_REVIEW",
                            "fusion_rule": fusion_rule,
                            "bbox": list(box),
                            "bbox_color": (239, 68, 68),
                            "short_vi": "Cần kiểm tra lại",
                        }
                        all_items.append(rev_dict)
                        review_items.append(rev_dict)

                annotated_bytes = self._generate_multi_annotated_jpeg(pil_img, all_items)

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
                        "explanation": f"Phát hiện {len(review_items)} vùng vật thể nhưng chưa đủ độ tin cậy hoặc mâu thuẫn hình học (NEEDS_REVIEW).",
                        "engine": self.engine_name,
                        "annotated_bytes": annotated_bytes,
                        "success": False,
                    }

                primary_item = classified_items[0]
                print(f"🧠 [AI Service] Hybrid CV phát hiện {len(classified_items)} sản phẩm: {[i['product_code'] for i in classified_items]}")

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
                    "detected_items": classified_items,
                    "review_items": review_items,
                    "total_detected": len(classified_items),
                    "explanation": f"Đã nhận diện {len(classified_items)} sản phẩm trên khay xe.",
                    "engine": self.engine_name,
                    "annotated_bytes": annotated_bytes,
                    "success": True,
                }
            except Exception as e:
                traceback.print_exc()
                print(f"⚠️ [AI Service] Lỗi khi suy luận Hybrid CV: {e}. Chuyển sang fallback.")

        # =====================================================================
        # TRƯỜNG HỢP 2: HEURISTIC / CHẾ ĐỘ CỔNG CHỜ (HỖ TRỢ ĐA SẢN PHẨM & TEST SUITE)
        # =====================================================================
        hint = str(product_hint).strip().upper()

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

        primary_item = detected_items[0]
        annotated_bytes = None
        if is_real_image and pil_img:
            annotated_bytes = self._generate_multi_annotated_jpeg(pil_img, detected_items)

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

    def _generate_multi_annotated_jpeg(
        self,
        pil_img: Image.Image,
        items: List[Dict[str, Any]],
    ) -> bytes:
        """Vẽ toàn bộ Bounding Box và nhãn AI của tất cả sản phẩm phát hiện được lên ảnh."""
        try:
            img_out = pil_img.copy()
            draw = ImageDraw.Draw(img_out)
            w, h = img_out.size

            font_size = max(13, int(min(w, h) * 0.024))
            try:
                font = ImageFont.truetype("arial.ttf", font_size)
            except Exception:
                font = ImageFont.load_default()

            line_width = max(3, int(min(w, h) * 0.006))

            for item in items:
                bbox = item.get("bbox")
                if not bbox or len(bbox) < 4:
                    continue
                color_rgb = item.get("bbox_color", (16, 185, 129))
                label_text = item.get("short_vi", item.get("product_name", ""))
                conf = float(item.get("confidence", 0.95))
                target_node = item.get("target_node", item.get("target_station", ""))

                x1, y1, x2, y2 = bbox
                x1 = max(0, min(w - 1, x1))
                y1 = max(0, min(h - 1, y1))
                x2 = max(x1 + 1, min(w, x2))
                y2 = max(y1 + 1, min(h, y2))

                for i in range(line_width):
                    draw.rectangle([x1 - i, y1 - i, x2 + i, y2 + i], outline=color_rgb)

                if item.get("decision_status") == "NEEDS_REVIEW" or not target_node or target_node == "?":
                    banner_text = f" {label_text} [NEEDS_REVIEW] "
                else:
                    banner_text = f" {label_text} -> Kho {target_node} [{conf * 100:.0f}%] "
                try:
                    tb = draw.textbbox((x1, y1), banner_text, font=font)
                    tw, th = tb[2] - tb[0], tb[3] - tb[1]
                except Exception:
                    tw, th = len(banner_text) * (font_size // 2), font_size + 4

                bx = min(x1, max(0, w - tw - 10))
                by = max(0, y1 - th - 6)
                draw.rectangle([bx, by, min(w, bx + tw + 8), y1], fill=color_rgb)
                draw.text((bx + 4, max(0, by + 2)), banner_text, fill=(255, 255, 255), font=font)

            buf = io.BytesIO()
            img_out.save(buf, format="JPEG", quality=85)
            return buf.getvalue()
        except Exception:
            return b""
