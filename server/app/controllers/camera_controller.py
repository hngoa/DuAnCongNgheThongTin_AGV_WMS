"""
===================================================================================
CONTROLLER XỬ LÝ HÌNH ẢNH CAMERA & TÍCH HỢP AI (CAMERA CONTROLLER)
NHẬN ẢNH TỪ APP ANDROID (CAMERAX), LƯU FILE & GỌI AI PHÂN LOẠI ĐIỀU PHỐI
===================================================================================
"""

from pathlib import Path
from uuid import uuid4
from datetime import datetime
import json
import threading
import time
import re
from typing import Dict, Any, Tuple, Optional
from app.config import UPLOAD_DIR, DEFAULT_MAP_ID

class CameraController:
    def __init__(self, workflow_manager, ai_service):
        self.workflow_manager = workflow_manager
        self.ai_service = ai_service
        self.agv_controller = None
        self.latest_frame_bytes = None
        self.cam_cmd_id = 0
        self.pending_cam_cmd = None
        self.pending_cam_metadata = {}
        self.last_acked_cam_cmd_id = 0
        self.last_cmd_time = 0.0
        self.last_poll_time = 0.0
        self.first_poll_time = 0.0
        self.last_shot1_auto_trigger = 0.0
        self.auto_capture_enabled = True
        self.latest_info = {
            "online": False,
            "received_at": "Chưa nhận ảnh",
            "product_code": "--",
            "confidence": 0.0,
            "task_id": 0,
            "image_url": "",
            "pending_cmd": None,
            "pending_cmd_id": 0
        }
        self.lock = threading.Lock()

    def record_poll(self):
        """Ghi nhận điện thoại Android đang online và poll trạng thái"""
        now = time.time()
        with self.lock:
            if not self.first_poll_time or (now - self.last_poll_time > 5.0):
                self.first_poll_time = now
            self.last_poll_time = now
            self.latest_info["online"] = True

    def check_auto_inbound_capture(self, agv_controller=None) -> bool:
        """
        Tự động kích hoạt Shot 1 (Chụp nhận diện hàng tại Trạm Tiếp Nhận A):
        Tự động chạy khi:
          1. Camera điện thoại đang online (poll trong vòng 5.0s gần nhất).
          2. Không có lệnh camera nào đang chờ thực thi (pending_cam_cmd is None).
          3. Xe AGV đang online và đang đỗ an toàn tại Trạm A (station == 'A', speed < 5.0, IDLE/AT_STATION/STOPPED).
          4. Không có nhiệm vụ WMS nào đang hoạt động (active_mission is None, không có task DISPATCHED/IN_TRANSIT/DELIVERED/UNLOADING).
          5. Cooldown thông minh:
             - Lần đầu tiên sau khi khởi động: Đợi 2.5s sau khi cả Camera và AGV cùng online để góc quay và xe ổn định rồi phát lệnh CAPTURE_NOW ngay.
             - Các lần tiếp theo nếu khay hàng trống (NO_OBJECT): Quét lại sau mỗi 8.0s để tự động nhận diện khi hàng được đặt lên khay.
        """
        if not self.auto_capture_enabled:
            return False

        ctrl = agv_controller or self.agv_controller
        now = time.time()

        # 1. Camera phải đang online
        if not self.last_poll_time or (now - self.last_poll_time) > 5.0:
            return False

        # 2. Không có lệnh camera nào đang chờ thực thi
        with self.lock:
            if self.pending_cam_cmd is not None:
                return False

        # 3. Xe AGV phải đang online
        if not ctrl or not ctrl.last_seen_time or (now - ctrl.last_seen_time) > 6.0:
            return False

        # 4. Xe AGV phải đang dừng đỗ an toàn tại Trạm A
        last_t = getattr(ctrl, "last_telemetry_data", None) or {}
        st = str(last_t.get("station", "A")).upper()
        state = str(last_t.get("state", "IDLE")).upper()
        speed = float(last_t.get("speed", 0.0))

        cur_x = float(last_t.get("x", 0.0))
        cur_y = float(last_t.get("y", 0.0))
        at_station_a = (st == "A") or (abs(cur_x) < 0.25 and abs(cur_y) < 0.25)

        if not at_station_a or speed > 5.0 or ("RUNNING" in state or "TURNING" in state or "GAP" in state):
            return False

        # 5. Hệ thống WMS không có nhiệm vụ nào đang hoạt động
        wf = self.workflow_manager
        if wf.active_mission is not None:
            return False

        recent_tasks = wf.task_repo.get_recent_tasks(limit=5)
        has_active_tasks = any(
            t["status"] in ("DISPATCHED", "IN_TRANSIT", "DELIVERED", "UNLOADING")
            for t in recent_tasks
        )
        if has_active_tasks or wf.active_unloading_tasks:
            return False

        # 6. Kiểm tra thời gian Cooldown:
        agv_first = getattr(ctrl, "first_seen_time", 0.0) or ctrl.last_seen_time
        if self.last_shot1_auto_trigger == 0.0:
            # Lần đầu tiên sau khi khởi động: Đợi tối thiểu 2.0s sau khi cả Camera và AGV cùng online
            ready_start = max(self.first_poll_time, agv_first)
            if (now - ready_start) < 2.0:
                return False
        else:
            # Các lần tiếp theo nếu khay trống: Quét chu kỳ 8.0 giây
            if (now - self.last_shot1_auto_trigger) < 8.0:
                return False

        # ĐỦ ĐIỀU KIỆN -> PHÁT LỆNH CAPTURE_NOW TỰ ĐỘNG!
        self.last_shot1_auto_trigger = now
        print("🎯 [AUTO SHOT 1]: Tự động phát lệnh chụp kiểm tra hàng tại Trạm Tiếp Nhận A (AGV & Camera đã sẵn sàng)")
        self.queue_command("CAPTURE_NOW", metadata={"station": "A", "mode": "CAPTURE"})
        return True

    def queue_command(self, cmd: str, metadata: Optional[Dict[str, Any]] = None) -> int:
        """Phát lệnh điều khiển từ xa cho Camera điện thoại (CAPTURE_NOW, CAPTURE_AND_DISPATCH...)"""
        with self.lock:
            self.cam_cmd_id += 1
            self.pending_cam_cmd = cmd.strip().upper()
            self.pending_cam_metadata = metadata or {}
            self.last_cmd_time = datetime.now().timestamp()
            self.latest_info["pending_cmd"] = self.pending_cam_cmd
            self.latest_info["pending_cmd_id"] = self.cam_cmd_id
            print(f"📡 [Camera Controller] Phát lệnh #{self.cam_cmd_id} xuống điện thoại: {self.pending_cam_cmd} (Meta: {self.pending_cam_metadata})")
            return self.cam_cmd_id

    def ack_command(self, ack_id: int):
        """Xác nhận điện thoại đã nhận và thực thi xong lệnh, đồng bộ cmd_id nếu app Android có id cao hơn"""
        with self.lock:
            # 1. Đồng bộ bộ đếm cam_cmd_id nếu điện thoại gửi lên ack_id cao hơn (tránh lệch bộ đếm khi khởi động lại Server)
            if ack_id > self.cam_cmd_id:
                self.cam_cmd_id = ack_id
                if self.pending_cam_cmd and self.cam_cmd_id <= ack_id:
                    self.cam_cmd_id = ack_id + 1
                    self.latest_info["pending_cmd_id"] = self.cam_cmd_id

            if ack_id >= self.cam_cmd_id:
                self.last_acked_cam_cmd_id = ack_id
                self.pending_cam_cmd = None
                self.pending_cam_metadata = {}
                self.latest_info["pending_cmd"] = None

    def handle_upload(self, content_type: str, body_bytes: bytes) -> Tuple[int, Dict[str, Any]]:
        """Xử lý multipart/form-data upload ảnh từ App Android"""
        fields, files = self._parse_multipart(content_type, body_bytes)
        
        if "image" not in files:
            return 400, {"status": "error", "message": "Thiếu dữ liệu file ảnh ('image')"}

        orig_name, img_bytes = files["image"]
        agv_id = fields.get("agv_id", "AGV_Ver2").strip()
        station = fields.get("station", "A").strip()
        product_hint = fields.get("product_code", "").strip()
        mode = fields.get("mode", "CAPTURE").strip().upper()

        with self.lock:
            self.latest_frame_bytes = img_bytes
            meta = dict(self.pending_cam_metadata)
            # Sau khi nhận ảnh, giải phóng lệnh pending và metadata kèm theo
            self.pending_cam_cmd = None
            self.pending_cam_metadata = {}
            self.latest_info["pending_cmd"] = None

        # Tự động đồng bộ metadata từ lệnh Server phát ra gần nhất (nếu có)
        if meta:
            if meta.get("station"):
                station = meta["station"]
            if meta.get("mode"):
                mode = meta["mode"].upper()

        # Nếu là chế độ truyền live stream khung hình
        if mode == "STREAM":
            with self.lock:
                self.latest_info.update({
                    "online": True,
                    "received_at": datetime.now().strftime("%H:%M:%S"),
                    "image_url": "/api/camera/live.jpg"
                })
            return 200, {"status": "success", "mode": "STREAM", "image_url": "/api/camera/live.jpg"}

        # KIỂM TRA QUY TRÌNH SHOT 2: XÁC NHẬN DỠ HÀNG BẰNG THỊ GIÁC TẠI KHO ĐÍCH
        is_shot_2 = (mode in ("VERIFY_UNLOAD", "SHOT2", "UNLOAD", "VERIFY"))
        awaiting_task = None

        # Nếu trạm gửi lên là "A" nhưng AGV đang ở trạm giao hàng B/C/D chờ dỡ
        if not is_shot_2 and self.workflow_manager:
            recent_tasks = self.workflow_manager.task_repo.get_recent_tasks(limit=5)
            for t in recent_tasks:
                if t["status"] in ("DELIVERED", "UNLOADING") and t.get("to_node") not in ("A", None, ""):
                    station = t["to_node"]
                    is_shot_2 = True
                    awaiting_task = t
                    break
                elif station != "A" and t["status"] in ("DELIVERED", "UNLOADING") and t.get("to_node") == station:
                    is_shot_2 = True
                    awaiting_task = t
                    break

        if is_shot_2:
            return self._handle_shot_2_unload(img_bytes, station, awaiting_task, product_hint=product_hint)

        # Chế độ CHỤP & GIAO HÀNG (CAPTURE & DISPATCH - SHOT 1)
        # 1. Lưu file ảnh vào thư mục uploads
        time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        uid = uuid4().hex[:6]
        fname = f"{time_str}_{station}_{uid}.jpg"
        save_path = UPLOAD_DIR / fname
        save_path.write_bytes(img_bytes)
        img_url = f"/images/{fname}"

        # 2. Gọi AI phân loại sản phẩm
        ai_res = self.ai_service.classify_image(img_bytes, product_hint=product_hint)
        detected_code = ai_res.get("product_code", "PKG_ELEC")
        conf = float(ai_res.get("confidence", 0.95))
        dec_status = ai_res.get("decision_status", "CLASSIFIED")

        # Lưu ảnh BBox AI nếu có
        annotated_bytes = ai_res.get("annotated_bytes")
        if annotated_bytes:
            ann_fname = f"annotated_{fname}"
            ann_path = UPLOAD_DIR / ann_fname
            try:
                ann_path.write_bytes(annotated_bytes)
                img_url = f"/images/{ann_fname}"
            except Exception:
                pass

        # Loại bỏ dữ liệu nhị phân raw bytes khỏi dict kết quả để đảm bảo JSON an toàn 100%
        if "annotated_bytes" in ai_res:
            del ai_res["annotated_bytes"]
        ai_res["annotated_image_url"] = img_url

        # 2b. Cổng an toàn: Nếu khay trống (NO_OBJECT), dừng tạo task
        if dec_status == "NO_OBJECT" or detected_code == "NONE":
            self.last_shot1_auto_trigger = time.time()
            with self.lock:
                self.latest_info.update({
                    "online": True,
                    "received_at": datetime.now().strftime("%H:%M:%S"),
                    "product_code": "KHAY_TRỐNG",
                    "confidence": 0.0,
                    "task_id": 0,
                    "image_url": img_url,
                    "ai_result": ai_res
                })
            return 200, {
                "status": "warning",
                "message": "Cảnh báo: Khay hàng trống (NO_OBJECT), hệ thống tạm dừng tạo nhiệm vụ chuyển kho",
                "ai_result": ai_res,
                "image_url": img_url
            }

        # 3. Kích hoạt luồng WMS & Lập Hàng Đợi Mission tối ưu theo bản đồ hiện hoạt
        self.last_shot1_auto_trigger = time.time()
        active_map = self.workflow_manager.map_repo.get_active_map()
        cur_map_id = active_map["map_id"] if active_map else DEFAULT_MAP_ID

        detected_items = ai_res.get("detected_items") or [{
            "product_code": detected_code,
            "product_name": ai_res.get("product_name", detected_code),
            "category": ai_res.get("category", "ELECTRONIC"),
            "target_station": ai_res.get("target_station", "B"),
            "confidence": conf
        }]

        dispatch_res = self.workflow_manager.plan_and_dispatch_mission_queue(
            map_id=cur_map_id,
            detected_items=detected_items,
            from_node=station,
            image_path=str(save_path)
        )

        with self.lock:
            self.latest_info.update({
                "online": True,
                "received_at": datetime.now().strftime("%H:%M:%S"),
                "product_code": detected_code,
                "confidence": conf,
                "task_id": dispatch_res.get("task_id", 0),
                "image_url": img_url,
                "dispatch": dispatch_res,
                "ai_result": ai_res
            })

        return 200, {
            "status": "success",
            "message": "Đã nhận ảnh, phân loại AI và lập nhiệm vụ thành công",
            "ai_result": ai_res,
            "dispatch": dispatch_res,
            "image_url": img_url
        }

    def _handle_shot_2_unload(self, img_bytes: bytes, station: str, awaiting_task: Optional[Dict[str, Any]] = None, product_hint: str = "") -> Tuple[int, Dict[str, Any]]:
        """Xử lý Shot 2: Kiểm định khay hàng bằng thị giác AI để xác nhận dỡ hàng hoàn tất"""
        time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        uid = uuid4().hex[:6]
        fname = f"shot2_{time_str}_{station}_{uid}.jpg"
        save_path = UPLOAD_DIR / fname
        save_path.write_bytes(img_bytes)
        img_url = f"/images/{fname}"

        ai_res = self.ai_service.classify_image(img_bytes, product_hint=product_hint)
        annotated_bytes = ai_res.get("annotated_bytes")
        if annotated_bytes:
            ann_fname = f"annotated_{fname}"
            ann_path = UPLOAD_DIR / ann_fname
            try:
                ann_path.write_bytes(annotated_bytes)
                img_url = f"/images/{ann_fname}"
            except Exception:
                pass

        # Loại bỏ dữ liệu nhị phân raw bytes khỏi dict kết quả để đảm bảo JSON an toàn 100%
        if "annotated_bytes" in ai_res:
            del ai_res["annotated_bytes"]
        ai_res["annotated_image_url"] = img_url

        task_id = awaiting_task["task_id"] if awaiting_task else None
        unload_res = self.workflow_manager.confirm_unloading_with_vision(
            image_bytes=img_bytes,
            ai_service=self.ai_service,
            station=station,
            task_id=task_id,
            ai_res=ai_res,
            product_hint=product_hint
        )

        with self.lock:
            self.latest_info.update({
                "online": True,
                "received_at": datetime.now().strftime("%H:%M:%S"),
                "product_code": "KHAY_TRỐNG (ĐÃ DỠ)" if unload_res.get("unloaded") else ai_res.get("product_code"),
                "confidence": float(ai_res.get("confidence", 0.0)),
                "task_id": unload_res.get("task_id", 0),
                "image_url": img_url,
                "ai_result": ai_res,
                "unload_verification": unload_res
            })

        return 200, {
            "status": "success" if unload_res.get("success") else "warning",
            "shot": 2,
            "message": unload_res.get("message"),
            "unload_verification": unload_res,
            "ai_result": ai_res,
            "image_url": img_url
        }

    def verify_unload(self, img_bytes: bytes, station: str = "", task_id: Optional[int] = None) -> Tuple[int, Dict[str, Any]]:
        """API tiện ích gọi trực tiếp để kiểm tra dỡ hàng Shot 2"""
        awaiting_task = self.workflow_manager.task_repo.get_by_id(task_id) if task_id else None
        return self._handle_shot_2_unload(img_bytes, station, awaiting_task)

    def get_latest_info(self) -> Dict[str, Any]:
        with self.lock:
            return dict(self.latest_info)

    def get_live_frame(self) -> Optional[bytes]:
        with self.lock:
            return self.latest_frame_bytes

    def _parse_multipart(self, content_type: str, body: bytes) -> Tuple[Dict[str, str], Dict[str, Tuple[str, bytes]]]:
        """Bộ phân tích chuẩn multipart/form-data thuần Python"""
        fields = {}
        files = {}
        m = re.search(r'boundary=([^;]+)', content_type)
        if not m:
            return fields, files
        boundary = m.group(1).strip('"\'').encode("utf-8")
        parts = body.split(b"--" + boundary)

        for p in parts:
            if not p or p.strip() == b"--":
                continue
            if b"\r\n\r\n" not in p:
                continue
            header_part, content = p.split(b"\r\n\r\n", 1)
            content = content.rstrip(b"\r\n")
            header_str = header_part.decode("utf-8", errors="ignore")

            m_name = re.search(r'name="([^"]+)"', header_str)
            if not m_name:
                continue
            field_name = m_name.group(1)

            m_fname = re.search(r'filename="([^"]+)"', header_str)
            if m_fname:
                filename = m_fname.group(1)
                files[field_name] = (filename, content)
            else:
                fields[field_name] = content.decode("utf-8", errors="ignore")

        return fields, files
