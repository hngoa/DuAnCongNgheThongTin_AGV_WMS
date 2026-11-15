"""
===================================================================================
BỘ ĐIỀU PHỐI YÊU CẦU HTTP (CENTRAL HTTP ROUTER & REQUEST HANDLER)
KẾT NỐI CONTROLLER VÀ PHỤC VỤ CÁC FILE TĨNH WEB (HTML, CSS, JS, IMAGES)
===================================================================================
"""

from http.server import BaseHTTPRequestHandler
import json
import socket
from pathlib import Path
from urllib.parse import urlparse, parse_qs
from app.config import (
    STATIC_DIR, TEMPLATES_DIR, UPLOAD_DIR,
    SECURITY_ENABLED, AGV_DEVICE_TOKEN, CAMERA_DEVICE_TOKEN, ADMIN_API_TOKEN
)

class MainHTTPHandler(BaseHTTPRequestHandler):
    # Các controller được gán từ tầng khởi chạy (Dependency Injection)
    agv_controller = None
    camera_controller = None
    wms_controller = None

    def log_message(self, format, *args):
        # Tắt bớt log mặc định để màn hình console sạch đẹp
        pass

    def handle_one_request(self):
        """Bảo vệ vòng lặp request khỏi các ngoại lệ ngắt socket đột ngột từ browser"""
        try:
            super().handle_one_request()
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, socket.error):
            self.close_connection = True

    def finish(self):
        """Bảo vệ quá trình đóng socket tránh raise ConnectionAbortedError"""
        try:
            super().finish()
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, socket.error):
            pass

    def _verify_token(self, allowed_tokens: list) -> bool:
        """
        Xác thực bảo mật kiểm soát truy cập (Device RBAC Authentication Guard):
        Kiểm tra Token hợp lệ qua:
          1. Header 'X-Device-Token'
          2. Header 'Authorization: Bearer <token>'
          3. Query param '?token=<token>'
          4. Trình duyệt Web cục bộ (Same-Origin / Direct Browser Session)
        """
        if not SECURITY_ENABLED:
            return True

        # Trình duyệt web cục bộ trên cùng máy chủ hoặc cùng origin
        sec_fetch = self.headers.get("Sec-Fetch-Site", "")
        referer = self.headers.get("Referer", "")
        if sec_fetch in ("same-origin", "none") or "localhost" in referer or "127.0.0.1" in referer:
            return True

        # 1. Header 'X-Device-Token'
        token = self.headers.get("X-Device-Token", "").strip()
        
        # 2. Header 'Authorization: Bearer <token>'
        if not token:
            auth_header = self.headers.get("Authorization", "").strip()
            if auth_header.startswith("Bearer "):
                token = auth_header[7:].strip()
                
        # 3. URL Query Param '?token=...'
        if not token:
            qs = parse_qs(urlparse(self.path).query)
            token = qs.get("token", [""])[0].strip()

        # Cho phép nếu khớp với Token của thiết bị hoặc Master Admin Token
        if token and (token in allowed_tokens or token == ADMIN_API_TOKEN):
            return True

        self._send_json({
            "status": "error",
            "code": 401,
            "message": "401 Unauthorized: Yêu cầu mã khóa xác thực thiết bị hợp lệ (X-Device-Token)",
            "guard": "AGV_Ver2_Security_RBAC"
        }, 401)
        return False

    def _send_data(self, data: bytes, content_type: str, code: int = 200, extra_headers: dict = None):
        """Gửi dữ liệu qua HTTP an toàn, tự động bắt và xử lý ngoại lệ khi client đóng kết nối (WinError 10053/10054)"""
        try:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(data)))
            if extra_headers:
                for k, v in extra_headers.items():
                    self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, socket.error):
            self.close_connection = True

    def _send_json(self, data, code=200):
        def _json_fallback(o):
            if isinstance(o, (bytes, bytearray)):
                return f"<bytes len={len(o)}>"
            if hasattr(o, "__dict__"):
                return o.__dict__
            return str(o)
        body = json.dumps(data, ensure_ascii=False, default=_json_fallback).encode("utf-8")
        self._send_data(body, "application/json; charset=utf-8", code)

    def do_OPTIONS(self):
        """Hỗ trợ CORS Pre-flight request từ các trình duyệt Web"""
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Device-Token, Authorization")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 0. BỎ QUA HOẶC PHỤC VỤ FAVICON
        if path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return

        # 1. PHỤC VỤ TRANG GIAO DIỆN WEB DASHBOARD & NHẬT KÝ DI CHUYỂN (LOGS)
        clean_path = path.rstrip("/")
        if path in ("/", "/index.html") or clean_path == "":
            index_file = TEMPLATES_DIR / "index.html"
            if index_file.is_file():
                self._send_data(index_file.read_bytes(), "text/html; charset=utf-8", 200)
                return
            else:
                self._send_json({"status": "error", "message": "Template index.html not found"}, 404)
                return

        if clean_path in ("/logs", "/log", "/logs.html", "/log.html"):
            logs_file = TEMPLATES_DIR / "logs.html"
            if logs_file.is_file():
                self._send_data(logs_file.read_bytes(), "text/html; charset=utf-8", 200)
                return
            else:
                self._send_json({"status": "error", "message": "Template logs.html not found"}, 404)
                return

        # 2. PHỤC VỤ FILE TĨNH (CSS & JS)
        if path.startswith("/static/"):
            rel_path = path.replace("/static/", "")
            target_file = STATIC_DIR / rel_path
            if target_file.is_file():
                mime = "text/css" if path.endswith(".css") else "application/javascript" if path.endswith(".js") else "application/octet-stream"
                self._send_data(target_file.read_bytes(), mime, 200)
                return
            else:
                self._send_json({"status": "error", "message": f"Static file not found: {rel_path}"}, 404)
                return

        # 3. PHỤC VỤ ẢNH ĐÃ CHỤP TỪ ĐIỆN THOẠI
        if path.startswith("/images/"):
            fname = Path(path.replace("/images/", "")).name
            img_file = UPLOAD_DIR / fname
            if img_file.is_file():
                self._send_data(img_file.read_bytes(), "image/jpeg", 200)
                return
            else:
                self._send_json({"status": "error", "message": "Image not found"}, 404)
                return

        # 3.5. HEALTH CHECK ENDPOINT (DÙNG CHO ANDROID VÀ ESP32 PHÁT HIỆN SERVER)
        if path in ("/api/camera/health", "/api/health"):
            if self.camera_controller:
                self.camera_controller.record_poll()
            self._send_json({"status": "ok", "service": "AGV_SERVER", "version": "Ver2_WMS"})
            return

        # 4. CÁC API TRẠNG THÁI & THÔNG TIN
        if path in ("/api/state", "/api/camera/latest", "/api/camera/live.jpg"):
            if not self._verify_token([AGV_DEVICE_TOKEN, CAMERA_DEVICE_TOKEN, ADMIN_API_TOKEN]):
                return
            if path == "/api/state":
                qs = parse_qs(parsed.query)
                ack_cam_id_str = qs.get("ack_cam_cmd_id", [None])[0]
                if ack_cam_id_str and self.camera_controller:
                    try:
                        ack_cam_id = int(ack_cam_id_str)
                        if ack_cam_id > 0:
                            self.camera_controller.ack_command(ack_cam_id)
                    except ValueError:
                        pass
                if self.camera_controller:
                    self.camera_controller.record_poll()
                    self.camera_controller.check_auto_inbound_capture(self.agv_controller)
                cam_info = self.camera_controller.get_latest_info() if self.camera_controller else {}
                state = self.agv_controller.get_dashboard_state(cam_info)
                self._send_json(state)
            elif path == "/api/camera/latest":
                cam_info = self.camera_controller.get_latest_info() if self.camera_controller else {}
                self._send_json({"status": "success", "data": cam_info})
            elif path == "/api/camera/live.jpg":
                frame = self.camera_controller.get_live_frame() if self.camera_controller else None
                if frame:
                    self._send_data(frame, "image/jpeg", 200, {"Cache-Control": "no-store, no-cache, must-revalidate"})
                else:
                    self._send_json({"status": "error", "message": "No live frame available"}, 404)
            return

        # 4.5. CÁC API NHẬT KÝ DI CHUYỂN (MOVEMENT & TELEMETRY LOGS)
        if path.startswith("/api/logs"):
            qs = parse_qs(parsed.query)
            if path == "/api/logs":
                limit = int(qs.get("limit", [100])[0])
                offset = int(qs.get("offset", [0])[0])
                state = qs.get("state", ["ALL"])[0]
                data = self.agv_controller.get_logs_data(limit=limit, offset=offset, state=state)
                self._send_json(data)
                return
            elif path == "/api/logs/export":
                code, filename, csv_bytes = self.agv_controller.export_logs_csv()
                self._send_data(csv_bytes, "text/csv; charset=utf-8", code, {"Content-Disposition": f"attachment; filename={filename}"})
                return

        # 5. CÁC API QUẢN TRỊ WMS (YÊU CẦU QUYỀN ADMIN)
        if path.startswith("/api/wms/") or path == "/api/export_csv":
            if not self._verify_token([ADMIN_API_TOKEN]):
                return

            if path == "/api/wms/map":
                code, data = self.wms_controller.get_map_data()
                self._send_json(data, code)
            elif path == "/api/wms/map/template":
                code, data = self.wms_controller.get_map_template()
                self._send_json(data, code)
            elif path == "/api/wms/map/export":
                code, data = self.wms_controller.export_map()
                self._send_json(data, code)
            elif path == "/api/wms/maps":
                code, data = self.wms_controller.get_all_maps()
                self._send_json(data, code)
            elif path == "/api/wms/rules":
                code, data = self.wms_controller.get_rules()
                self._send_json(data, code)
            elif path == "/api/wms/inventory":
                code, data = self.wms_controller.get_inventory()
                self._send_json(data, code)
            elif path == "/api/wms/tasks":
                code, data = self.wms_controller.get_tasks()
                self._send_json(data, code)
            elif path == "/api/wms/mission_queue":
                code, data = self.wms_controller.get_mission_queue()
                self._send_json(data, code)
            elif path == "/api/wms/products":
                code, data = self.wms_controller.get_products()
                self._send_json(data, code)
            elif path == "/api/export_csv":
                code, filename, csv_bytes = self.wms_controller.export_csv()
                self._send_data(csv_bytes, "text/csv; charset=utf-8", code, {"Content-Disposition": f"attachment; filename={filename}"})
            return

        self._send_json({"status": "error", "message": f"Endpoint not found: {path}"}, 404)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        content_len = int(self.headers.get("Content-Length", 0))
        content_type = self.headers.get("Content-Type", "")
        raw_body = self.rfile.read(content_len) if content_len > 0 else b""

        # 1. API NHẬN ẢNH TỪ APP ANDROID (MULTIPART HOẶC SHOT 2 VERIFY)
        if path in ("/api/camera/upload", "/api/camera/verify_unload", "/api/wms/verify_unload"):
            if not self._verify_token([CAMERA_DEVICE_TOKEN, ADMIN_API_TOKEN]):
                return
            if (path.endswith("verify_unload")) and "multipart" not in content_type:
                code, data = self.camera_controller.verify_unload(raw_body)
            else:
                code, data = self.camera_controller.handle_upload(content_type, raw_body)
            self._send_json(data, code)
            return

        # 2. CÁC API JSON KHÁC (TELEMETRY & COMMAND)
        try:
            body_json = json.loads(raw_body.decode("utf-8")) if raw_body else {}
        except Exception:
            body_json = {}

        if path == "/api/telemetry":
            if not self._verify_token([AGV_DEVICE_TOKEN, ADMIN_API_TOKEN]):
                return
            code, data = self.agv_controller.handle_telemetry(body_json)
            self._send_json(data, code)
        elif path == "/api/command":
            if not self._verify_token([AGV_DEVICE_TOKEN, CAMERA_DEVICE_TOKEN, ADMIN_API_TOKEN]):
                return
            code, data = self.agv_controller.handle_command(body_json)
            self._send_json(data, code)
        elif path in ("/api/camera/trigger", "/api/camera/command"):
            if not self._verify_token([CAMERA_DEVICE_TOKEN, ADMIN_API_TOKEN]):
                return
            cmd = str(body_json.get("command", body_json.get("action", "CAPTURE_NOW"))).upper()
            metadata = body_json.get("metadata", {})
            if "station" in body_json and "station" not in metadata:
                metadata["station"] = body_json["station"]
            if "mode" in body_json and "mode" not in metadata:
                metadata["mode"] = body_json["mode"]
            cmd_id = self.camera_controller.queue_command(cmd, metadata) if self.camera_controller else 0
            self._send_json({
                "status": "success",
                "message": f"Đã phát lệnh {cmd} (#{cmd_id}) xuống camera",
                "cmd_id": cmd_id
            })
            return
        elif path.startswith("/api/logs/"):
            if path == "/api/logs/clear":
                data = self.agv_controller.clear_logs()
                self._send_json(data)
            elif path == "/api/logs/toggle":
                enabled = bool(body_json.get("enabled", True))
                data = self.agv_controller.toggle_logging(enabled)
                self._send_json(data)
            else:
                self._send_json({"status": "error", "message": f"Logs POST endpoint not found: {path}"}, 404)
        elif path.startswith("/api/wms/"):
            if not self._verify_token([ADMIN_API_TOKEN]):
                return
            if path == "/api/wms/map/switch":
                code, data = self.wms_controller.switch_map(body_json)
                self._send_json(data, code)
            elif path == "/api/wms/map/import":
                code, data = self.wms_controller.import_map(body_json)
                self._send_json(data, code)
            elif path == "/api/wms/products/save":
                code, data = self.wms_controller.save_product(body_json)
                self._send_json(data, code)
            elif path == "/api/wms/products/delete":
                code, data = self.wms_controller.delete_product(body_json)
                self._send_json(data, code)
            elif path == "/api/wms/rules/save":
                code, data = self.wms_controller.save_rule(body_json)
                self._send_json(data, code)
            elif path == "/api/wms/rules/delete":
                code, data = self.wms_controller.delete_rule(body_json)
                self._send_json(data, code)
            else:
                self._send_json({"status": "error", "message": f"WMS POST endpoint not found: {path}"}, 404)
        else:
            self._send_json({"status": "error", "message": f"POST endpoint not found: {path}"}, 404)
