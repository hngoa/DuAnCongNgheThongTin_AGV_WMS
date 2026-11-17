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
    SECURITY_ENABLED, AGV_DEVICE_TOKEN, CAMERA_DEVICE_TOKEN, ADMIN_API_TOKEN,
    MAX_JSON_BODY_BYTES, MAX_UPLOAD_BODY_BYTES
)
from app.services.security_service import SecurityService, WEB_SESSION_COOKIE_NAME

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

    def _get_client_ip(self) -> str:
        """Lấy địa chỉ IP thực tế từ tầng Socket TCP (Không thể bị giả mạo qua HTTP Header)."""
        addr = getattr(self, "client_address", None)
        if isinstance(addr, (tuple, list)) and len(addr) >= 1:
            return str(addr[0]).strip()
        return ""

    def _extract_device_token(self) -> str:
        """Trích xuất mã khóa xác thực thiết bị qua phương thức duy nhất: Header 'X-Device-Token'."""
        if not hasattr(self, "headers") or not self.headers:
            return ""
        return self.headers.get("X-Device-Token", "").strip()

    def _verify_token(self, allowed_tokens: list) -> bool:
        """
        Xác thực bảo mật kiểm soát truy cập đa tầng (Device RBAC & Anti-Spoofing Guard):
          1. Chặn IP đang bị khóa tạm thời do tấn công vét cạn (Brute-Force Rate Limit).
          2. So sánh Token bằng thuật toán Constant-Time (hmac.compare_digest) qua Header 'X-Device-Token'.
          3. Kiểm tra Cookie phiên Web đã ký HMAC-SHA256 (HttpOnly, SameSite=Strict).
          4. Chỉ cho phép Same-Origin không kèm Token khi kết nối vật lý từ Loopback (127.0.0.1 / ::1),
             tuyệt đối chặn giả mạo Header 'Referer' hoặc 'Sec-Fetch-Site' từ mạng LAN bên ngoài.
        """
        if not SECURITY_ENABLED:
            self._auth_role = "ADMIN"
            return True

        client_ip = self._get_client_ip()
        is_allowed_rate, retry_after = SecurityService.check_rate_limit(client_ip)
        if not is_allowed_rate:
            self._send_json({
                "status": "error",
                "code": 429,
                "message": f"429 Too Many Requests: IP {client_ip} tạm khóa do xác thực sai quá ngưỡng. Thử lại sau {retry_after}s.",
                "guard": "AGV_Ver2_RateLimit_Guard"
            }, 429)
            return False

        token = self._extract_device_token()
        if token:
            if (
                SecurityService.match_token_in_list(token, allowed_tokens)
                or SecurityService.constant_time_equals(token, ADMIN_API_TOKEN)
            ):
                self._auth_role = SecurityService.resolve_token_role(token)
                SecurityService.record_auth_success(client_ip)
                return True

        cookie_hdr = self.headers.get("Cookie", "") if hasattr(self, "headers") and self.headers else ""
        if SecurityService.verify_web_session_cookie(cookie_hdr):
            self._auth_role = "LOCAL_WEB"
            SecurityService.record_auth_success(client_ip)
            return True

        addr = getattr(self, "client_address", None)
        sec_fetch = self.headers.get("Sec-Fetch-Site", "") if hasattr(self, "headers") and self.headers else ""
        if SecurityService.is_loopback_client(addr) and sec_fetch in ("same-origin", "none"):
            self._auth_role = "LOCAL_WEB"
            return True

        SecurityService.record_auth_failure(client_ip)
        self._send_json({
            "status": "error",
            "code": 401,
            "message": "401 Unauthorized: Yêu cầu mã khóa xác thực thiết bị hợp lệ (X-Device-Token)",
            "guard": "AGV_Ver2_Security_RBAC"
        }, 401)
        return False

    def _send_data(self, data: bytes, content_type: str, code: int = 200, extra_headers: dict = None):
        """Gửi dữ liệu qua HTTP an toàn kèm Security Headers, tự động bắt ngoại lệ khi client đóng kết nối"""
        try:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "SAMEORIGIN")
            self.send_header("Referrer-Policy", "same-origin")
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
        self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Device-Token, X-Payload-Signature")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 0. BỎ QUA HOẶC PHỤC VỤ FAVICON
        if path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return

        # 1. PHỤC VỤ TRANG GIAO DIỆN WEB DASHBOARD & NHẬT KÝ DI CHUYỂN (LOGS) KÈM SESSION COOKIE KÝ SỐ HMAC
        clean_path = path.rstrip("/")
        session_cookie = f"{WEB_SESSION_COOKIE_NAME}={SecurityService.generate_web_session_token()}; Path=/; HttpOnly; SameSite=Strict"
        if path in ("/", "/index.html") or clean_path == "":
            index_file = TEMPLATES_DIR / "index.html"
            if index_file.is_file():
                self._send_data(index_file.read_bytes(), "text/html; charset=utf-8", 200, {"Set-Cookie": session_cookie})
                return
            else:
                self._send_json({"status": "error", "message": "Template index.html not found"}, 404)
                return

        if clean_path in ("/logs", "/log", "/logs.html", "/log.html"):
            logs_file = TEMPLATES_DIR / "logs.html"
            if logs_file.is_file():
                self._send_data(logs_file.read_bytes(), "text/html; charset=utf-8", 200, {"Set-Cookie": session_cookie})
                return
            else:
                self._send_json({"status": "error", "message": "Template logs.html not found"}, 404)
                return

        # 2. PHỤC VỤ FILE TĨNH (CSS & JS) - BẢO VỆ CHỐNG PATH TRAVERSAL (../)
        if path.startswith("/static/"):
            rel_path = path[len("/static/"):]
            if ".." in rel_path or "\\" in rel_path or "\x00" in rel_path:
                self._send_json({"status": "error", "code": 403, "message": "403 Forbidden: Phát hiện truy cập đường dẫn trái phép (Path Traversal)"}, 403)
                return
            target_file = STATIC_DIR / rel_path
            if not SecurityService.is_safe_subpath(STATIC_DIR, target_file):
                self._send_json({"status": "error", "code": 403, "message": "403 Forbidden: Truy cập ngoài phạm vi thư mục tĩnh"}, 403)
                return
            if target_file.is_file():
                mime = "text/css" if path.endswith(".css") else "application/javascript" if path.endswith(".js") else "application/octet-stream"
                self._send_data(target_file.read_bytes(), mime, 200)
                return
            else:
                self._send_json({"status": "error", "message": f"Static file not found: {rel_path}"}, 404)
                return

        # 3. PHỤC VỤ ẢNH ĐÃ CHỤP TỪ ĐIỆN THOẠI - BẢO VỆ CHỐNG PATH TRAVERSAL
        if path.startswith("/images/"):
            raw_rel = path[len("/images/"):]
            if ".." in raw_rel or "/" in raw_rel or "\\" in raw_rel or "\x00" in raw_rel:
                self._send_json({"status": "error", "code": 403, "message": "403 Forbidden: Đường dẫn ảnh không hợp lệ"}, 403)
                return
            fname = Path(raw_rel).name
            img_file = UPLOAD_DIR / fname
            if not SecurityService.is_safe_subpath(UPLOAD_DIR, img_file):
                self._send_json({"status": "error", "code": 403, "message": "403 Forbidden"}, 403)
                return
            if img_file.is_file():
                self._send_data(img_file.read_bytes(), "image/jpeg", 200)
                return
            else:
                self._send_json({"status": "error", "message": "Image not found"}, 404)
                return

        # 3.5. HEALTH CHECK ENDPOINT (DÙNG CHO PHÁT HIỆN SERVER QUA mDNS KÈM CHỮ KÝ XÁC THỰC)
        if path == "/api/camera/health":
            if self.camera_controller:
                self.camera_controller.record_poll()
            self._send_json({
                "status": "ok",
                "service": "AGV_SERVER",
                "version": "Ver2_WMS",
                "server_sig": SecurityService.sign_command(1, "AGV_SERVER", "Ver2_WMS")
            })
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
                is_camera_client = (
                    ack_cam_id_str is not None
                    or SecurityService.constant_time_equals(self._extract_device_token(), CAMERA_DEVICE_TOKEN)
                )
                if self.camera_controller:
                    if is_camera_client:
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

        # 4.5. CÁC API NHẬT KÝ DI CHUYỂN (MOVEMENT & TELEMETRY LOGS - YÊU CẦU XÁC THỰC QUẢN TRỊ)
        if path.startswith("/api/logs"):
            if not self._verify_token([ADMIN_API_TOKEN]):
                return
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

    def _read_verified_body(self, max_bytes: int) -> tuple:
        """
        Kiểm tra kích thước Content-Length trước khi đọc vào RAM (Chống DoS tràn bộ nhớ)
        và xác minh chữ ký số HMAC-SHA256 (X-Payload-Signature) nếu thiết bị gửi kèm.
        Trả về: (ok: bool, raw_body: bytes)
        """
        try:
            content_len = int(self.headers.get("Content-Length", 0))
        except ValueError:
            self._send_json({"status": "error", "code": 400, "message": "400 Bad Request: Content-Length không hợp lệ"}, 400)
            return False, b""

        if content_len < 0:
            self._send_json({"status": "error", "code": 400, "message": "400 Bad Request: Content-Length âm"}, 400)
            return False, b""

        if content_len > max_bytes:
            self._send_json({
                "status": "error",
                "code": 413,
                "message": f"413 Payload Too Large: Kích thước gói tin ({content_len} bytes) vượt ngưỡng an toàn ({max_bytes} bytes)",
                "guard": "AGV_Ver2_PayloadSize_Guard"
            }, 413)
            return False, b""

        raw_body = self.rfile.read(content_len) if content_len > 0 else b""
        sig_hdr = self.headers.get("X-Payload-Signature", "").strip() if hasattr(self, "headers") and self.headers else ""
        if sig_hdr and not SecurityService.verify_payload_signature(raw_body, sig_hdr):
            self._send_json({
                "status": "error",
                "code": 401,
                "message": "401 Unauthorized: Chữ ký gói tin HMAC-SHA256 (X-Payload-Signature) không hợp lệ, phát hiện dữ liệu bị can thiệp!",
                "guard": "AGV_Ver2_HMAC_Guard"
            }, 401)
            return False, b""

        return True, raw_body

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        content_type = self.headers.get("Content-Type", "")

        # 1. API NHẬN ẢNH TỪ APP ANDROID (SHOT 1, SHOT 2 & LIVE STREAM)
        if path == "/api/camera/upload":
            if not self._verify_token([CAMERA_DEVICE_TOKEN, ADMIN_API_TOKEN]):
                return
            ok, raw_body = self._read_verified_body(MAX_UPLOAD_BODY_BYTES)
            if not ok:
                return
            code, data = self.camera_controller.handle_upload(content_type, raw_body)
            self._send_json(data, code)
            return

        # 2. CÁC API JSON KHÁC (KIỂM TRA QUYỀN TRƯỚC KHI ĐỌC BODY, GIỚI HẠN 64KB)
        if path == "/api/telemetry":
            if not self._verify_token([AGV_DEVICE_TOKEN, ADMIN_API_TOKEN]):
                return
            ok, raw_body = self._read_verified_body(MAX_JSON_BODY_BYTES)
            if not ok:
                return
            try:
                body_json = json.loads(raw_body.decode("utf-8")) if raw_body else {}
            except Exception:
                body_json = {}
            code, data = self.agv_controller.handle_telemetry(body_json)
            self._send_json(data, code)
            return

        if path == "/api/command":
            # Chỉ cho phép ADMIN, Web Session hoặc CAMERA (với tập lệnh giới hạn); AGV không được phát lệnh điều khiển
            if not self._verify_token([CAMERA_DEVICE_TOKEN, ADMIN_API_TOKEN]):
                return
            ok, raw_body = self._read_verified_body(MAX_JSON_BODY_BYTES)
            if not ok:
                return
            try:
                body_json = json.loads(raw_body.decode("utf-8")) if raw_body else {}
            except Exception:
                body_json = {}

            action = str(body_json.get("action", "")).strip().lower()
            caller_role = getattr(self, "_auth_role", "ADMIN")
            if not SecurityService.is_command_allowed_for_role(caller_role, action):
                self._send_json({
                    "status": "error",
                    "code": 403,
                    "message": f"403 Forbidden: Vai trò '{caller_role}' không được phép thực thi lệnh '{action}'",
                    "guard": "AGV_Ver2_LeastPrivilege_RBAC"
                }, 403)
                return

            code, data = self.agv_controller.handle_command(body_json)
            self._send_json(data, code)
            return

        if path.startswith("/api/logs/"):
            if not self._verify_token([ADMIN_API_TOKEN]):
                return
            ok, raw_body = self._read_verified_body(MAX_JSON_BODY_BYTES)
            if not ok:
                return
            try:
                body_json = json.loads(raw_body.decode("utf-8")) if raw_body else {}
            except Exception:
                body_json = {}
            if path == "/api/logs/clear":
                data = self.agv_controller.clear_logs()
                self._send_json(data)
            elif path == "/api/logs/toggle":
                enabled = bool(body_json.get("enabled", True))
                data = self.agv_controller.toggle_logging(enabled)
                self._send_json(data)
            else:
                self._send_json({"status": "error", "message": f"Logs POST endpoint not found: {path}"}, 404)
            return

        if path.startswith("/api/wms/"):
            if not self._verify_token([ADMIN_API_TOKEN]):
                return
            ok, raw_body = self._read_verified_body(MAX_JSON_BODY_BYTES)
            if not ok:
                return
            try:
                body_json = json.loads(raw_body.decode("utf-8")) if raw_body else {}
            except Exception:
                body_json = {}
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
            return

        self._send_json({"status": "error", "message": f"POST endpoint not found: {path}"}, 404)
