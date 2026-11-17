"""
===================================================================================
DỊCH VỤ BẢO MẬT HỆ THỐNG IOT AGV_VER2 (IOT SECURITY & CRYPTOGRAPHIC GUARD V2.2)
TÍCH HỢP ĐA TẦNG BẢO VỆ MẠNG - THIẾT BỊ - DỮ LIỆU:
  1. So sánh mã khóa thời gian hằng (Constant-Time Token Comparison - Chống Timing Attack)
  2. Chữ ký số HMAC-SHA256 cho lệnh điều khiển Server -> ESP32 / Camera (Chống MITM & Spoofing)
  3. Xác thực chữ ký gói tin Telemetry / Command từ thiết bị (X-Payload-Signature)
  4. Quản lý phiên Web UI bằng Cookie ký số HMAC HttpOnly (Chống giả mạo Header Referer/Sec-Fetch-Site)
  5. Phân quyền tối thiểu theo vai trò (Least-Privilege RBAC trên /api/command)
  6. Chống tấn công vét cạn (Per-IP Brute-Force Rate Limiting & Lockout)
  7. Kiểm tra an toàn đường dẫn tệp (Path Traversal Canonicalization Guard)
===================================================================================
"""

import hashlib
import hmac
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from app.config import (
    AGV_DEVICE_TOKEN,
    CAMERA_DEVICE_TOKEN,
    ADMIN_API_TOKEN,
    AGV_HMAC_SECRET,
    AUTH_MAX_FAILURES,
    AUTH_WINDOW_SEC,
    AUTH_LOCKOUT_SEC,
)

# Danh sách các hành động an toàn mà Camera Android được phép gọi trên POST /api/command
CAMERA_ALLOWED_COMMANDS = {
    "capture",
    "capture_now",
    "trigger_camera",
    "mission",
    "go_to",
}

WEB_SESSION_COOKIE_NAME = "agv_wms_session"


class SecurityService:
    """Bộ kiểm soát bảo mật trung tâm cho toàn bộ giao tiếp IoT (ESP32, Camera, Web SCADA)"""

    _lock = threading.Lock()
    _failed_attempts: Dict[str, List[float]] = {}
    _locked_until: Dict[str, float] = {}

    @staticmethod
    def constant_time_equals(val_a: str, val_b: str) -> bool:
        """So sánh 2 chuỗi với thời gian hằng (Constant-Time) chống tấn công đo thời gian (Timing Attack)."""
        if not isinstance(val_a, str) or not isinstance(val_b, str):
            return False
        if not val_a or not val_b:
            return False
        return hmac.compare_digest(val_a.encode("utf-8"), val_b.encode("utf-8"))

    @classmethod
    def match_token_in_list(cls, token: str, allowed_tokens: List[str]) -> bool:
        """Kiểm tra token có khớp với danh sách cho phép hay không bằng Constant-Time."""
        if not token:
            return False
        matched = False
        for candidate in allowed_tokens:
            if candidate and cls.constant_time_equals(token, candidate):
                matched = True
        return matched

    @classmethod
    def resolve_token_role(cls, token: str) -> str:
        """Xác định vai trò (Role) của thiết bị dựa trên Token."""
        if not token:
            return "ANONYMOUS"
        if cls.constant_time_equals(token, ADMIN_API_TOKEN):
            return "ADMIN"
        if cls.constant_time_equals(token, AGV_DEVICE_TOKEN):
            return "AGV"
        if cls.constant_time_equals(token, CAMERA_DEVICE_TOKEN):
            return "CAMERA"
        return "INVALID"

    @staticmethod
    def sign_command(cmd_id: int, cmd: Optional[str], route: str = "") -> str:
        """
        Tạo chữ ký số HMAC-SHA256 (128-bit / 32 ký tự hex) cho lệnh điều khiển gửi xuống ESP32-S3 / Camera.
        Chuỗi chuẩn hóa: "<cmd_id>:<cmd>:<route>"
        """
        if not cmd or int(cmd_id or 0) <= 0:
            return ""
        canonical = f"{int(cmd_id)}:{str(cmd).strip()}:{str(route or '').strip()}"
        digest = hmac.new(
            AGV_HMAC_SECRET.encode("utf-8"),
            canonical.encode("utf-8"),
            hashlib.sha256,
        ).hexdigest()
        return digest[:32]

    @classmethod
    def verify_command_signature(
        cls, cmd_id: int, cmd: Optional[str], route: str, signature_hex: str
    ) -> bool:
        """Xác minh chữ ký số HMAC-SHA256 của lệnh điều khiển."""
        expected = cls.sign_command(cmd_id, cmd, route)
        if not expected or not signature_hex:
            return False
        return cls.constant_time_equals(expected, signature_hex.strip().lower())

    @staticmethod
    def sign_payload(raw_bytes: bytes) -> str:
        """Ký số HMAC-SHA256 (32 ký tự hex) trên toàn bộ nội dung gói tin HTTP Body."""
        digest = hmac.new(
            AGV_HMAC_SECRET.encode("utf-8"),
            raw_bytes or b"",
            hashlib.sha256,
        ).hexdigest()
        return digest[:32]

    @classmethod
    def verify_payload_signature(cls, raw_bytes: bytes, signature_hex: str) -> bool:
        """Xác minh chữ ký HMAC-SHA256 của gói tin Telemetry / Command gửi từ ESP32 hoặc Android."""
        if not signature_hex:
            return False
        expected = cls.sign_payload(raw_bytes)
        return cls.constant_time_equals(expected, signature_hex.strip().lower())

    @staticmethod
    def generate_web_session_token() -> str:
        """Sinh mã phiên làm việc (Session Token) đã ký HMAC-SHA256 cho Web SCADA Dashboard."""
        msg = f"AGV_WMS_WEB_SESSION:{ADMIN_API_TOKEN}"
        return hmac.new(
            AGV_HMAC_SECRET.encode("utf-8"),
            msg.encode("utf-8"),
            hashlib.sha256,
        ).hexdigest()[:32]

    @classmethod
    def verify_web_session_cookie(cls, cookie_header: str) -> bool:
        """Kiểm tra Cookie phiên Web Dashboard hợp lệ (Chống giả mạo Referer / Sec-Fetch-Site từ mạng ngoài)."""
        if not cookie_header:
            return False
        expected = cls.generate_web_session_token()
        for part in cookie_header.split(";"):
            part = part.strip()
            if part.startswith(f"{WEB_SESSION_COOKIE_NAME}="):
                val = part[len(WEB_SESSION_COOKIE_NAME) + 1 :].strip()
                if cls.constant_time_equals(val, expected):
                    return True
        return False

    @staticmethod
    def is_loopback_client(client_address: Optional[Tuple[str, int]]) -> bool:
        """
        Kiểm tra kết nối có xuất phát trực tiếp từ máy chủ cục bộ (127.0.0.1 / ::1)
        hoặc từ môi trường kiểm thử nội bộ (client_address is None) hay không.
        """
        if client_address is None:
            return True
        if isinstance(client_address, (tuple, list)) and len(client_address) >= 1:
            ip = str(client_address[0]).strip()
            return ip in ("127.0.0.1", "::1", "localhost", "::ffff:127.0.0.1")
        return False

    @staticmethod
    def is_command_allowed_for_role(role: str, action: str) -> bool:
        """
        Kiểm soát phân quyền tối thiểu (Least-Privilege RBAC) cho từng lệnh trên POST /api/command:
          - ADMIN / LOCAL_WEB : Được phép thực thi mọi lệnh điều khiển & cấu hình.
          - CAMERA            : Chỉ được phép kích hoạt chụp ảnh hoặc gửi yêu cầu điều phối cơ bản;
                                CẤM các lệnh dừng khẩn cấp, xóa nhiệm vụ, reset odometry, xoay xe.
          - AGV / ANONYMOUS   : CẤM gọi POST /api/command.
        """
        act = str(action or "").strip().lower()
        if role in ("ADMIN", "LOCAL_WEB"):
            return True
        if role == "CAMERA":
            return act in CAMERA_ALLOWED_COMMANDS
        return False

    @staticmethod
    def is_safe_subpath(base_dir: Path, target_path: Path) -> bool:
        """Chống tấn công leo thang thư mục (Path Traversal ../) khi truy cập file tĩnh và ảnh."""
        try:
            resolved_base = base_dir.resolve()
            resolved_target = target_path.resolve()
            return resolved_target.is_relative_to(resolved_base)
        except Exception:
            return False

    @classmethod
    def check_rate_limit(cls, client_ip: str) -> Tuple[bool, float]:
        """
        Kiểm tra IP có đang bị khóa tạm thời do gửi sai Token quá ngưỡng (Brute-Force) hay không.
        Trả về: (is_allowed, retry_after_sec)
        """
        if not client_ip:
            return True, 0.0
        now = time.time()
        with cls._lock:
            unlock_at = cls._locked_until.get(client_ip, 0.0)
            if unlock_at > now:
                return False, round(unlock_at - now, 1)
            elif unlock_at > 0.0:
                del cls._locked_until[client_ip]
                cls._failed_attempts.pop(client_ip, None)
            return True, 0.0

    @classmethod
    def record_auth_failure(cls, client_ip: str) -> bool:
        """
        Ghi nhận 1 lần xác thực thất bại cho IP.
        Trả về True nếu IP này vừa chạm ngưỡng khóa (Lockout).
        """
        if not client_ip:
            return False
        now = time.time()
        with cls._lock:
            attempts = cls._failed_attempts.get(client_ip, [])
            cutoff = now - AUTH_WINDOW_SEC
            attempts = [t for t in attempts if t >= cutoff]
            attempts.append(now)
            cls._failed_attempts[client_ip] = attempts
            if len(attempts) >= AUTH_MAX_FAILURES:
                cls._locked_until[client_ip] = now + AUTH_LOCKOUT_SEC
                return True
            return False

    @classmethod
    def record_auth_success(cls, client_ip: str):
        """Xóa bộ đếm lỗi khi thiết bị xác thực thành công."""
        if not client_ip:
            return
        with cls._lock:
            cls._failed_attempts.pop(client_ip, None)
            cls._locked_until.pop(client_ip, None)

    @classmethod
    def reset_rate_limits(cls):
        """Đặt lại bộ đếm Rate Limit (dùng khi khởi tạo lại hoặc chạy kiểm thử)."""
        with cls._lock:
            cls._failed_attempts.clear()
            cls._locked_until.clear()
