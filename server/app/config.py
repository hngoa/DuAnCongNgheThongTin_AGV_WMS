"""
===================================================================================
CẤU HÌNH HỆ THỐNG AGV_VER2 WMS SERVER (CONFIG V2.1)
===================================================================================
"""

from pathlib import Path
import os
import socket

# Đường dẫn thư mục
APP_DIR = Path(__file__).resolve().parent
SERVER_DIR = APP_DIR.parent
STATIC_DIR = APP_DIR / "static"
TEMPLATES_DIR = APP_DIR / "templates"
UPLOAD_DIR = SERVER_DIR / "uploads"
UPLOAD_DIR.mkdir(exist_ok=True)

DB_PATH = SERVER_DIR / "database" / "wms_database.db"

# Cấu hình Mạng & Tên Miền mDNS Cục Bộ (RFC 6762)
HOST = os.getenv("AGV_SERVER_HOST", "0.0.0.0")
PORT = int(os.getenv("AGV_SERVER_PORT", "5000"))
MDNS_HOSTNAME = os.getenv("AGV_MDNS_HOSTNAME", "agv-wms")  # Cho phép gọi trực tiếp: http://agv-wms.local:5000

# ===================================================================================
# CẤU HÌNH BẢO MẬT, XÁC THỰC THIẾT BỊ (RBAC) & CHỮ KÝ SỐ HMAC-SHA256
# Hỗ trợ nạp từ biến môi trường (Environment Variables) để không lộ khóa khi triển khai
# ===================================================================================
SECURITY_ENABLED = os.getenv("AGV_SECURITY_ENABLED", "true").strip().lower() not in ("0", "false", "no")

# Token cấp riêng cho Xe AGV (ESP32-S3): Quyền gửi Telemetry & nhận lệnh
AGV_DEVICE_TOKEN = os.getenv("AGV_DEVICE_TOKEN", "agv_sec_robot_v2_7f8a9b")

# Token cấp riêng cho Camera AI (Android PhoneCamera): Quyền upload ảnh & phân loại
CAMERA_DEVICE_TOKEN = os.getenv("CAMERA_DEVICE_TOKEN", "agv_sec_camera_v2_3e4d5c")

# Token Quản trị viên / Bảng điều khiển Web SCADA Dashboard: Toàn quyền hệ thống
ADMIN_API_TOKEN = os.getenv("ADMIN_API_TOKEN", "agv_sec_admin_v2_1a2b3c")

# Khóa bí mật chia sẻ (Shared Secret) dùng cho ký số HMAC-SHA256 lệnh điều khiển & phiên Web
AGV_HMAC_SECRET = os.getenv("AGV_HMAC_SECRET", "agv_hmac_shared_key_v2_9d8e7f6a5b4c")

# Giới hạn kích thước gói tin HTTP POST (Chống tấn công tràn bộ nhớ RAM / DoS)
MAX_JSON_BODY_BYTES = int(os.getenv("AGV_MAX_JSON_BYTES", str(64 * 1024)))          # Tối đa 64 KB cho JSON API
MAX_UPLOAD_BODY_BYTES = int(os.getenv("AGV_MAX_UPLOAD_BYTES", str(10 * 1024 * 1024))) # Tối đa 10 MB cho ảnh Camera

# Cấu hình chống dò quét mật khẩu / Token Brute-Force theo IP
AUTH_MAX_FAILURES = int(os.getenv("AGV_AUTH_MAX_FAILURES", "15"))
AUTH_WINDOW_SEC = float(os.getenv("AGV_AUTH_WINDOW_SEC", "60.0"))
AUTH_LOCKOUT_SEC = float(os.getenv("AGV_AUTH_LOCKOUT_SEC", "30.0"))

# Cấu hình Bản đồ mặc định
DEFAULT_MAP_ID = "SABAN_154_151"
DEFAULT_AGV_ID = "AGV_Ver2"
AGV_WATCHDOG_TIMEOUT_SEC = 6.0

# Ngưỡng an toàn điện áp pin LiPo 3S
BATTERY_THRESHOLDS = {
    "NORMAL": 11.2,
    "LOW": 10.5,
    "CRITICAL": 9.9
}

def get_local_ips():
    """Lấy danh sách các IP LAN nội bộ (ưu tiên Wi-Fi thực tế 192.168/10/172 trước VPN)"""
    ips = []
    try:
        hostname = socket.gethostname()
        all_ips = socket.gethostbyname_ex(hostname)[2]
        # Ưu tiên các dải IP Wi-Fi gia đình / văn phòng phổ biến
        lan_ips = [ip for ip in all_ips if ip.startswith(("192.168.", "10.", "172."))]
        other_ips = [ip for ip in all_ips if not ip.startswith("127.") and ip not in lan_ips]
        ips = lan_ips + other_ips
    except Exception:
        pass
    return ips or ["127.0.0.1"]
