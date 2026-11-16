"""
===================================================================================
CẤU HÌNH HỆ THỐNG AGV_VER2 WMS SERVER (CONFIG V2.1)
===================================================================================
"""

from pathlib import Path
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
HOST = "0.0.0.0"
PORT = 5000
MDNS_HOSTNAME = "agv-wms"  # Cho phép gọi trực tiếp: http://agv-wms.local:5000

# ===================================================================================
# CẤU HÌNH BẢO MẬT & XÁC THỰC THIẾT BỊ (DEVICE AUTHENTICATION & RBAC)
# ===================================================================================
SECURITY_ENABLED = True

# Token cấp riêng cho Xe AGV (ESP32-S3): Quyền gửi Telemetry & nhận lệnh
AGV_DEVICE_TOKEN = "agv_sec_robot_v2_7f8a9b"

# Token cấp riêng cho Camera AI (Android PhoneCamera): Quyền upload ảnh & phân loại
CAMERA_DEVICE_TOKEN = "agv_sec_camera_v2_3e4d5c"

# Token Quản trị viên / Bảng điều khiển Web SCADA Dashboard: Toàn quyền hệ thống
ADMIN_API_TOKEN = "agv_sec_admin_v2_1a2b3c"

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
