"""
===================================================================================
ĐIỂM KHỞI CHẠY CHÍNH HỆ THỐNG AGV_VER2 WMS SERVER (ENTRY POINT V2.1)
KIẾN TRÚC MODULE HÓA: ZERO-DEPENDENCY, KHỞI CHẠY TỨC THÌ 100% OFFLINE
===================================================================================
"""

import sys
import socket
from pathlib import Path
from http.server import ThreadingHTTPServer

class GracefulThreadingHTTPServer(ThreadingHTTPServer):
    """
    HTTP Server chịu lỗi cao (Fault-tolerant):
    Tự động lọc và bỏ qua các lỗi hủy kết nối tự nhiên từ trình duyệt Web (WinError 10053, 10054, BrokenPipe)
    giúp màn hình console luôn sạch đẹp và ổn định tuyệt đối trong môi trường công nghiệp.
    """
    def handle_error(self, request, client_address):
        err_type = sys.exc_info()[0]
        if err_type in (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, socket.error):
            return
        super().handle_error(request, client_address)

# Thêm đường dẫn project vào sys.path
SERVER_DIR = Path(__file__).resolve().parent
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

# Import các tầng kiến trúc
from app.config import HOST, PORT, DB_PATH, MDNS_HOSTNAME, SECURITY_ENABLED
from database.seed_data import seed_database
from fsm.fsm_manager import WMSWorkflowManager
from app.services.ai_service import AIProductClassifierService
from app.services.mdns_service import MDNSService
from app.controllers.camera_controller import CameraController
from app.controllers.agv_controller import AGVController
from app.controllers.wms_controller import WMSController
from app.router import MainHTTPHandler

def main():
    print("=" * 75)
    print("      AGV_VER2 - WMS DIGITAL TWIN & SMART DISPATCHING SERVER (V2.1)")
    print("=" * 75)

    # 1. Khởi tạo Cơ sở dữ liệu (giữ nguyên cấu hình sản phẩm, trạm & tồn kho nếu đã tồn tại)
    print(">> [1/4] Đang kiểm tra & khởi tạo Cơ sở dữ liệu SQLite...")
    seed_database(DB_PATH, reset=False)

    # 2. Khởi tạo Bộ điều phối Workflow FSM & Dịch vụ AI
    print(">> [2/4] Đang nạp Bộ điều phối Workflow & Dịch vụ Trí tuệ Nhân tạo AI...")
    workflow_manager = WMSWorkflowManager(DB_PATH, agv_id="AGV_Ver2")
    ai_service = AIProductClassifierService(workflow_manager=workflow_manager)

    # 3. Khởi tạo các Controllers & Tiêm phụ thuộc (Dependency Injection)
    print(">> [3/4] Đang kết nối Controllers & Tầng điều phối HTTP Router...")
    cam_controller = CameraController(workflow_manager, ai_service)
    agv_controller = AGVController(workflow_manager)
    wms_controller = WMSController(workflow_manager, agv_controller=agv_controller)

    # Liên kết phụ thuộc chéo cho điều khiển tự động Shot 1 & Shot 2
    workflow_manager.camera_controller = cam_controller
    agv_controller.camera_controller = cam_controller
    cam_controller.agv_controller = agv_controller

    MainHTTPHandler.agv_controller = agv_controller
    MainHTTPHandler.camera_controller = cam_controller
    MainHTTPHandler.wms_controller = wms_controller

    # 4. Khởi chạy Dịch vụ Tên miền Cục bộ mDNS (RFC 6762)
    print(">> [4/4] Đang kích hoạt Dịch vụ Tên miền Cục bộ mDNS (agv-wms.local)...")
    mdns_service = MDNSService(hostname=MDNS_HOSTNAME, port=PORT)
    mdns_service.start()

    # 5. Khởi chạy HTTP Server đa luồng chịu lỗi cao (Graceful Multi-threaded Server)
    server = GracefulThreadingHTTPServer((HOST, PORT), MainHTTPHandler)

    print("\n" + "=" * 75)
    print(f"🚀 SERVER CHUẨN CÔNG NGHIỆP ĐÃ SẴN SÀNG TẠI CỔNG {PORT}!")
    print(f"   [Phương thức kết nối]     : TÊN MIỀN CỤC BỘ mDNS (RFC 6762) + TOKEN RBAC")
    print(f"   [Địa chỉ kết nối duy nhất]: http://{MDNS_HOSTNAME}.local:{PORT}")
    print(f"   [Bảo Mật Xác Thực]        : {'BẬT (Token Guard RBAC + HMAC-SHA256)' if SECURITY_ENABLED else 'TẮT'}")
    print("=" * 75)
    print(f"👉 ESP32, Android và Web kết nối trực tiếp qua: http://{MDNS_HOSTNAME}.local:{PORT}")
    print("👉 Nhấn Ctrl + C để dừng máy chủ an toàn.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n>> Đang tắt máy chủ...")
        mdns_service.stop()
        server.server_close()
        print(">> Đã tắt máy chủ thành công.")

if __name__ == "__main__":
    main()
