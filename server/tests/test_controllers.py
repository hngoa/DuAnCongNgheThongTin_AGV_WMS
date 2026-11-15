"""
===================================================================================
UNIT TEST KIỂM THỬ TRỰC TIẾP TẦNG CONTROLLER & ROUTER LOGIC (V2.1)
KIỂM TRA MULTIPART PARSER, JSON TELEMETRY, ĐIỀU KHIỂN & CSV EXPORT
===================================================================================
"""

import sys
import json
import time
from pathlib import Path

SERVER_DIR = Path(__file__).resolve().parent.parent
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from app.config import DB_PATH, DEFAULT_MAP_ID
from database.seed_data import seed_database
from fsm.fsm_manager import WMSWorkflowManager
from app.services.ai_service import AIProductClassifierService
from app.controllers.camera_controller import CameraController
from app.controllers.agv_controller import AGVController
from app.controllers.wms_controller import WMSController

def run_controller_tests():
    print("=" * 75)
    print("🚀 BẮT ĐẦU KIỂM THỬ TẦNG CONTROLLERS & ENDPOINTS LOGIC (V2.1)")
    print("=" * 75)

    test_db = SERVER_DIR / "database" / "test_ctrl_wms.db"
    if test_db.exists():
        test_db.unlink()

    # 1. Khởi tạo Database & Services
    print(">> [BƯỚC 1]: Nạp Database & Services...")
    seed_database(test_db)
    workflow_manager = WMSWorkflowManager(test_db, agv_id="AGV_Ver2")
    ai_service = AIProductClassifierService()

    cam_ctrl = CameraController(workflow_manager, ai_service)
    agv_ctrl = AGVController(workflow_manager)
    wms_ctrl = WMSController(workflow_manager)

    # 2. Kiểm thử WMS Controller
    print("\n>> [BƯỚC 2]: Kiểm thử WMS Controller (Map, Inventory, Tasks)...")
    # (a) Get Map
    code, map_res = wms_ctrl.get_map_data(DEFAULT_MAP_ID)
    assert code == 200 and len(map_res["nodes"]) == 4 and len(map_res["segments"]) == 8
    print(f"   ✅ [WMS MAP] Trả về 200 OK: {len(map_res['nodes'])} Nodes, {len(map_res['segments'])} Segments")

    # (b) Get Inventory
    code, inv_res = wms_ctrl.get_inventory(DEFAULT_MAP_ID)
    assert code == 200 and "B" in inv_res["inventory"]
    stock_b = inv_res["inventory"]["B"]["capacity"]["current_stock"]
    print(f"   ✅ [WMS INVENTORY] Trả về 200 OK: Kho B đang có {stock_b} kiện hàng đa chủng loại")

    # (c) Get Products
    code, prod_res = wms_ctrl.get_products()
    assert code == 200 and len(prod_res["products"]) >= 3
    print(f"   ✅ [WMS PRODUCTS] Trả về 200 OK: {[p['product_code'] for p in prod_res['products']]}")

    # 3. Kiểm thử Camera Controller (Upload Multipart từ App Android)
    print("\n>> [BƯỚC 3]: Kiểm thử Camera Controller (Giả lập upload ảnh từ PhoneCamera)...")
    # Giả lập AGV kết nối và gửi telemetry chu kỳ đầu tiên tại Trạm A
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "dist": 0.0, "speed": 0.0,
        "line_mask": 4, "line_err": 0.0, "ticks_l": 0, "ticks_r": 0,
        "pwm_l": 0, "pwm_r": 0, "voltage": 12.4, "current": 0.2,
        "power": 2.48, "energy_mah": 0.1, "state": "AT_STATION",
        "ack_id": 0, "ina_ok": True
    })

    boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW"
    content_type = f"multipart/form-data; boundary={boundary}"
    
    # Tạo body multipart giả lập chụp ảnh linh kiện Điện tử
    fake_jpeg = b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb" + b"\x00" * 50
    multipart_body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="agv_id"\r\n\r\n'
        f"AGV_Ver2\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"A\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"PKG_ELEC\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"CAPTURE\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="phone_cam.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + fake_jpeg + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, upload_res = cam_ctrl.handle_upload(content_type, multipart_body)
    assert code == 200 and upload_res["status"] == "success"
    task_id = upload_res["dispatch"]["task_id"]
    target_node = upload_res["dispatch"]["target_node"]
    cmd_obj = upload_res["dispatch"]["command"]
    assert target_node == "B"
    print(f"   ✅ [CAMERA UPLOAD] Đã nhận ảnh, AI phân loại PKG_ELEC -> Phân bổ Kho {target_node} (Task #{task_id})")
    print(f"   ✅ [LỆNH ĐIỀU PHỐI] Lệnh đã nạp vào hàng đợi: #{cmd_obj['cmd_id']} ({cmd_obj['cmd']})")

    # 4. Kiểm thử AGV Controller (Telemetry từ ESP32-S3)
    print("\n>> [BƯỚC 4]: Kiểm thử AGV Controller (Nhận Telemetry chu kỳ 100ms)...")
    # (a) Gói 1: ESP32 đang bám line và gửi ACK cmd_id
    telemetry_pkg_1 = {
        "x": 0.00, "y": 0.50, "yaw": 90.0, "dist": 0.50, "speed": 22.0,
        "line_mask": 4, "line_err": 0.0, "ticks_l": 520, "ticks_r": 518,
        "pwm_l": 190, "pwm_r": 190, "voltage": 12.35, "current": 0.88,
        "power": 10.86, "energy_mah": 5.4, "state": "LINE_FOLLOWING",
        "ack_id": cmd_obj["cmd_id"], "ina_ok": True
    }
    code, t_res_1 = agv_ctrl.handle_telemetry(telemetry_pkg_1)
    assert code == 200 and t_res_1["status"] == "ok"
    print(f"   ✅ [TELEMETRY 1] ESP32 gửi ACK #{cmd_obj['cmd_id']} -> agv_state = {t_res_1['agv_state']}")

    # (b) Gói 2: ESP32 đến trạm B an toàn (AT_STATION)
    telemetry_pkg_2 = {
        "x": 0.00, "y": 1.51, "yaw": 90.0, "dist": 1.51, "speed": 0.0,
        "line_mask": 31, "line_err": 0.0, "voltage": 12.25, "current": 0.25,
        "power": 3.06, "energy_mah": 12.8, "state": "AT_STATION",
        "ack_id": cmd_obj["cmd_id"], "ina_ok": True
    }
    code, t_res_2 = agv_ctrl.handle_telemetry(telemetry_pkg_2)
    assert code == 200
    print(f"   ✅ [TELEMETRY 2] ESP32 chạm trạm B -> agv_state = {t_res_2['agv_state']}, bắt đầu đếm dỡ hàng")

    # Giả lập hoàn tất thời gian dỡ hàng (Dwell time)
    workflow_manager.active_unloading_tasks[task_id] = 0.0
    agv_ctrl.handle_telemetry(telemetry_pkg_2)
    task_done = workflow_manager.task_repo.get_by_id(task_id)
    assert task_done["status"] == "COMPLETED"
    print(f"   ✅ [TASK COMPLETE] Task #{task_id} đã hoàn tất dỡ hàng -> Trạng thái: {task_done['status']}")

    # 5. Kiểm thử AGV Controller (Lệnh điều khiển thủ công từ Web & Phát lệnh Camera từ xa)
    print("\n>> [BƯỚC 5]: Kiểm thử AGV Controller (Lệnh thủ công & Lệnh chụp Camera từ xa)...")
    agv_ctrl.camera_controller = cam_ctrl
    code, cam_cmd_res = agv_ctrl.handle_command({"action": "capture_now", "station": "A"})
    assert code == 200 and cam_cmd_res["action"] == "capture_now"
    assert cam_ctrl.pending_cam_cmd == "CAPTURE_NOW"
    assert cam_cmd_res["cam_cmd_id"] > 0
    print(f"   ✅ [REMOTE CAM COMMAND] Phát lệnh chụp khay hàng #{cam_cmd_res['cam_cmd_id']} xuống Camera thành công")

    # Giả lập điện thoại ACK lệnh
    cam_ctrl.ack_command(cam_cmd_res["cam_cmd_id"])
    assert cam_ctrl.pending_cam_cmd is None
    print(f"   ✅ [CAMERA ACK] Xác nhận điện thoại đã nhận và thực thi xong lệnh #{cam_cmd_res['cam_cmd_id']}")

    # Kiểm thử Cơ chế Tự động Kích hoạt Shot 1 tại Trạm A khi khởi động
    cam_ctrl.record_poll()
    agv_ctrl.handle_telemetry({"station": "A", "state": "IDLE", "speed": 0.0, "x": 0.0, "y": 0.0, "voltage": 12.4})
    # Giả lập mốc thời gian khởi động ổn định 3.0s
    now_ts = time.time()
    cam_ctrl.first_poll_time = now_ts - 3.0
    agv_ctrl.first_seen_time = now_ts - 3.0
    agv_ctrl.last_seen_time = now_ts - 0.1
    cam_ctrl.last_shot1_auto_trigger = 0.0
    auto_triggered = cam_ctrl.check_auto_inbound_capture(agv_ctrl)
    assert auto_triggered is True, "Lỗi: check_auto_inbound_capture phải trả về True khi cả AGV và Camera sẵn sàng tại Trạm A"
    assert cam_ctrl.pending_cam_cmd == "CAPTURE_NOW", f"Lỗi: Lệnh phải là CAPTURE_NOW, thực tế: {cam_ctrl.pending_cam_cmd}"
    assert cam_ctrl.pending_cam_metadata.get("station") == "A"
    print(f"   ✅ [AUTO SHOT 1 TEST] Tự động phát lệnh chụp #{cam_ctrl.cam_cmd_id} ({cam_ctrl.pending_cam_cmd}) khi khởi động tại Trạm A thành công!")
    cam_ctrl.ack_command(cam_ctrl.cam_cmd_id)

    code, cmd_res = agv_ctrl.handle_command({"action": "go_to", "station": "C"})
    assert code == 200 and cmd_res["cmd"] == "GO_TO:C"
    print(f"   ✅ [WEB COMMAND] Phát lệnh 'GO_TO:C' thành công (cmd_id: #{cmd_res['cmd_id']})")

    # 6. Kiểm thử Composite State cho Web Dashboard
    print("\n>> [BƯỚC 6]: Kiểm thử Tổng hợp Trạng thái cho Web Dashboard (GET /api/state)...")
    cam_info = cam_ctrl.get_latest_info()
    state = agv_ctrl.get_dashboard_state(cam_info)
    assert state["connected"] is True
    assert state["odom_y"] in (0.0, 1.51)
    assert state["voltage"] > 10.0
    assert len(state["trail"]) >= 2
    print(f"   ✅ [DASHBOARD STATE] Tổng hợp đầy đủ: Odom({state['odom_x']}, {state['odom_y']}), Pin: {state['voltage']}V, Trail: {len(state['trail'])} điểm")

    # 7. Kiểm thử Xuất file CSV báo cáo đo kiểm
    print("\n>> [BƯỚC 7]: Kiểm thử Xuất nhật ký đo kiểm CSV (GET /api/export_csv)...")
    code, filename, csv_bytes = wms_ctrl.export_csv()
    assert code == 200 and len(csv_bytes) > 0
    print(f"   ✅ [CSV EXPORT] Tạo thành công file CSV: '{filename}' ({len(csv_bytes)} bytes)")

    # Dọn dẹp Database kiểm thử để không lưu data ảo
    if test_db.exists():
        try:
            test_db.unlink()
        except Exception:
            pass

    print("\n" + "=" * 75)
    print("🎉 TẤT CẢ 7 BÀI TEST CONTROLLERS ĐÃ THÀNH CÔNG VƯỢT TRỘI (100% PASS)!")
    print("=" * 75)

if __name__ == "__main__":
    run_controller_tests()
