"""
===================================================================================
KIỂM THỬ QUY TRÌNH 2-SHOT AI & TỰ ĐỘNG CHỌN KHO GẦN NHẤT (AGV_VER2)
===================================================================================
1. SHOT 1: NHẬN DIỆN 3 LOẠI HÀNG (PKG_ELEC, PKG_MECH, PKG_TOOL)
   -> TỰ ĐỘNG CHỌN KHO GẦN NHẤT BẰNG DIJKSTRA CÒN CHỖ CHỨA
   -> CHẶN XE NẾU KHAY TRỐNG (NO_OBJECT)
2. SHOT 2: XÁC NHẬN DỠ HÀNG BẰNG THỊ GIÁC TẠI TRẠM ĐÍCH
   -> NẾU CÒN HÀNG: XE DỪNG CHỜ, KHÔNG CHUYỂN TRẠM, CHƯA TĂNG KHO
   -> KHI CHẮC CHẮN ĐÃ DỠ (NO_OBJECT): TĂNG TỒN KHO (+1), PHÁT LỆNH SANG TRẠM TIẾP
===================================================================================
"""

import sys
from pathlib import Path

SERVER_DIR = Path(__file__).resolve().parent.parent
DB_DIR = SERVER_DIR / "database"
for p in (SERVER_DIR, DB_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from seed_data import seed_database
from app.config import UPLOAD_DIR
from fsm.fsm_manager import WMSWorkflowManager
from fsm.fsm_definitions import TaskState
from app.services.ai_service import AIProductClassifierService
from app.controllers.camera_controller import CameraController
from app.controllers.agv_controller import AGVController

TEST_DB = SERVER_DIR / "database" / "test_two_shot.db"

def run_tests():
    print("=" * 75)
    print("  BẮT ĐẦU TEST TOÀN DIỆN: 2-SHOT AI & TỰ ĐỘNG CHỌN KHO GẦN NHẤT")
    print("=" * 75)

    existing_uploads = set(UPLOAD_DIR.glob("*.jpg")) if UPLOAD_DIR.is_dir() else set()
    if TEST_DB.exists():
        try:
            TEST_DB.unlink()
        except Exception:
            pass

    seed_database(TEST_DB)
    active_map_id = "SABAN_154_151"

    wf_manager = WMSWorkflowManager(TEST_DB, agv_id="AGV_Ver2")
    ai_service = AIProductClassifierService()
    cam_ctrl = CameraController(wf_manager, ai_service)
    agv_ctrl = AGVController(wf_manager)
    wf_manager.camera_controller = cam_ctrl
    agv_ctrl.camera_controller = cam_ctrl

    # Đặt AGV ở trạng thái IDLE sẵn sàng nhận việc
    wf_manager.fleet_repo.update_telemetry(
        agv_id="AGV_Ver2", x=0.0, y=0.0, yaw=90.0, speed=0.0,
        voltage=12.4, current=0.25, battery_pct=95.0, status="IDLE"
    )

    # -------------------------------------------------------------------------
    # TEST 1: TỰ ĐỘNG CHỌN KHO GẦN NHẤT CHO 3 LOẠI HÀNG HOÁ
    # -------------------------------------------------------------------------
    print("\n>> [TEST 1]: Kiểm tra Thuật toán Chọn Kho Gần Nhất (Nearest Warehouse Allocation)...")
    router = wf_manager.get_router(active_map_id)

    # 1.1 Hàng PKG_ELEC (Hộp chữ nhật) -> Kho B (Khoảng cách 1.51m)
    alloc_elec = wf_manager.alloc_repo.allocate_nearest_target_node(
        active_map_id, "PKG_ELEC", "ELECTRONIC", from_node="A", router=router
    )
    assert alloc_elec is not None, "Lỗi: Không tìm được kho cho PKG_ELEC"
    assert alloc_elec["target_node_id"] == "B", f"Lỗi: PKG_ELEC phải tới B, thực tế: {alloc_elec['target_node_id']}"
    assert alloc_elec["distance_m"] == 1.51
    print(f"   ✅ [PKG_ELEC - Hộp chữ nhật]: Chọn kho gần nhất {alloc_elec['target_node_id']} (Khoảng cách: {alloc_elec['distance_m']}m)")

    # 1.2 Hàng PKG_MECH (Trụ tròn) -> Kho C (Khoảng cách 3.05m)
    alloc_mech = wf_manager.alloc_repo.allocate_nearest_target_node(
        active_map_id, "PKG_MECH", "MECHANICAL", from_node="A", router=router
    )
    assert alloc_mech is not None, "Lỗi: Không tìm được kho cho PKG_MECH"
    assert alloc_mech["target_node_id"] == "C", f"Lỗi: PKG_MECH phải tới C, thực tế: {alloc_mech['target_node_id']}"
    print(f"   ✅ [PKG_MECH - Trụ tròn]: Chọn kho gần nhất {alloc_mech['target_node_id']} (Khoảng cách: {alloc_mech['distance_m']}m)")

    # 1.3 Hàng PKG_TOOL (Lăng trụ tam giác) -> Kho D (Khoảng cách 1.54m qua SEG_AD)
    alloc_tool = wf_manager.alloc_repo.allocate_nearest_target_node(
        active_map_id, "PKG_TOOL", "TOOL", from_node="A", router=router
    )
    assert alloc_tool is not None, "Lỗi: Không tìm được kho cho PKG_TOOL"
    assert alloc_tool["target_node_id"] == "D", f"Lỗi: PKG_TOOL phải tới D, thực tế: {alloc_tool['target_node_id']}"
    print(f"   ✅ [PKG_TOOL - Lăng trụ tam giác]: Chọn kho gần nhất {alloc_tool['target_node_id']} (Khoảng cách: {alloc_tool['distance_m']}m)")

    # -------------------------------------------------------------------------
    # TEST 2: KHO GẦN NHẤT KHI CÓ NHIỀU KHO HOẶC KHO CHÍNH BỊ ĐẦY
    # -------------------------------------------------------------------------
    print("\n>> [TEST 2]: Kiểm tra Chuyển Kho Dự Phòng Gần Nhất khi Kho Chính Đầy Sức Chứa...")
    # Giả lập làm đầy Kho B (max 30 kiện)
    wf_manager.inv_repo.update_stock(active_map_id, "B", "PKG_ELEC", delta=30)
    cap_b = wf_manager.inv_repo.get_node_capacity(active_map_id, "B")
    assert cap_b["has_space"] is False, "Lỗi: Kho B phải báo đầy"

    # Khi Kho B đầy, hàng PKG_ELEC tự động chuyển sang kho dự phòng còn chỗ gần nhất (Kho D qua rule fallback '*')
    alloc_elec_full = wf_manager.alloc_repo.allocate_nearest_target_node(
        active_map_id, "PKG_ELEC", "ELECTRONIC", from_node="A", router=router
    )
    assert alloc_elec_full["target_node_id"] == "D", f"Lỗi: Khi B đầy, phải chuyển sang D, thực tế: {alloc_elec_full['target_node_id']}"
    print(f"   ✅ Khi Kho B đầy (30/30), hệ thống tự động định tuyến PKG_ELEC sang Kho dự phòng gần nhất: {alloc_elec_full['target_node_id']} (1.54m)")

    # Trả lại Kho B trống
    wf_manager.inv_repo.update_stock(active_map_id, "B", "PKG_ELEC", delta=-30)

    # -------------------------------------------------------------------------
    # TEST 3: SHOT 1 CỔNG AN TOÀN - CHẶN XE KHI KHAY TRỐNG (NO_OBJECT)
    # -------------------------------------------------------------------------
    print("\n>> [TEST 3]: Kiểm tra Shot 1 Cổng An Toàn: Chặn xe xuất bến nếu khay trống...")
    boundary = "----TestBoundary123"
    content_type = f"multipart/form-data; boundary={boundary}"
    empty_tray_body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"A\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"NONE\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"CAPTURE\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="empty.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_empty = cam_ctrl.handle_upload(content_type, empty_tray_body)
    assert resp_empty["status"] == "warning", "Lỗi: Khay trống phải trả về warning"
    assert wf_manager.agv_fsm.pending_command is None, "Lỗi: Không được phát lệnh cho xe khi khay trống"
    print(f"   ✅ [SHOT 1 AN TOÀN]: Khay trống (NO_OBJECT) -> Hệ thống từ chối tạo task, xe AGV giữ nguyên vị trí!")

    # -------------------------------------------------------------------------
    # TEST 4: LUỒNG HOÀN CHỈNH: SHOT 1 -> ĐIỀU PHỐI -> DI CHUYỂN -> SHOT 2
    # -------------------------------------------------------------------------
    print("\n>> [TEST 4]: Luồng Toàn Diện: Shot 1 Nhận Diện -> Xe Tới Kho B -> Shot 2 Kiểm Định Dỡ Hàng...")
    # Bước 4.1: Shot 1 chụp kiện hàng PKG_ELEC tại Trạm A
    box_body = (
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
        f'Content-Disposition: form-data; name="image"; filename="box.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_shot1 = cam_ctrl.handle_upload(content_type, box_body)
    assert code == 200 and resp_shot1["status"] == "success"
    task_id = resp_shot1["dispatch"]["task_id"]
    target_node = resp_shot1["dispatch"]["target_node"]
    cmd_1 = resp_shot1["dispatch"]["command"]
    assert target_node == "B"
    print(f"   ✅ [BƯỚC 4.1]: Shot 1 nhận diện PKG_ELEC -> Cấp Kho B (1.51m) -> Lệnh #{cmd_1['cmd_id']}: {cmd_1['cmd']}")

    # Bước 4.2: ESP32 gửi ACK và di chuyển đến Trạm B
    agv_ctrl.handle_telemetry({"state": "LINE_FOLLOWING", "ack_id": cmd_1["cmd_id"], "voltage": 12.3})
    agv_ctrl.handle_telemetry({"state": "AT_STATION", "ack_id": cmd_1["cmd_id"], "voltage": 12.2})
    task_at_station = wf_manager.task_repo.get_by_id(task_id)
    assert task_at_station["status"] == TaskState.DELIVERED.value
    assert cam_ctrl.pending_cam_cmd == "CAPTURE_NOW", f"Lỗi: Server chưa tự động phát lệnh CAPTURE_NOW (thực tế: {cam_ctrl.pending_cam_cmd})"
    print(f"   ✅ [BƯỚC 4.2]: Xe AGV đã tới Trạm B an toàn, dừng phanh chờ dỡ hàng (Task #{task_id}: {task_at_station['status']})")
    print(f"   ✅ [SHOT 2 AUTO TRIGGER]: Server đã tự động phát lệnh CAPTURE_NOW (#{cam_ctrl.cam_cmd_id}) xuống điện thoại khi xe chạm trạm B")

    # Bước 4.3: Shot 2 - KIỂM TRA KHI HÀNG VẪN CÒN TRÊN XE (CHƯA DỠ)
    print("   >> [BƯỚC 4.3]: Chụp Shot 2 khi hàng VẪN CÒN trên xe...")
    still_cargo_body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"B\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"PKG_ELEC\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"VERIFY_UNLOAD\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="still_cargo.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_still = cam_ctrl.handle_upload(content_type, still_cargo_body)
    assert resp_still["unload_verification"]["unloaded"] is False, "Lỗi: Hàng còn mà lại báo đã dỡ"
    task_still = wf_manager.task_repo.get_by_id(task_id)
    assert task_still["status"] == TaskState.DELIVERED.value, "Lỗi: Không được hoàn tất task khi hàng còn"
    inv_b_before = {i["product_code"]: i["quantity"] for i in wf_manager.inv_repo.get_node_inventory(active_map_id, "B")}
    assert inv_b_before.get("PKG_ELEC", 0) == 0, "Lỗi: Tồn kho không được tăng khi chưa dỡ hàng"
    print(f"      🛡️ [SHOT 2 CHẶN THÀNH CÔNG]: AI phát hiện hàng vẫn còn! {resp_still['message']}")
    print(f"      🛡️ Xe AGV tiếp tục đỗ giữ vị trí, tồn kho chưa tăng (Kho B = {inv_b_before.get('PKG_ELEC', 0)})")

    # Bước 4.4: Shot 2 - KIỂM TRA KHI HÀNG ĐÃ DỠ XONG (KHAY TRỐNG NO_OBJECT)
    print("   >> [BƯỚC 4.4]: Chụp Shot 2 khi hàng ĐÃ ĐƯỢC DỠ KHỎI XE (Khay trống)...")
    unloaded_body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"B\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"NONE\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"VERIFY_UNLOAD\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="empty_tray.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_unloaded = cam_ctrl.handle_upload(content_type, unloaded_body)
    assert resp_unloaded["unload_verification"]["unloaded"] is True, "Lỗi: Phải xác nhận dỡ hàng thành công"
    task_done = wf_manager.task_repo.get_by_id(task_id)
    assert task_done["status"] == TaskState.COMPLETED.value, "Lỗi: Task phải là COMPLETED"

    inv_b_after = {i["product_code"]: i["quantity"] for i in wf_manager.inv_repo.get_node_inventory(active_map_id, "B")}
    assert inv_b_after.get("PKG_ELEC", 0) == 1, "Lỗi: Tồn kho PKG_ELEC tại Kho B phải tăng thành 1"

    next_cmd = resp_unloaded["unload_verification"]["command"]
    assert next_cmd["cmd"] == "GO_TO:A", f"Lỗi: Sau khi dỡ xong phải về Trạm A, thực tế: {next_cmd['cmd']}"
    print(f"      🎉 [SHOT 2 XÁC NHẬN THÀNH CÔNG]: {resp_unloaded['message']}")
    print(f"      🎉 Tồn kho PKG_ELEC tại Kho B đã tăng thành: {inv_b_after['PKG_ELEC']} kiện (+1)")
    print(f"      🚀 AGV nhận lệnh chuyển sang trạm kế tiếp: #{next_cmd['cmd_id']} ({next_cmd['cmd']})")

    # -------------------------------------------------------------------------
    # TEST 5: ĐA SẢN PHẨM & TỰ ĐỘNG LẬP HÀNG ĐỢI MISSION ĐA TRẠM (MULTI-STOP MISSION QUEUE)
    # -------------------------------------------------------------------------
    print("\n>> [TEST 5]: Kiểm thử Đa Sản Phẩm & Tự Động Lập Hàng Đợi Mission Đa Trạm (TSP Dijkstra)...")
    # Reset xe về Trạm A
    wf_manager.fleet_repo.update_telemetry(
        agv_id="AGV_Ver2", x=0.0, y=0.0, yaw=90.0, speed=0.0,
        voltage=12.4, current=0.25, battery_pct=95.0, status="IDLE"
    )

    # 5.1 Chụp Shot 1 phát hiện cả 3 loại hàng trên xe: Điện tử (B), Cơ khí (C), Dụng cụ (D)
    multi_cargo_body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"A\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"PKG_ELEC,PKG_MECH,PKG_TOOL\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"CAPTURE\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="multi_items.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_multi = cam_ctrl.handle_upload(content_type, multi_cargo_body)
    assert code == 200 and resp_multi["status"] == "success"
    disp = resp_multi["dispatch"]
    assert disp["mode"] == "MISSION_QUEUE", "Lỗi: Phải kích hoạt chế độ MISSION_QUEUE"
    assert disp["total_missions"] == 4, f"Lỗi: Cần 3 chặng giao hàng + 1 chặng về A, thực tế: {disp['total_missions']}"
    assert disp["optimal_sequence"] == ["A", "B", "C", "D", "A"], f"Lỗi thứ tự tối ưu: {disp['optimal_sequence']}"
    print(f"   ✅ [SHOT 1 ĐA HÀNG HÓA]: Phát hiện 3 sản phẩm trên khay xe!")
    print(f"   ✅ [LẬP HÀNG ĐỢI MISSION TỐI ƯU]: {' -> '.join(disp['optimal_sequence'])} (Tổng cự ly: {disp['total_distance_m']}m)")
    print(f"   ✅ [MISSION 1 ACTIVE]: Lệnh #{disp['command']['cmd_id']} ({disp['command']['cmd']})")

    # 5.2 AGV chạy chặng 1 đến Trạm B (Dỡ PKG_ELEC)
    agv_ctrl.handle_telemetry({"state": "LINE_FOLLOWING", "ack_id": disp["command"]["cmd_id"], "voltage": 12.3})
    agv_ctrl.handle_telemetry({"state": "AT_STATION", "ack_id": disp["command"]["cmd_id"], "voltage": 12.2})

    # Tại Trạm B: Hàng PKG_ELEC được dỡ xuống (trên xe chỉ còn PKG_MECH và PKG_TOOL)
    shot2_b_body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"B\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"PKG_MECH,PKG_TOOL\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"VERIFY_UNLOAD\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="shot2_b.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_b = cam_ctrl.handle_upload(content_type, shot2_b_body)
    assert resp_b["unload_verification"]["unloaded"] is True
    assert resp_b["unload_verification"]["status"] == "MISSION_ADVANCED"
    assert resp_b["unload_verification"]["next_station"] == "C"
    cmd_c = resp_b["unload_verification"]["command"]
    assert cmd_c["cmd"] == "GO_TO:C"
    print(f"   ✅ [TRẠM B]: Dỡ PKG_ELEC thành công -> Tự động chuyển Mission 2: #{cmd_c['cmd_id']} ({cmd_c['cmd']})")

    # 5.3 AGV chạy chặng 2 đến Trạm C (Dỡ PKG_MECH)
    agv_ctrl.handle_telemetry({"state": "LINE_FOLLOWING", "ack_id": cmd_c["cmd_id"], "voltage": 12.3})
    agv_ctrl.handle_telemetry({"state": "AT_STATION", "ack_id": cmd_c["cmd_id"], "voltage": 12.2})

    # Tại Trạm C: Dỡ PKG_MECH xuống (trên xe chỉ còn PKG_TOOL)
    shot2_c_body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"C\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"PKG_TOOL\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"VERIFY_UNLOAD\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="shot2_c.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_c = cam_ctrl.handle_upload(content_type, shot2_c_body)
    assert resp_c["unload_verification"]["unloaded"] is True
    assert resp_c["unload_verification"]["next_station"] == "D"
    cmd_d = resp_c["unload_verification"]["command"]
    assert cmd_d["cmd"] == "GO_TO:D"
    print(f"   ✅ [TRẠM C]: Dỡ PKG_MECH thành công -> Tự động chuyển Mission 3: #{cmd_d['cmd_id']} ({cmd_d['cmd']})")

    # 5.4 AGV chạy chặng 3 đến Trạm D (Dỡ PKG_TOOL)
    agv_ctrl.handle_telemetry({"state": "LINE_FOLLOWING", "ack_id": cmd_d["cmd_id"], "voltage": 12.3})
    agv_ctrl.handle_telemetry({"state": "AT_STATION", "ack_id": cmd_d["cmd_id"], "voltage": 12.2})

    # Tại Trạm D: Dỡ PKG_TOOL xuống (trên xe hoàn toàn trống NO_OBJECT)
    shot2_d_body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"D\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"NONE\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"VERIFY_UNLOAD\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="shot2_d.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_d = cam_ctrl.handle_upload(content_type, shot2_d_body)
    assert resp_d["unload_verification"]["unloaded"] is True
    assert resp_d["unload_verification"]["next_station"] == "A"
    cmd_ret = resp_d["unload_verification"]["command"]
    assert cmd_ret["cmd"] == "GO_TO:A"
    print(f"   ✅ [TRẠM D]: Dỡ PKG_TOOL thành công (Khay trống 100%) -> Tự động kích hoạt Mission Return: #{cmd_ret['cmd_id']} ({cmd_ret['cmd']})")

    # 5.5 AGV chạy chặng về Trạm Tiếp Nhận A và cập nhật hoàn tất toàn bộ chu trình
    agv_ctrl.handle_telemetry({"state": "LINE_FOLLOWING", "ack_id": cmd_ret["cmd_id"], "station": "D", "voltage": 12.1})
    agv_ctrl.handle_telemetry({"state": "AT_STATION", "ack_id": cmd_ret["cmd_id"], "station": "A", "voltage": 12.0})

    assert wf_manager.active_mission is None, "Lỗi: Sau khi về Trạm A, active_mission phải là None!"
    assert wf_manager.agv_fsm.current_state.value == "IDLE", f"Lỗi: AGV phải ở trạng thái IDLE, thực tế: {wf_manager.agv_fsm.current_state.value}"
    unlocked_segs = [s for s in wf_manager.map_repo.get_segments(active_map_id) if s.get("locked_by_agv") == "AGV_Ver2"]
    assert len(unlocked_segs) == 0, f"Lỗi: Toàn bộ khóa đường phải được giải phóng khi về Trạm A! Còn khóa: {unlocked_segs}"
    print("   ✅ [TRẠM A - HOÀN TẤT CHU TRÌNH]: AGV đã về Trạm Tiếp Nhận A, giải phóng toàn bộ khóa đường, AGV chuyển sang IDLE sẵn sàng cho ca mới!")

    # =========================================================================
    # TEST 6: KIỂM THỬ CƠ CHẾ HẾT HÀNG SỚM (EARLY EMPTY TRAY RETURN)
    # Khay hàng gồm nhiều món nhưng bị dỡ sạch ngay tại Trạm B -> Xe tự động về Trạm A
    # =========================================================================
    print("\n>> [TEST 6]: Kiểm thử Cơ chế Hết hàng sớm (Early Empty Tray Return)...")
    # Giả lập phát hiện 2 sản phẩm (Kho B và Kho C)
    multi_items_2 = [
        {"product_code": "PKG_ELEC", "confidence": 0.95},
        {"product_code": "PKG_MECH", "confidence": 0.90}
    ]
    res_q2 = wf_manager.plan_and_dispatch_mission_queue(
        map_id=active_map_id,
        detected_items=multi_items_2,
        from_node="A"
    )
    cmd_b2 = res_q2["command"]
    assert cmd_b2["cmd"] == "GO_TO:B"
    print(f"   ✅ [LẬP QUEUE 2 CHẶNG]: A -> B -> C -> A. Bắt đầu Mission 1: {cmd_b2['cmd']}")

    # AGV chạy đến Trạm B
    agv_ctrl.handle_telemetry({"state": "LINE_FOLLOWING", "ack_id": cmd_b2["cmd_id"], "voltage": 12.2})
    agv_ctrl.handle_telemetry({"state": "AT_STATION", "ack_id": cmd_b2["cmd_id"], "station": "B", "voltage": 12.1})

    # Tại Trạm B: Người thao tác dỡ HẾT TOÀN BỘ HÀNG trên xe (khay hoàn toàn trống NO_OBJECT)
    shot2_empty_b = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="station"\r\n\r\n'
        f"B\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="product_code"\r\n\r\n'
        f"NONE\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="mode"\r\n\r\n'
        f"VERIFY_UNLOAD\r\n"
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="image"; filename="shot2_empty_b.jpg"\r\n'
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + (b"\xff\xd8\xff\xe0" + b"\x00" * 40) + f"\r\n--{boundary}--\r\n".encode("utf-8")

    code, resp_eb = cam_ctrl.handle_upload(content_type, shot2_empty_b)
    assert resp_eb["unload_verification"]["unloaded"] is True
    assert resp_eb["unload_verification"]["next_station"] == "A", f"Lỗi: Hết hàng sớm phải về A ngay, thực tế: {resp_eb['unload_verification']['next_station']}"
    cmd_ret_early = resp_eb["unload_verification"]["command"]
    assert cmd_ret_early["cmd"] == "GO_TO:A", f"Lỗi: Lệnh phải là GO_TO:A, thực tế: {cmd_ret_early['cmd']}"
    print(f"   ✅ [CƠ CHẾ HẾT HÀNG SỚM]: Dỡ sạch hàng tại Trạm B -> Server tự động hủy chặng C và phát lệnh #{cmd_ret_early['cmd_id']} ({cmd_ret_early['cmd']}) quay về Trạm A!")

    # AGV chạy từ B về A
    agv_ctrl.handle_telemetry({"state": "LINE_FOLLOWING", "ack_id": cmd_ret_early["cmd_id"], "station": "B", "voltage": 12.0})
    agv_ctrl.handle_telemetry({"state": "AT_STATION", "ack_id": cmd_ret_early["cmd_id"], "station": "A", "voltage": 11.9})

    assert wf_manager.active_mission is None
    assert wf_manager.agv_fsm.current_state.value == "IDLE"
    print("   ✅ [HOÀN TẤT VỀ TRẠM A]: Xe AGV đã về Trạm Tiếp Nhận A sau khi hết hàng sớm thành công!")

    if TEST_DB.exists():
        try:
            TEST_DB.unlink()
        except Exception:
            pass
    if UPLOAD_DIR.is_dir():
        for f in set(UPLOAD_DIR.glob("*.jpg")) - existing_uploads:
            try:
                f.unlink()
            except Exception:
                pass

    print("\n" + "=" * 75)
    print("🎉 TOÀN BỘ CÁC BÀI TEST 2-SHOT & CHỌN KHO GẦN NHẤT ĐÃ ĐẠT 100% PASS!")
    print("=" * 75)

if __name__ == "__main__":
    run_tests()
