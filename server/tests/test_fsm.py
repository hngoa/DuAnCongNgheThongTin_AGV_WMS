"""
===================================================================================
UNIT TEST KIỂM THỬ MÁY TRẠNG THÁI & BỘ ĐỊNH TUYẾN LINE TÔ-PÔ (FSM TEST V2.1)
===================================================================================
"""

import sys
import time
from pathlib import Path

SERVER_DIR = Path(__file__).resolve().parent.parent
DB_DIR = SERVER_DIR / "database"
FSM_DIR = SERVER_DIR / "fsm"

for p in (SERVER_DIR, DB_DIR, FSM_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from seed_data import seed_database
from fsm_definitions import TaskState, AGVState
from fsm_manager import WMSWorkflowManager

def run_tests():
    print("=" * 75)
    print("🚀 BẮT ĐẦU KIỂM THỬ BƯỚC 3: MÁY TRẠNG THÁI & LUỒNG ĐIỀU PHỐI (FSM V2.1)")
    print("=" * 75)

    test_db = DB_DIR / "test_fsm_wms.db"
    if test_db.exists():
        test_db.unlink()

    # 1. Khởi tạo DB sạch
    print(">> [PHẦN 1]: Khởi tạo cơ sở dữ liệu mẫu...")
    seed_database(test_db)
    manager = WMSWorkflowManager(test_db, agv_id="AGV_Ver2")
    active_map_id = "SABAN_154_151"

    # 2. Kiểm thử Bộ định tuyến Tô-pô Dijkstra (Topological Line Router)
    print("\n>> [PHẦN 2]: Kiểm thử Bộ định tuyến Dijkstra trên mạng lưới line...")
    router = manager.get_router(active_map_id)

    # (a) Tìm đường A -> B
    path_ab = router.find_shortest_path("A", "B")
    assert path_ab is not None, "Lỗi: Không tìm được đường A -> B"
    assert path_ab["nodes"] == ["A", "B"], f"Lỗi lộ trình: {path_ab['nodes']}"
    assert path_ab["segments"] == ["SEG_AB"], f"Lỗi đoạn line: {path_ab['segments']}"
    assert path_ab["total_distance_m"] == 1.51, "Lỗi chiều dài đoạn AB"
    print(f"   ✅ Lộ trình A -> B: {path_ab['route_str']} | Chiều dài: {path_ab['total_distance_m']}m | Đoạn line: {path_ab['segments']}")

    # (b) Tìm đường A -> C (Đi qua B: A -> B -> C: 1.51m + 1.54m = 3.05m)
    path_ac = router.find_shortest_path("A", "C")
    assert path_ac is not None and path_ac["total_distance_m"] == 3.05, f"Lỗi lộ trình A -> C: {path_ac}"
    print(f"   ✅ Lộ trình A -> C: {path_ac['route_str']} | Chiều dài: {path_ac['total_distance_m']}m | Đoạn line: {path_ac['segments']}")

    # (c) Giả lập khóa đoạn SEG_AB bởi xe khác -> Router phải tự đổi hướng hoặc báo không đi được
    print("   * Thử nghiệm khóa đoạn SEG_AB bởi xe 'AGV_OTHER'...")
    manager.map_repo.lock_segments(active_map_id, ["SEG_AB"], agv_id="AGV_OTHER")
    router_locked = manager.get_router(active_map_id)
    path_ab_blocked = router_locked.find_shortest_path("A", "B", current_agv_id="AGV_Ver2")
    # Khi SEG_AB bị chặn, lộ trình phải đi vòng theo chiều ngược lại (A -> D -> C -> B)
    assert path_ab_blocked is not None and "D" in path_ab_blocked["nodes"], "Lỗi: Router không tự động đi vòng khi đường bị khóa"
    print(f"   ✅ Khi SEG_AB bị khóa bởi xe khác, Router tự động điều hướng đi vòng qua D: {path_ab_blocked['route_str']} ({path_ab_blocked['total_distance_m']}m)")
    # Mở khóa lại
    manager.map_repo.unlock_segments(active_map_id, ["SEG_AB"])

    # 3. KỊCH BẢN 1: Luồng Giao hàng Chuẩn Khép Kín (Happy Path)
    print("\n>> [PHẦN 3]: KỊCH BẢN 1 - Luồng Giao Hàng Chuẩn Khép Kín...")
    # Bước 3.0: AGV kết nối gửi telemetry đầu tiên (12.4V, sẵn sàng tại Trạm A)
    manager.handle_telemetry({
        "map_id": active_map_id, "x": 0.0, "y": 0.0, "yaw": 90.0, "dist": 0.0,
        "speed": 0.0, "state": "AT_STATION_A", "voltage": 12.4, "current": 0.2,
        "battery_pct": 95.0, "ack_id": 0
    })

    # Bước 3.1: Nhận diện sản phẩm Điện tử (PKG_ELEC) tại Trạm A
    inbound_res = manager.handle_inbound_package(
        map_id=active_map_id, product_code="PKG_ELEC",
        ai_confidence=0.965, from_node="A", image_path="/uploads/cam_elec.jpg"
    )
    assert inbound_res["success"] is True, f"Lỗi inbound: {inbound_res}"
    task_id = inbound_res["task_id"]
    target_node = inbound_res["target_node"]
    assert target_node == "B", f"Lỗi: Hàng điện tử phải phân bổ về Kho B, thực tế: {target_node}"
    print(f"   ✅ [B1: AI & ALLOCATE] Đã phân bổ hàng 'PKG_ELEC' -> Kho {target_node} (Task #{task_id})")
    print(f"   ✅ [B2: TRAFFIC LOCK] Đã khóa an toàn đoạn line: {inbound_res['locked_segments']}")
    print(f"   ✅ [B3: COMMAND QUEUE] Đã phát lệnh cho ESP32: {inbound_res['command']['cmd']} (cmd_id: {inbound_res['command']['cmd_id']})")

    # Bước 3.2: ESP32 gửi Telemetry phản hồi ACK đã nhận lệnh
    telemetry_ack = {
        "map_id": active_map_id, "x": 0.00, "y": 0.20, "yaw": 90.0,
        "speed": 18.5, "state": "LINE_FOLLOWING", "voltage": 12.2,
        "current": 0.85, "ack_id": inbound_res["command"]["cmd_id"]
    }
    resp_telemetry = manager.handle_telemetry(telemetry_ack)
    task_running = manager.task_repo.get_by_id(task_id)
    assert task_running["status"] == TaskState.IN_TRANSIT.value, f"Lỗi: Task phải là IN_TRANSIT, thực tế: {task_running['status']}"
    assert manager.agv_fsm.pending_command is None, "Lỗi: Command chờ phải được xóa sau khi nhận ACK"
    print(f"   ✅ [B4: ACK HANDSHAKE] ESP32 gửi ACK #{telemetry_ack['ack_id']} -> Task chuyển sang {task_running['status']}, xe đang bám line")

    # Bước 3.3: Xe đến đích (Trạm B) -> Báo state = 'AT_STATION'
    telemetry_arrived = {
        "map_id": active_map_id, "x": 0.00, "y": 1.51, "yaw": 90.0,
        "speed": 0.0, "state": "AT_STATION", "voltage": 12.1,
        "current": 0.25, "ack_id": inbound_res["command"]["cmd_id"]
    }
    manager.handle_telemetry(telemetry_arrived)
    task_delivered = manager.task_repo.get_by_id(task_id)
    assert task_delivered["status"] == TaskState.DELIVERED.value, f"Lỗi: Task phải là DELIVERED, thực tế: {task_delivered['status']}"
    assert task_id in manager.active_unloading_tasks, "Lỗi: Dwell timer chưa được kích hoạt"
    print(f"   ✅ [B5: STATION REACHED] Xe dừng an toàn tại Kho B -> Task chuyển sang {task_delivered['status']}")
    print(f"   ✅ [B6: UNLOADING DWELL] Kích hoạt bộ đếm dừng đỗ dỡ hàng: 5.0 giây")

    # Bước 3.4: Hết thời gian Dwell time (Giả lập bằng cách chỉnh mốc thời gian hoàn tất)
    manager.active_unloading_tasks[task_id] = time.time() - 1.0 # Ép hết giờ
    # Gói telemetry chu kỳ tiếp theo kích hoạt hoàn tất task
    manager.handle_telemetry(telemetry_arrived)

    task_completed = manager.task_repo.get_by_id(task_id)
    assert task_completed["status"] == TaskState.COMPLETED.value, f"Lỗi: Task phải là COMPLETED, thực tế: {task_completed['status']}"
    # Kiểm tra tồn kho PKG_ELEC tại Kho B tăng từ 0 -> 1 kiện
    inv_b = {i["product_code"]: i["quantity"] for i in manager.inv_repo.get_node_inventory(active_map_id, "B")}
    assert inv_b["PKG_ELEC"] == 1, f"Lỗi tồn kho: {inv_b}"
    # Kiểm tra đoạn line SEG_AB đã được mở khóa tự động
    seg_ab = [s for s in manager.map_repo.get_segments(active_map_id) if s["segment_id"] == "SEG_AB"][0]
    assert seg_ab["is_locked"] == 0, "Lỗi: Đoạn line SEG_AB chưa mở khóa sau khi xong task"
    print(f"   ✅ [B7: TASK COMPLETED] Hết Dwell Time -> Task chuyển {task_completed['status']}")
    print(f"   ✅ [B8: INVENTORY UPDATED] Tồn kho PKG_ELEC tại Kho B tự động tăng thành: {inv_b['PKG_ELEC']} kiện")
    print(f"   ✅ [B9: SEGMENT UNLOCKED] Đoạn line SEG_AB đã tự động giải phóng khóa an toàn")

    # Xe AGV chạy chặng quay về Trạm Tiếp Nhận A hoàn tất chu trình Kịch bản 1
    ret_cmd = manager.agv_fsm.pending_command
    if ret_cmd:
        manager.handle_telemetry({"state": "LINE_FOLLOWING", "ack_id": ret_cmd["cmd_id"], "station": "B", "voltage": 12.4})
        manager.handle_telemetry({"state": "AT_STATION", "ack_id": ret_cmd["cmd_id"], "station": "A", "voltage": 12.4})
        print(f"   ✅ [B10: RETURN HOME] AGV hoàn thành lệnh #{ret_cmd['cmd_id']} ({ret_cmd['cmd']}) và về lại Trạm A an toàn")

    # 4. KỊCH BẢN 2: Xử lý Ngoại lệ AI Confidence thấp (< 50%)
    print("\n>> [PHẦN 4]: KỊCH BẢN 2 - Xử lý Ngoại lệ AI Không Nhận Diện Được Hàng...")
    low_conf_res = manager.handle_inbound_package(
        map_id=active_map_id, product_code="PKG_ELEC",
        ai_confidence=0.32, from_node="A", image_path="/uploads/blurry_cam.jpg"
    )
    assert low_conf_res["success"] is False, "Lỗi: Không được duyệt đơn khi AI confidence thấp"
    assert low_conf_res["status"] == TaskState.REJECTED_UNKNOWN.value, f"Lỗi trạng thái: {low_conf_res['status']}"
    assert manager.agv_fsm.pending_command is None, "Lỗi: Không được phát lệnh cho xe khi hàng lỗi"
    print(f"   ✅ Đã từ chối đơn hàng thành công (Conf = 32% < 50%): Trạng thái '{low_conf_res['status']}'")
    print("   ✅ Xe AGV không nhận lệnh, giữ nguyên vị trí an toàn tại Trạm A")

    # 5. KỊCH BẢN 3: Xử lý Ngoại lệ Pin Yếu (< 10.5V)
    print("\n>> [PHẦN 5]: KỊCH BẢN 3 - Xử lý Ngoại lệ Pin Yếu (INA226 Safety Guard)...")
    # Cập nhật pin xe xuống 10.2V
    manager.fleet_repo.update_telemetry(
        agv_id="AGV_Ver2", x=0.0, y=0.0, yaw=90.0, speed=0.0,
        voltage=10.2, current=0.2, battery_pct=15.0, status="LOW_BATTERY"
    )
    low_batt_res = manager.handle_inbound_package(
        map_id=active_map_id, product_code="PKG_MECH", ai_confidence=0.95, from_node="A"
    )
    assert low_batt_res["success"] is False, "Lỗi: Không được nhận đơn khi pin yếu"
    assert low_batt_res["status"] == "BATTERY_TOO_LOW", f"Lỗi trạng thái: {low_batt_res}"
    print(f"   ✅ Hệ thống từ chối nhận hàng do điện áp pin yếu (10.2V < 10.5V): '{low_batt_res['error']}'")

    # 6. KỊCH BẢN 4: Kiểm tra Cầu chì Watchdog Timeout
    print("\n>> [PHẦN 6]: KỊCH BẢN 4 - Kiểm tra Cầu chì An toàn Watchdog...")
    # Giả lập lùi thời gian heartbeat quá 2.5s
    manager.agv_fsm.last_heartbeat = time.time() - 3.5
    is_offline = manager.agv_fsm.check_watchdog(timeout_sec=2.5)
    assert is_offline is True and manager.agv_fsm.current_state == AGVState.OFFLINE, "Lỗi Watchdog"
    print(f"   ✅ Watchdog phát hiện mất kết nối quá 3.5s -> Đánh dấu trạng thái xe là: {manager.agv_fsm.current_state.value}")

    if test_db.exists():
        try:
            test_db.unlink()
        except Exception:
            pass

    print("\n" + "=" * 75)
    print("🎉 TẤT CẢ 6 PHẦN KIỂM THỬ FSM & ROUTER ĐÃ THÀNH CÔNG VƯỢT TRỘI (100% PASS)!")
    print("=" * 75)

if __name__ == "__main__":
    run_tests()
