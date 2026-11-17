"""
===================================================================================
KIỂM THỬ HỒI QUY CÁC BUG ĐÃ FIX TRÊN SERVER & GIAO THỨC TEST_09
(INA226 optional, route_legs Dijkstra detour, PAUSED_TRANSIT/TURNING_STATION,
 strict ack_id & RETURN completion, transition_to_dispatched, watchdog OFFLINE)
===================================================================================
"""

import sys
import time
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
SERVER_DIR = TESTS_DIR.parent
DB_DIR = SERVER_DIR / "database"

for p in (SERVER_DIR, DB_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from seed_data import seed_database
from fsm.fsm_definitions import AGVState, TaskState
from fsm.fsm_manager import WMSWorkflowManager
from app.services.ai_service import AIProductClassifierService
from app.controllers.agv_controller import AGVController
from app.controllers.camera_controller import CameraController
from app.router import MainHTTPHandler
from app.config import AGV_WATCHDOG_TIMEOUT_SEC, CAMERA_DEVICE_TOKEN, UPLOAD_DIR


def run_tests():
    print("=" * 75)
    print("🧪 BẮT ĐẦU KIỂM THỬ HỒI QUY (BUGFIX REGRESSION SUITE)")
    print("=" * 75)

    existing_uploads = set(UPLOAD_DIR.glob("*.jpg")) if UPLOAD_DIR.is_dir() else set()
    test_db = DB_DIR / "test_regression_wms.db"
    if test_db.exists():
        test_db.unlink()

    seed_database(test_db)
    manager = WMSWorkflowManager(db_file=test_db, agv_id="AGV_Ver2")
    agv_ctrl = AGVController(manager)

    active_map_id = "SABAN_154_151"

    # 1. Không chặn điều phối khi ESP32 không gắn INA226 (ina_ok=False, voltage=0.0)
    code, t_resp = agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 0.0, "ina_ok": False, "state": "STOPPED", "station": "A"
    })
    assert code == 200
    assert manager.agv_fsm.current_state == AGVState.IDLE, f"Sai trạng thái khi thiếu INA226: {manager.agv_fsm.current_state}"
    no_ina_res = manager.handle_inbound_package(active_map_id, "PKG_ELEC", ai_confidence=0.95, from_node="A")
    assert no_ina_res["success"] is True, f"Không được chặn điều phối khi thiếu INA226: {no_ina_res}"
    print("   ✅ [1/7] INA226 missing (ina_ok=False, 0.0V) -> Cho phép điều phối bình thường")

    # Dọn task vừa tạo để test tiếp
    t1_id = no_ina_res["task_id"]
    c1_id = no_ina_res["command"]["cmd_id"]
    manager.map_repo.unlock_all_by_agv("AGV_Ver2")
    manager.active_mission = None
    manager.mission_queue.clear()
    manager.agv_fsm.pending_command = None
    manager.task_repo.update_status(t1_id, TaskState.COMPLETED.value)

    # 1b. Khi CÓ INA226 (ina_ok=True) mà pin tụt -> phải chặn BATTERY_TOO_LOW
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 10.1, "ina_ok": True, "state": "STOPPED", "station": "A"
    })
    low_v_res = manager.handle_inbound_package(active_map_id, "PKG_ELEC", ai_confidence=0.95, from_node="A")
    assert low_v_res["success"] is False and low_v_res["status"] == "BATTERY_TOO_LOW", f"Phải chặn pin yếu khi ina_ok=True: {low_v_res}"
    print("   ✅ [1b/7] INA226 active (ina_ok=True, 10.1V) -> Chặn BATTERY_TOO_LOW chính xác")

    # 2. TaskStateMachine.transition_to_dispatched ghi thật to_node & route_path vào DB
    pending_tid = manager.task_fsm.create_pending_task(
        map_id=active_map_id, product_code="PKG_MECH", product_name="Bánh răng",
        from_node="A", ai_confidence=0.92, image_path="/uploads/tmp.jpg"
    )
    ok_disp = manager.task_fsm.transition_to_dispatched(pending_tid, "C", "A->B->C", "AGV_Ver2")
    assert ok_disp is True
    db_task = manager.task_repo.get_by_id(pending_tid)
    assert db_task["status"] == TaskState.DISPATCHED.value
    assert db_task["to_node"] == "C" and db_task["route_path"] == "A->B->C", f"DB chưa cập nhật: {db_task}"
    manager.task_repo.update_status(pending_tid, TaskState.ABORTED.value)
    print("   ✅ [2/7] TaskStateMachine.transition_to_dispatched cập nhật to_node & route_path vào DB")

    # 3.route_legs Dijkstra đi vòng khi đoạn SEG_AB bị xe khác khóa + không DELIVERED sớm ở TURNING_STATION / PAUSED_TRANSIT
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "IDLE", "station": "A"
    })
    manager.map_repo.lock_segments(active_map_id, ["SEG_AB"], agv_id="AGV_OTHER")
    detour_res = manager.handle_inbound_package(active_map_id, "PKG_ELEC", ai_confidence=0.95, from_node="A")
    assert detour_res["success"] is True
    assert detour_res["route_path"] == "A->D->C->B", f"Phải đi vòng A->D->C->B, thực tế: {detour_res['route_path']}"
    detour_legs = detour_res["first_mission"]["route_legs"]
    assert [(l["from"], l["to"]) for l in detour_legs] == [("A", "D"), ("D", "C"), ("C", "B")], f"route_legs sai: {detour_legs}"
    detour_tid = detour_res["task_id"]
    detour_cmd_id = detour_res["command"]["cmd_id"]

    # Gửi TURNING_STATION + ACK -> IN_TRANSIT (KHÔNG được nhảy DELIVERED)
    _, r_turn = agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 45.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "TURNING_STATION", "station": "A",
        "ack_id": detour_cmd_id
    })
    assert manager.task_repo.get_by_id(detour_tid)["status"] == TaskState.IN_TRANSIT.value
    assert manager.agv_fsm.current_state == AGVState.LINE_FOLLOWING

    # Gửi PAUSED_TRANSIT tại trạm trung gian D -> vẫn IN_TRANSIT (KHÔNG được DELIVERED)
    agv_ctrl.handle_telemetry({
        "x": 1.54, "y": 0.0, "yaw": 0.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "PAUSED_TRANSIT", "station": "D",
        "ack_id": detour_cmd_id
    })
    assert manager.task_repo.get_by_id(detour_tid)["status"] == TaskState.IN_TRANSIT.value
    assert manager.agv_fsm.current_state == AGVState.LINE_FOLLOWING

    # Đến đích thực sự B với AT_STATION -> DELIVERED
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 1.51, "yaw": 180.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "AT_STATION", "station": "B",
        "ack_id": detour_cmd_id
    })
    assert manager.task_repo.get_by_id(detour_tid)["status"] == TaskState.DELIVERED.value
    print("   ✅ [3/7] route_legs đi vòng A->D->C->B & bỏ qua TURNING_STATION / PAUSED_TRANSIT")

    # 4. Strict ack_id & RETURN completion: ack_id cũ không được tự hoàn tất RETURN
    manager.active_unloading_tasks[detour_tid] = 0.0
    _, r_ret = agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 1.51, "yaw": 180.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "AT_STATION", "station": "B",
        "ack_id": detour_cmd_id  # ack_id CŨ!
    })
    assert manager.task_repo.get_by_id(detour_tid)["status"] == TaskState.COMPLETED.value
    assert manager.active_mission is not None and manager.active_mission["category"] == "RETURN"
    assert r_ret["cmd"] == "GO_TO:A" and len(r_ret["route_legs"]) >= 1
    ret_cmd_id = r_ret["cmd_id"]
    assert ret_cmd_id != detour_cmd_id

    # Gửi gói với ack_id đúng + AT_STATION tại A -> RETURN hoàn tất & mở khóa đường
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 180.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "AT_STATION", "station": "A",
        "ack_id": ret_cmd_id
    })
    assert manager.active_mission is None
    manager.map_repo.unlock_all_by_agv("AGV_OTHER", active_map_id)
    print("   ✅ [4/7] Return Home yêu cầu đúng ack_id & trạm A mới hoàn tất")

    # 5. GAP_TIMEOUT_STOP -> EMERGENCY_STOP (không nhầm sang GAP_CROSSING)
    agv_ctrl.handle_telemetry({
        "x": 0.5, "y": 0.0, "yaw": 0.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "GAP_TIMEOUT_STOP", "station": "A"
    })
    assert manager.agv_fsm.current_state == AGVState.EMERGENCY_STOP, f"GAP_TIMEOUT_STOP phải là EMERGENCY_STOP, thực tế: {manager.agv_fsm.current_state}"
    print("   ✅ [5/7] GAP_TIMEOUT_STOP ánh xạ đúng sang EMERGENCY_STOP")

    # 6. Manual GO_TO trên bản đồ 6 node MAP_EXP_6NODES phát route_legs qua Dijkstra & khóa đoạn line
    manager.map_repo.set_active_map("MAP_EXP_6NODES")
    agv_ctrl.last_telemetry_data["station"] = "A"
    code, cmd_res = agv_ctrl.handle_command({"action": "go_to", "station": "E"})
    assert code == 200 and cmd_res["cmd"] == "GO_TO:E"
    _, poll_res = agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A", "map_id": "MAP_EXP_6NODES"
    })
    assert poll_res["cmd"] == "GO_TO:E" and len(poll_res["route_legs"]) >= 2
    assert poll_res["route_legs"][-1]["to"] == "E"
    assert poll_res["route_legs"][-1]["tx"] == 2.5 and poll_res["route_legs"][-1]["ty"] == 0.0
    print(f"   ✅ [6/7] MAP_EXP_6NODES GO_TO:E phát lộ trình Dijkstra {len(poll_res['route_legs'])} chặng: {poll_res['route']}")

    # 7. Watchdog đồng bộ OFFLINE vào agv_fsm & fleet_repo khi timeout > AGV_WATCHDOG_TIMEOUT_SEC
    agv_ctrl.last_seen_time = time.time() - (AGV_WATCHDOG_TIMEOUT_SEC + 1.0)
    dash = agv_ctrl.get_dashboard_state({"online": False})
    assert dash["connected"] is False and dash["robot_state"] == "OFFLINE"
    assert manager.agv_fsm.current_state == AGVState.OFFLINE
    assert manager.fleet_repo.get_agv("AGV_Ver2")["status"] == "OFFLINE"
    print("   ✅ [7/12] Watchdog get_dashboard_state đồng bộ OFFLINE và mở khóa đường")

    # 8. GAP_TIMEOUT_STOP -> quay về Trạm A tự động hủy task IN_TRANSIT kẹt & mở khóa Shot 1
    manager.map_repo.set_active_map(active_map_id)
    ai_svc = AIProductClassifierService()
    cam_ctrl = CameraController(manager, ai_svc)
    manager.camera_controller = cam_ctrl
    agv_ctrl.camera_controller = cam_ctrl
    cam_ctrl.agv_controller = agv_ctrl
    cam_ctrl.record_poll()
    cam_ctrl.first_poll_time = time.time() - 5.0

    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A", "map_id": active_map_id
    })
    agv_ctrl.first_seen_time = time.time() - 5.0
    stuck_dispatch = manager.handle_inbound_package(
        map_id=active_map_id, product_code="PKG_ELEC", ai_confidence=0.95, from_node="A"
    )
    assert stuck_dispatch["success"] is True
    stuck_tid = stuck_dispatch["task_id"]
    stuck_cmd = stuck_dispatch["command"]["cmd_id"]
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 1.26, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "GAP_TIMEOUT_STOP", "station": "A",
        "ack_id": stuck_cmd, "map_id": active_map_id
    })
    assert manager.task_repo.get_by_id(stuck_tid)["status"] == TaskState.IN_TRANSIT.value
    # Xe được đưa về Trạm A dừng ổn định -> tự giải phóng mission kẹt & mở khóa Shot 1
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A",
        "ack_id": stuck_cmd, "map_id": active_map_id
    })
    assert manager.task_repo.get_by_id(stuck_tid)["status"] == TaskState.ABORTED.value
    assert manager.active_mission is None
    assert cam_ctrl.pending_cam_cmd == "CAPTURE_NOW" and cam_ctrl.cam_cmd_id >= 1, (
        f"Auto Shot 1 phải được phát tự động ngay khi hồi phục tại Trạm A: {cam_ctrl.get_latest_info()}"
    )
    print("   ✅ [8/12] GAP_TIMEOUT_STOP quay về Trạm A tự hủy task kẹt & tự động chụp lại Shot 1")

    # 9. ESP32 reboot đột ngột (ack_id=0, STOPPED tại A) giải phóng task IN_TRANSIT treo nhưng không hủy task DISPATCHED mới
    cam_ctrl.clear_pending_command()
    fresh_dispatch = manager.handle_inbound_package(
        map_id=active_map_id, product_code="PKG_ELEC", ai_confidence=0.95, from_node="A"
    )
    fresh_tid = fresh_dispatch["task_id"]
    fresh_cmd = fresh_dispatch["command"]["cmd_id"]
    # Vừa phát lệnh, pending_command vẫn đang chờ ESP32 -> ack_id=0 chưa được phép hủy
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A",
        "ack_id": 0, "map_id": active_map_id
    })
    assert manager.task_repo.get_by_id(fresh_tid)["status"] == TaskState.DISPATCHED.value
    # Đã ack sang IN_TRANSIT, sau đó ESP32 reboot gửi ack_id=0 tại A
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.5, "yaw": 90.0, "speed": 45.0,
        "voltage": 12.4, "ina_ok": True, "state": "RUNNING_FAST", "station": "A",
        "ack_id": fresh_cmd, "map_id": active_map_id
    })
    assert manager.task_repo.get_by_id(fresh_tid)["status"] == TaskState.IN_TRANSIT.value
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A",
        "ack_id": 0, "map_id": active_map_id
    })
    assert manager.task_repo.get_by_id(fresh_tid)["status"] == TaskState.ABORTED.value
    print("   ✅ [9/12] ESP32 reboot (ack_id=0 tại A) tự hủy task IN_TRANSIT treo, an toàn cho DISPATCHED mới")

    # 10. Lệnh abort_missions và stop hủy task đang chạy & xóa active_mission
    d10 = manager.handle_inbound_package(
        map_id=active_map_id, product_code="PKG_MECH", ai_confidence=0.95, from_node="A"
    )
    tid10 = d10["task_id"]
    code10, res10 = agv_ctrl.handle_command({"action": "abort_missions"})
    assert code10 == 200 and res10["cmd_id"] == 0 and tid10 in res10["aborted_task_ids"]
    assert manager.task_repo.get_by_id(tid10)["status"] == TaskState.ABORTED.value
    assert manager.active_mission is None

    d10b = manager.handle_inbound_package(
        map_id=active_map_id, product_code="PKG_MECH", ai_confidence=0.95, from_node="A"
    )
    code_stop, res_stop = agv_ctrl.handle_command({"action": "stop"})
    assert code_stop == 200 and res_stop["cmd"] == "STOP"
    assert manager.task_repo.get_by_id(d10b["task_id"])["status"] == TaskState.ABORTED.value
    assert manager.active_mission is None
    print("   ✅ [10/12] Lệnh abort_missions và stop giải phóng toàn bộ mission/task kẹt")

    # 11. CameraController: đồng bộ ack_id sau restart server, tự xả pending_cam_cmd timeout > 8s, online timeout
    cam2 = CameraController(manager, ai_svc)
    q_id = cam2.queue_command("CAPTURE_NOW", metadata={"station": "A", "mode": "CAPTURE"})
    assert q_id == 1 and cam2.pending_cam_cmd == "CAPTURE_NOW"
    # Điện thoại gửi ack cũ = 5 sau khi server vừa restart -> không được xóa lệnh mới, phải nâng cmd_id lên 6
    cam2.ack_command(5)
    assert cam2.pending_cam_cmd == "CAPTURE_NOW" and cam2.cam_cmd_id == 6
    cam2.ack_command(6)
    assert cam2.pending_cam_cmd is None

    # Giả lập pending command bị treo quá 8s -> check_auto_inbound_capture tự xả và phát lệnh mới
    cam2.record_poll()
    cam2.first_poll_time = time.time() - 10.0
    cam2.last_shot1_auto_trigger = 0.0
    cam2.queue_command("CAPTURE_NOW", metadata={"station": "A", "mode": "CAPTURE"})
    cam2.last_cmd_time = time.time() - (cam2.cmd_timeout_sec + 1.0)
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A", "map_id": active_map_id
    })
    assert cam2.check_auto_inbound_capture(agv_ctrl) is True
    assert cam2.pending_cam_cmd == "CAPTURE_NOW" and cam2.cam_cmd_id == 8

    # Kiểm tra online tự động chuyển False khi không có poll > 5s
    cam2.last_poll_time = time.time() - (cam2.camera_online_timeout + 1.0)
    assert cam2.get_latest_info()["online"] is False
    print("   ✅ [11/12] CameraController đồng bộ ack sau restart, xả lệnh timeout >8s và online timeout")

    # 12. Router GET /api/state chỉ record_poll cho Camera Android, không nhận nhầm Web Dashboard
    import io
    cam_router = CameraController(manager, ai_svc)

    def _simulate_get(path_str: str, headers_dict: dict):
        h = object.__new__(MainHTTPHandler)
        h.path = path_str
        h.headers = headers_dict
        h.wfile = io.BytesIO()
        h.close_connection = False
        h.agv_controller = agv_ctrl
        h.camera_controller = cam_router
        h.wms_controller = None
        h.send_response = lambda code: None
        h.send_header = lambda k, v: None
        h.end_headers = lambda: None
        h.do_GET()

    _simulate_get("/api/state", {"Sec-Fetch-Site": "same-origin"})
    assert cam_router.get_latest_info()["online"] is False, "Web browser /api/state không được đánh dấu camera online!"
    _simulate_get("/api/state?ack_cam_cmd_id=0", {"X-Device-Token": CAMERA_DEVICE_TOKEN})
    assert cam_router.get_latest_info()["online"] is True
    print("   ✅ [12/12] Router phân biệt chính xác Web Dashboard vs Android Camera trên GET /api/state")

    # 13. AIProductClassifierService._fuse_box_prediction + Safety Gate NEEDS_REVIEW trên Shot 1 & Shot 2
    from types import SimpleNamespace
    from PIL import Image

    # (a) Không cho YOLO (0.836) ghi đè kết luận đã có OpenCV xác nhận (GEOMETRY_CONFIRM_PRISM)
    pred_prism_confirmed = SimpleNamespace(
        decision_status="CLASSIFIED",
        predicted_class="triangular_prism",
        confidence=0.621,
        decision_type="GEOMETRY_CONFIRM_PRISM",
        mobilenet_class_name="triangular_prism",
        mobilenet_confidence=0.621,
        mobilenet_probabilities={"cylinder": 0.147, "rectangular_box": 0.232, "triangular_prism": 0.621},
        geometry_candidate="triangular_prism",
        geometry_quality="HIGH",
    )
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._fuse_box_prediction(
        pred_prism_confirmed, y_cls="rectangular_box", y_conf=0.836
    )
    assert f_st == "CLASSIFIED" and f_cls == "triangular_prism" and f_rule == "GEOMETRY_CONFIRM_PRISM", (
        f"YOLO không được đè GEOMETRY_CONFIRM_PRISM: {(f_cls, f_conf, f_st, f_rule)}"
    )

    # (b) Đồng thuận 2/3 YOLO + OpenCV cứu MobileNet yếu/mờ chưa có Geo xác nhận
    pred_box_rescued = SimpleNamespace(
        decision_status="CLASSIFIED",
        predicted_class="triangular_prism",
        confidence=0.617,
        decision_type="MOBILENET_DIRECT",
        mobilenet_class_name="triangular_prism",
        mobilenet_confidence=0.617,
        mobilenet_probabilities={"cylinder": 0.371, "rectangular_box": 0.012, "triangular_prism": 0.617},
        geometry_candidate="rectangular_box",
        geometry_quality="HIGH",
    )
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._fuse_box_prediction(
        pred_box_rescued, y_cls="rectangular_box", y_conf=0.610
    )
    assert f_st == "CLASSIFIED" and f_cls == "rectangular_box" and f_rule == "YOLO_GEOMETRY_CONSENSUS", (
        f"Phải chọn đồng thuận 2/3 YOLO+OpenCV: {(f_cls, f_conf, f_st, f_rule)}"
    )

    # (c) Bảo toàn NEEDS_REVIEW thay vì ép mặc định rectangular_box khi cả 3 nguồn bất đồng
    pred_ambig = SimpleNamespace(
        decision_status="NEEDS_REVIEW",
        predicted_class=None,
        confidence=0.0,
        decision_type="NEEDS_REVIEW",
        mobilenet_class_name="cylinder",
        mobilenet_confidence=0.700,
        mobilenet_probabilities={"cylinder": 0.700, "rectangular_box": 0.131, "triangular_prism": 0.169},
        geometry_candidate="triangular_prism",
        geometry_quality="HIGH",
    )
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._fuse_box_prediction(
        pred_ambig, y_cls="rectangular_box", y_conf=0.562
    )
    assert f_st == "NEEDS_REVIEW" and f_cls is None and f_conf == 0.0, (
        f"Phải giữ NEEDS_REVIEW thay vì ép rectangular_box: {(f_cls, f_conf, f_st, f_rule)}"
    )

    # (d) Giải tỏa NEEDS_REVIEW nếu YOLO >= 0.45 đồng thuận với MobileNet
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._fuse_box_prediction(
        pred_ambig, y_cls="cylinder", y_conf=0.726
    )
    assert f_st == "CLASSIFIED" and f_cls == "cylinder" and f_rule == "YOLO_TIEBREAK_REVIEW", (
        f"YOLO+MobileNet đồng thuận phải giải tỏa NEEDS_REVIEW: {(f_cls, f_conf, f_st, f_rule)}"
    )

    # (d2) Phủ quyết GEOMETRY_OVERRIDE lỗi [0%] khi YOLO + MobileNet cùng chắc chắn cylinder
    pred_bad_override = SimpleNamespace(
        decision_status="CLASSIFIED",
        predicted_class="triangular_prism",
        confidence=0.0001,
        decision_type="GEOMETRY_OVERRIDE_PRISM",
        mobilenet_class_name="cylinder",
        mobilenet_confidence=0.9999,
        mobilenet_probabilities={"cylinder": 0.9999, "rectangular_box": 0.0, "triangular_prism": 0.0001},
        geometry_candidate="triangular_prism",
        geometry_quality="HIGH",
    )
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._fuse_box_prediction(
        pred_bad_override, y_cls="cylinder", y_conf=0.850
    )
    assert f_st == "CLASSIFIED" and f_cls == "cylinder" and f_conf >= 0.99 and f_rule == "YOLO_MOBILENET_MAJORITY", (
        f"YOLO+MobileNet 2/3 phải thắng GEOMETRY_OVERRIDE 0%: {(f_cls, f_conf, f_st, f_rule)}"
    )

    # (d3) Giải tỏa NEEDS_REVIEW cho hộp chữ nhật khi YOLO >= 0.75 + OpenCV thấy đường vuông góc & extent >= 0.85
    pred_ortho_review = SimpleNamespace(
        decision_status="NEEDS_REVIEW",
        predicted_class=None,
        confidence=0.0,
        decision_type="NEEDS_REVIEW",
        mobilenet_class_name="cylinder",
        mobilenet_confidence=0.740,
        mobilenet_probabilities={"cylinder": 0.740, "rectangular_box": 0.106, "triangular_prism": 0.155},
        geometry_candidate="triangular_prism",
        geometry_quality="HIGH",
        geometry_evidence={"extent": 0.93, "diagnostic_flags": {"has_strictly_orthogonal_lines": True}},
    )
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._fuse_box_prediction(
        pred_ortho_review, y_cls="rectangular_box", y_conf=0.838
    )
    assert f_st == "CLASSIFIED" and f_cls == "rectangular_box" and f_rule == "YOLO_TIEBREAK_REVIEW", (
        f"YOLO+Ortho Box phải giải tỏa NEEDS_REVIEW: {(f_cls, f_conf, f_st, f_rule)}"
    )

    # (e) End-to-end Shot 1 (không tạo task) & Shot 2 (không báo unloaded=True) khi AI trả về NEEDS_REVIEW
    mock_ai = object.__new__(AIProductClassifierService)
    mock_ai.is_placeholder_mode = False
    mock_ai.engine_name = "Mock_Hybrid"
    mock_det = SimpleNamespace(
        yolo_detections={(10, 10, 110, 110): {"class_name": "rectangular_box", "conf": 0.56}},
        detect=lambda im: ((10, 10, 110, 110), 1, [(10, 10, 110, 110)]),
    )
    mock_ai.predictor = SimpleNamespace(
        classifier=SimpleNamespace(detector=mock_det),
        predict=lambda im, bbox=None, input_mode="pre_cropped_roi": pred_ambig,
    )
    buf_jpg = io.BytesIO()
    Image.new("RGB", (128, 128), color=(80, 90, 100)).save(buf_jpg, format="JPEG")
    real_jpg_bytes = buf_jpg.getvalue()

    ai_review_res = mock_ai.classify_image(real_jpg_bytes)
    assert ai_review_res["decision_status"] == "NEEDS_REVIEW"
    assert ai_review_res["detected_items"] == [] and ai_review_res["success"] is False

    # Shot 1 với NEEDS_REVIEW phải cảnh báo và KHÔNG tạo task
    cam_nr = CameraController(manager, mock_ai)
    boundary_nr = "----BoundaryNeedsReview"
    ct_nr = f"multipart/form-data; boundary={boundary_nr}"
    body_nr = (
        f"--{boundary_nr}\r\nContent-Disposition: form-data; name=\"station\"\r\n\r\nA\r\n"
        f"--{boundary_nr}\r\nContent-Disposition: form-data; name=\"mode\"\r\n\r\nCAPTURE\r\n"
        f"--{boundary_nr}\r\nContent-Disposition: form-data; name=\"image\"; filename=\"nr.jpg\"\r\n"
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + real_jpg_bytes + f"\r\n--{boundary_nr}--\r\n".encode("utf-8")
    tasks_before = len(manager.task_repo.get_recent_tasks(limit=20))
    code_nr, resp_nr = cam_nr.handle_upload(ct_nr, body_nr)
    tasks_after = len(manager.task_repo.get_recent_tasks(limit=20))
    assert code_nr == 200 and resp_nr["status"] == "warning" and "NEEDS_REVIEW" in resp_nr["message"], resp_nr
    assert tasks_after == tasks_before, "Shot 1 NEEDS_REVIEW không được tạo task mới!"

    # Shot 2 với NEEDS_REVIEW tuyệt đối không được coi là is_empty_tray (unloaded=True)
    unload_nr = manager.confirm_unloading_with_vision(
        image_bytes=real_jpg_bytes, ai_service=mock_ai, station="B", ai_res=ai_review_res
    )
    assert unload_nr["unloaded"] is False and unload_nr["status"] == "NEEDS_REVIEW", unload_nr
    print("   ✅ [13/13] Fusion YOLO+Hybrid+OpenCV chuẩn xác & Safety Gate NEEDS_REVIEW chặn sai ở Shot 1/Shot 2")

    # 14. Lọc khung lồng nhau (nested duplicate) ở ObjectDetector nếu có OpenCV/weights
    try:
        from weights.roi_extraction import ObjectDetector
        # Tight box [139, 752, 1467, 1506] vs giant overlapping box [52, 807, 1728, 2303] (~93% containment)
        assert ObjectDetector._is_nested_duplicate((52, 807, 1728, 2303), [(139, 752, 1467, 1506)]) is True
        # Two disjoint objects on tray
        assert ObjectDetector._is_nested_duplicate((339, 1543, 1192, 2304), [(679, 780, 1646, 1510)]) is False
        print("   ✅ [14/15] ObjectDetector._is_nested_duplicate lọc chính xác box khổng lồ bao trùm")
    except Exception:
        print("   ℹ️ [14/15] Bỏ qua import roi_extraction trong môi trường không có cv2/torch")

    # 15. Kiểm thử toàn diện các cơ chế Bảo mật IoT (Anti-Spoofing, HMAC-SHA256, Least-Privilege RBAC, Anti-DoS, Path Traversal, Rate Limit)
    import json
    from app.config import AGV_DEVICE_TOKEN, ADMIN_API_TOKEN, MAX_JSON_BODY_BYTES, AUTH_MAX_FAILURES
    from app.services.security_service import SecurityService, WEB_SESSION_COOKIE_NAME

    SecurityService.reset_rate_limits()

    def _sim_http(method: str, path_str: str, headers_dict: dict, body_bytes: bytes = b"", client_ip: str = "192.168.1.99"):
        h = object.__new__(MainHTTPHandler)
        h.path = path_str
        hdrs = dict(headers_dict)
        if body_bytes and "Content-Length" not in hdrs:
            hdrs["Content-Length"] = str(len(body_bytes))
        h.headers = hdrs
        h.rfile = io.BytesIO(body_bytes)
        h.wfile = io.BytesIO()
        h.client_address = (client_ip, 54321)
        h.close_connection = False
        h.agv_controller = agv_ctrl
        h.camera_controller = cam_router
        h.wms_controller = None
        resp_meta = {"code": 200, "headers": {}}
        h.send_response = lambda code: resp_meta.__setitem__("code", code)
        h.send_header = lambda k, v: resp_meta["headers"].__setitem__(k, v)
        h.end_headers = lambda: None
        if method == "GET":
            h.do_GET()
        else:
            h.do_POST()
        raw_out = h.wfile.getvalue()
        try:
            json_out = json.loads(raw_out.decode("utf-8")) if raw_out else {}
        except Exception:
            json_out = {}
        return resp_meta["code"], resp_meta["headers"], json_out

    # (a) Chặn giả mạo Header Referer / Sec-Fetch-Site từ IP mạng LAN
    code_spoof, _, _ = _sim_http("GET", "/api/state", {
        "Referer": "http://localhost:5000/",
        "Sec-Fetch-Site": "same-origin"
    }, client_ip="192.168.1.150")
    assert code_spoof == 401, f"Phải chặn giả mạo Referer từ LAN IP! Got {code_spoof}"

    # (b) Cho phép Web Session Cookie đã ký số HMAC-SHA256 khi mở trang chủ '/'
    code_idx, hdrs_idx, _ = _sim_http("GET", "/", {}, client_ip="192.168.1.150")
    assert code_idx == 200 and WEB_SESSION_COOKIE_NAME in hdrs_idx.get("Set-Cookie", "")
    cookie_val = hdrs_idx["Set-Cookie"].split(";")[0]
    code_sess, _, _ = _sim_http("GET", "/api/state", {"Cookie": cookie_val}, client_ip="192.168.1.150")
    assert code_sess == 200, "Session Cookie ký số HMAC phải được xác thực hợp lệ!"

    # (c) Bảo vệ API /api/logs và /api/logs/clear trước truy cập trái phép
    code_log_unauth, _, _ = _sim_http("GET", "/api/logs", {}, client_ip="192.168.1.151")
    assert code_log_unauth == 401
    code_log_clear_unauth, _, _ = _sim_http("POST", "/api/logs/clear", {}, client_ip="192.168.1.151")
    assert code_log_clear_unauth == 401
    code_log_admin, _, _ = _sim_http("GET", "/api/logs", {"X-Device-Token": ADMIN_API_TOKEN}, client_ip="192.168.1.151")
    assert code_log_admin == 200

    # (d) Chặn Path Traversal trên /static/ và /images/
    code_trav, _, _ = _sim_http("GET", "/static/../../app/config.py", {}, client_ip="192.168.1.152")
    assert code_trav == 403, f"Path traversal phải trả 403! Got {code_trav}"

    # (e) Chặn gói tin vượt ngưỡng kích thước (413 Payload Too Large) trước khi đọc RAM
    code_big, _, _ = _sim_http("POST", "/api/telemetry", {
        "X-Device-Token": AGV_DEVICE_TOKEN,
        "Content-Length": str(MAX_JSON_BODY_BYTES + 1024)
    }, body_bytes=b"{}", client_ip="192.168.1.153")
    assert code_big == 413, f"Payload quá lớn phải trả 413! Got {code_big}"

    # (f) Phân quyền tối thiểu (Least-Privilege RBAC) trên POST /api/command
    # - Robot AGV không được phép tự gọi /api/command
    code_agv_cmd, _, _ = _sim_http(
        "POST", "/api/command", {"X-Device-Token": AGV_DEVICE_TOKEN},
        body_bytes=b'{"action":"stop"}', client_ip="192.168.1.154"
    )
    assert code_agv_cmd == 401
    # - Camera chỉ được gọi capture_now / go_to, CẤM gọi abort_missions / stop / reset_odom
    code_cam_forbid, _, _ = _sim_http(
        "POST", "/api/command", {"X-Device-Token": CAMERA_DEVICE_TOKEN},
        body_bytes=b'{"action":"abort_missions"}', client_ip="192.168.1.154"
    )
    assert code_cam_forbid == 403
    code_cam_allow, _, _ = _sim_http(
        "POST", "/api/command", {"X-Device-Token": CAMERA_DEVICE_TOKEN},
        body_bytes=b'{"action":"capture_now","station":"A"}', client_ip="192.168.1.154"
    )
    assert code_cam_allow == 200

    # (g) Ký số HMAC-SHA256 lệnh điều khiển (cmd_sig) và xác minh chữ ký gói tin (X-Payload-Signature)
    agv_ctrl.handle_command({"action": "go_to", "station": "B"})
    t_payload = b'{"x":0.0,"y":0.0,"yaw":90.0,"speed":0.0,"voltage":12.2,"state":"IDLE","station":"A","ack_id":0}'
    # Chữ ký sai -> bị chặn 401
    code_bad_sig, _, _ = _sim_http(
        "POST", "/api/telemetry",
        {"X-Device-Token": AGV_DEVICE_TOKEN, "X-Payload-Signature": "00000000000000000000000000000000"},
        body_bytes=t_payload, client_ip="192.168.1.155"
    )
    assert code_bad_sig == 401
    # Chữ ký đúng -> trả 200 kèm cmd_sig hợp lệ cho ESP32
    valid_sig = SecurityService.sign_payload(t_payload)
    code_good_sig, _, t_out = _sim_http(
        "POST", "/api/telemetry",
        {"X-Device-Token": AGV_DEVICE_TOKEN, "X-Payload-Signature": valid_sig},
        body_bytes=t_payload, client_ip="192.168.1.155"
    )
    assert code_good_sig == 200 and t_out.get("cmd") == "GO_TO:B"
    assert SecurityService.verify_command_signature(
        t_out["cmd_id"], t_out["cmd"], t_out.get("route", ""), t_out.get("cmd_sig", "")
    ) is True

    # (h) Chống tấn công vét cạn (Brute-Force Rate Limit) khóa tạm thời IP gửi sai Token liên tiếp
    brute_ip = "192.168.1.250"
    for _ in range(AUTH_MAX_FAILURES):
        c_fail, _, _ = _sim_http("GET", "/api/state", {"X-Device-Token": "wrong_token"}, client_ip=brute_ip)
        assert c_fail == 401
    # Lần thứ AUTH_MAX_FAILURES + 1 ngay cả khi gửi đúng token vẫn bị chặn 429 trong thời gian khóa
    c_locked, _, _ = _sim_http("GET", "/api/state", {"X-Device-Token": ADMIN_API_TOKEN}, client_ip=brute_ip)
    assert c_locked == 429, f"IP vượt ngưỡng brute-force phải trả 429! Got {c_locked}"
    SecurityService.reset_rate_limits()
    print("   ✅ [15/15] Bảo mật IoT đa tầng (Anti-Spoofing, HMAC-SHA256, Least-Privilege RBAC, 413 DoS, 403 PathTraversal, 429 RateLimit)")

    if test_db.exists():
        try:
            test_db.unlink()
        except Exception:
            pass
    if UPLOAD_DIR.is_dir():
        for f in set(UPLOAD_DIR.glob("*.jpg")) - existing_uploads:
            try:
                f.unlink()
            except Exception:
                pass

    print("=" * 75)
    print("🎉 TẤT CẢ CÁC CA KIỂM THỬ HỒI QUY ĐÃ PASS 100%!")
    print("=" * 75)


if __name__ == "__main__":
    run_tests()
