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

    # 13. Đánh giá kết quả CV Ver3 (YOLO11n + MobileNetV3 + GeometricAnalyzer) & Safety Gates trên Shot 1 & Shot 2
    from types import SimpleNamespace
    from PIL import Image
    import tempfile

    # (a) Giữ nguyên kết luận GEOMETRY_CONFIRM_PRISM từ CV Ver3
    obj_prism_confirmed = {
        "object_id": 1,
        "bbox": [20, 20, 120, 120],
        "class_name": "triangular_prism",
        "confidence": 0.621,
        "decision_type": "GEOMETRY_CONFIRM_PRISM",
        "explanation": "Lăng trụ xác nhận hình học",
    }
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._evaluate_ver3_detection(obj_prism_confirmed)
    assert f_st == "CLASSIFIED" and f_cls == "triangular_prism" and f_conf == 0.621 and f_rule == "GEOMETRY_CONFIRM_PRISM", (
        f"Sai đánh giá GEOMETRY_CONFIRM_PRISM: {(f_cls, f_conf, f_st, f_rule)}"
    )

    # (b) Bảo toàn kết luận GEOMETRY_OVERRIDE_BOX / SCENE_RECONCILE từ CV Ver3
    obj_box_override = {
        "class_name": "rectangular_box",
        "confidence": 0.812,
        "decision_type": "GEOMETRY_OVERRIDE_BOX",
    }
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._evaluate_ver3_detection(obj_box_override)
    assert f_st == "CLASSIFIED" and f_cls == "rectangular_box" and f_conf == 0.812 and f_rule == "GEOMETRY_OVERRIDE_BOX", (
        f"Sai đánh giá GEOMETRY_OVERRIDE_BOX: {(f_cls, f_conf, f_st, f_rule)}"
    )

    obj_direct_empty_type = {
        "class_name": "cylinder",
        "confidence": 0.9534,
        "decision_type": "",
    }
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._evaluate_ver3_detection(obj_direct_empty_type)
    assert f_st == "CLASSIFIED" and f_cls == "cylinder" and f_conf == 0.953 and f_rule == "CV_VER3_CLASSIFIED", (
        f"Sai đánh giá fallback decision_type: {(f_cls, f_conf, f_st, f_rule)}"
    )

    # (c) Từ chối NEEDS_REVIEW hoặc độ tin cậy thấp (< 0.50)
    obj_ambig = {
        "object_id": 1,
        "bbox": [10, 10, 110, 110],
        "class_name": "cylinder",
        "confidence": 0.42,
        "decision_type": "NEEDS_REVIEW",
    }
    f_cls, f_conf, f_st, f_rule = AIProductClassifierService._evaluate_ver3_detection(obj_ambig)
    assert f_st == "NEEDS_REVIEW" and f_cls is None and f_conf == 0.0 and f_rule == "NEEDS_REVIEW", (
        f"Phải giữ NEEDS_REVIEW: {(f_cls, f_conf, f_st, f_rule)}"
    )

    obj_low_conf = {"class_name": "cylinder", "confidence": 0.48, "decision_type": "MOBILENET_DIRECT"}
    f_cls, f_conf, f_st, _ = AIProductClassifierService._evaluate_ver3_detection(obj_low_conf)
    assert f_st == "NEEDS_REVIEW" and f_cls is None and f_conf == 0.0

    # (d) Từ chối nhãn lạ ngoài CLASS_TO_WMS_MAPPING
    obj_unknown = {"class_name": "unknown_shape", "confidence": 0.99, "decision_type": "MOBILENET_DIRECT"}
    f_cls, f_conf, f_st, _ = AIProductClassifierService._evaluate_ver3_detection(obj_unknown)
    assert f_st == "NEEDS_REVIEW" and f_cls is None and f_conf == 0.0, (
        f"Nhãn lạ không được phép CLASSIFIED: {(f_cls, f_conf, f_st)}"
    )

    # (e) End-to-end Shot 1 (không tạo task) & Shot 2 (không báo unloaded=True) khi AI trả về NEEDS_REVIEW, và xác minh ảnh truyền vào là BGR ndarray
    captured_bgr_shapes = []
    mock_ai = object.__new__(AIProductClassifierService)
    mock_ai.yolo_path = Path("yolo11n_v5b_best.pt")
    mock_ai.checkpoint_path = Path("best.pth")
    mock_ai.is_placeholder_mode = False
    mock_ai.engine_name = "CV_Ver3_YOLO11n_MobileNetV3_Geometry"

    def _mock_rec_nr(img_bgr):
        captured_bgr_shapes.append((getattr(img_bgr, "shape", None), int(img_bgr[0, 0, 0]), int(img_bgr[0, 0, 2])))
        return [obj_ambig], img_bgr, 12.5

    mock_ai.recognizer = SimpleNamespace(recognize_scene=_mock_rec_nr)
    buf_jpg = io.BytesIO()
    Image.new("RGB", (128, 128), color=(20, 90, 200)).save(buf_jpg, format="JPEG")
    real_jpg_bytes = buf_jpg.getvalue()

    ai_review_res = mock_ai.classify_image(real_jpg_bytes)
    assert ai_review_res["decision_status"] == "NEEDS_REVIEW"
    assert ai_review_res["detected_items"] == [] and ai_review_res["success"] is False
    # Ảnh truyền cho recognize_scene phải là OpenCV BGR (kênh B > kênh R cho ảnh màu RGB=(20,90,200))
    assert captured_bgr_shapes and captured_bgr_shapes[0][0] == (128, 128, 3)
    assert captured_bgr_shapes[0][1] > 150 and captured_bgr_shapes[0][2] < 60, f"Không đúng kênh BGR: {captured_bgr_shapes[0]}"

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

    # (f) CV Ver3 báo khay trống (NO_OBJECT) khi recognize_scene trả về []
    mock_ai.recognizer = SimpleNamespace(recognize_scene=lambda img_bgr: ([], img_bgr, 8.4))
    res_empty = mock_ai.classify_image(real_jpg_bytes)
    assert res_empty["product_code"] == "NONE" and res_empty["decision_status"] == "NO_OBJECT"
    assert res_empty["detected_items"] == [] and res_empty["total_detected"] == 0 and res_empty["success"] is False

    # (g) Multi-object (3 vật thể từ CV Ver3 recognize_scene) ánh xạ đủ 3 sản phẩm & giữ log CV Ver3
    multi_ver3_objs = [
        {
            "object_id": 1, "bbox": [8, 8, 40, 60], "class_name": "rectangular_box",
            "confidence": 0.91, "decision_type": "MOBILENET_DIRECT", "explanation": "Box",
            "vietnamese_log": "#1 Hợp lệ",
        },
        {
            "object_id": 2, "bbox": [45, 8, 80, 60], "class_name": "cylinder",
            "confidence": 0.94, "decision_type": "MOBILENET_DIRECT", "explanation": "Cyl",
            "vietnamese_log": "#2 Hợp lệ",
        },
        {
            "object_id": 3, "bbox": [85, 8, 120, 60], "class_name": "triangular_prism",
            "confidence": 0.88, "decision_type": "GEOMETRY_CONFIRM_PRISM", "explanation": "Prism",
            "vietnamese_log": "#3 Hợp lệ",
        },
    ]
    mock_ai.recognizer = SimpleNamespace(recognize_scene=lambda img_bgr: (multi_ver3_objs, img_bgr, 28.7))
    res_multi = mock_ai.classify_image(real_jpg_bytes)
    assert res_multi["decision_status"] == "CLASSIFIED" and res_multi["total_detected"] == 3, res_multi
    assert [i["product_code"] for i in res_multi["detected_items"]] == ["PKG_ELEC", "PKG_MECH", "PKG_TOOL"]
    assert res_multi["latency_ms"] == 28.7
    assert res_multi["detected_items"][2]["decision_type"] == "GEOMETRY_CONFIRM_PRISM"

    # (h) _init_engine yêu cầu ĐỦ cả yolo11n_v5b_best.pt & best.pth và nạp SLAMCVRecognizer
    captured_init = {}

    class _DummySLAMCVRecognizer:
        def __init__(self, **kwargs):
            captured_init.update(kwargs)

    prev_ver3_mod = sys.modules.get("slam_cv_recognizer")
    try:
        sys.modules["slam_cv_recognizer"] = SimpleNamespace(SLAMCVRecognizer=_DummySLAMCVRecognizer)
        with tempfile.TemporaryDirectory() as tmp_wdir:
            tmp_best = Path(tmp_wdir) / "best.pth"
            tmp_best.write_bytes(b"stub_mnv3_checkpoint")

            # Thiếu yolo11n_v5b_best.pt -> giữ Placeholder_Mode và TỪ CHỐI bịa PKG_ELEC 95% trên ảnh thật
            svc_missing_yolo = AIProductClassifierService(model_path=tmp_best)
            assert svc_missing_yolo.is_placeholder_mode is True
            assert svc_missing_yolo.engine_name == "Placeholder_Mode"
            res_no_fake = svc_missing_yolo.classify_image(real_jpg_bytes)
            assert res_no_fake["decision_status"] == "NEEDS_REVIEW" and res_no_fake["success"] is False, res_no_fake

            # Khi recognize_scene ném exception -> PHẢI trả NEEDS_REVIEW, không tụt xuống fallback 95%
            def _boom(_img):
                raise RuntimeError("HoughLinesP shape error")
            mock_ai.recognizer = SimpleNamespace(recognize_scene=_boom)
            res_err = mock_ai.classify_image(real_jpg_bytes)
            assert res_err["decision_status"] == "NEEDS_REVIEW" and res_err["success"] is False, res_err

            # Đủ cả YOLO11n + MobileNetV3 -> kích hoạt CV Ver3
            tmp_yolo = Path(tmp_wdir) / "yolo11n_v5b_best.pt"
            tmp_yolo.write_bytes(b"stub_yolo_checkpoint")
            svc_ver3 = AIProductClassifierService(model_path=tmp_best, yolo_path=tmp_yolo)
            assert svc_ver3.is_placeholder_mode is False
            assert svc_ver3.engine_name == "CV_Ver3_YOLO11n_MobileNetV3_Geometry"
            assert captured_init.get("yolo_path") == tmp_yolo
            assert captured_init.get("mnv3_path") == tmp_best
            assert captured_init.get("device") == "cpu"
    finally:
        if prev_ver3_mod is None:
            sys.modules.pop("slam_cv_recognizer", None)
        else:
            sys.modules["slam_cv_recognizer"] = prev_ver3_mod

    # (i) Nhãn động theo DB & tránh đè nhãn cố định (Shot 1 vs Shot 2 tại trạm đích)
    b1, f1 = AIProductClassifierService._format_item_banner(0, {
        "object_id": 1, "product_code": "PKG_MECH", "short_vi": "PKG_MECH (Hình trụ)",
        "target_node": "D", "confidence": 0.98,
    })
    assert "PKG_MECH" in b1 and "-> Kho D" in b1 and f1 is True, b1
    b2, f2 = AIProductClassifierService._format_item_banner(0, {
        "object_id": 1, "product_code": "PKG_MECH", "short_vi": "PKG_MECH (Hình trụ)",
        "target_node": "D", "confidence": 0.98,
    }, unload_station="D", expected_product_codes=["PKG_MECH"])
    assert "DỠ @ KHO D" in b2 and f2 is True, b2
    b_hold, f_hold = AIProductClassifierService._format_item_banner(1, {
        "object_id": 2, "product_code": "PKG_TOOL", "short_vi": "PKG_TOOL (Lăng trụ)",
        "target_node": "C", "confidence": 0.98,
    }, unload_station="D", expected_product_codes=["PKG_MECH"])
    assert "Giữ -> Kho C" in b_hold and f_hold is False, b_hold

    # Kiểm tra _select_label_rect không đặt 2 nhãn đè lên nhau khi 2 khung kề sát/chồng góc
    boxes_pair = [(100, 100, 600, 500), (200, 450, 700, 900)]
    r0 = AIProductClassifierService._select_label_rect(0, boxes_pair, [], 260, 40, 1000, 1000)
    r1 = AIProductClassifierService._select_label_rect(1, boxes_pair, [r0], 260, 40, 1000, 1000)
    inter_x = max(0, min(r0[2], r1[2]) - max(r0[0], r1[0]))
    inter_y = max(0, min(r0[3], r1[3]) - max(r0[1], r1[1]))
    assert inter_x * inter_y == 0, f"Nhãn không được đè lên nhau: r0={r0}, r1={r1}"
    assert r0[0] >= 100 and r0[2] <= 600 and r1[0] >= 200 and r1[2] <= 700, (r0, r1)

    print("   ✅ [13/15] CV Ver3, Safety Gates & Nhãn BBox động thông minh Shot 1/Shot 2 chuẩn xác")

    # 14. Kiểm thử SLAMCVRecognizer._resolve_nested_boxes & GeometricAnalyzer (tương thích OpenCV 4.x/5.0 HoughLinesP)
    try:
        from weights.slam_cv_recognizer import SLAMCVRecognizer, GeometricAnalyzer
        rec_stub = object.__new__(SLAMCVRecognizer)
        # Box nhỏ nằm lọt bên trong box lớn có conf cao hơn (containment > 0.85) -> loại box thừa còn 1
        kept_nested = rec_stub._resolve_nested_boxes(
            [((139, 752, 1467, 1506), 0.90), ((220, 820, 1400, 1450), 0.40)],
            h_img=2500, w_img=2500,
        )
        assert len(kept_nested) == 1, f"Phải loại box lồng nhau: {kept_nested}"
        # Hai box tách rời nhau trên khay -> giữ nguyên cả 2
        kept_disjoint = rec_stub._resolve_nested_boxes(
            [((100, 100, 300, 300), 0.90), ((500, 500, 700, 700), 0.85)],
            h_img=2500, w_img=2500,
        )
        assert len(kept_disjoint) == 2, f"Phải giữ đủ 2 box tách rời: {kept_disjoint}"
        # Lọc vách hộp / nẹp kim loại trên ảnh Android độ phân giải cao (1728x2304)
        filtered_borders = rec_stub._resolve_nested_boxes(
            [((0, 12, 399, 2201), 0.0536), ((1462, 210, 1728, 1861), 0.1093)],
            h_img=2304, w_img=1728,
        )
        assert filtered_borders == [], f"Phải lọc sạch vách/nẹp viền khay trống: {filtered_borders}"

        # Kit 3 màu vẫn kích hoạt tách màu ngay cả khi YOLO gộp còn 1 box, và khôi phục khối bị sót
        import numpy as _np
        kit_canvas = Image.new("RGB", (600, 600), color=(240, 240, 240))
        from PIL import ImageDraw as _IDraw
        _IDraw.Draw(kit_canvas).rectangle([40, 40, 240, 220], fill=(210, 90, 170))
        _IDraw.Draw(kit_canvas).rectangle([320, 60, 520, 260], fill=(220, 180, 20))
        _IDraw.Draw(kit_canvas).rectangle([160, 320, 380, 540], fill=(210, 30, 30))
        kit_bgr = _np.array(kit_canvas, dtype=_np.uint8)[:, :, ::-1].copy()
        is_kit_1, _ = SLAMCVRecognizer._detect_physical_tray_kit(kit_bgr, num_raw_boxes=1)
        assert is_kit_1 is True, "Kit đủ cụm màu vẫn phải bật is_kit khi YOLO gộp 1 box"
        recovered = SLAMCVRecognizer._recover_missed_kit_boxes(kit_bgr, [(320, 60, 520, 260), (160, 320, 380, 540)])
        assert len(recovered) == 3, f"Phải khôi phục cụm màu hồng bị YOLO bỏ sót: {recovered}"

        # Vẽ hình chữ nhật rõ cạnh để kích hoạt cv2.HoughLinesP (OpenCV 5.0 trả shape (N,4))
        rect_canvas = Image.new("RGB", (256, 256), color=(240, 240, 240))
        _IDraw.Draw(rect_canvas).rectangle([50, 70, 200, 180], fill=(170, 80, 150))
        geo_res = GeometricAnalyzer().analyze(rect_canvas, is_bgr=False)
        assert geo_res.total_lines >= 2 and geo_res.right_angles_count >= 3, geo_res
        assert rec_stub._count_crop_lines(_np.array(rect_canvas)) >= 2
        print("   ✅ [14/15] SLAMCVRecognizer & GeometricAnalyzer chạy chuẩn xác trên OpenCV (4.x/5.0)")
    except ImportError:
        print("   ℹ️ [14/15] Bỏ qua import slam_cv_recognizer trong môi trường không có cv2/torch/ultralytics")

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
    print("   ✅ [15/16] Bảo mật IoT đa tầng (Anti-Spoofing, HMAC-SHA256, Least-Privilege RBAC, 413 DoS, 403 PathTraversal, 429 RateLimit)")

    # 16. Điều phối trực tiếp đường ngắn nhất từ trạm kho (B -> C) khi Shot 2 phát hiện hàng tiếp theo thay vì quay về A
    agv_ctrl.handle_command({"action": "abort_missions"})
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A", "map_id": active_map_id
    })
    d_ab = manager.handle_inbound_package(active_map_id, "PKG_ELEC", ai_confidence=0.95, from_node="A")
    assert d_ab["success"] is True and d_ab["route_path"] == "A->B"
    tid_ab = d_ab["task_id"]
    cmd_ab = d_ab["command"]["cmd_id"]
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 1.51, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "AT_STATION", "station": "B",
        "ack_id": cmd_ab, "map_id": active_map_id
    })
    # Tại trạm B, Shot 2 thấy PKG_ELEC đã được lấy xuống và có PKG_MECH (đi trạm C) trên khay
    ai_shot2_at_b = {
        "success": True,
        "decision_status": "CLASSIFIED",
        "product_code": "PKG_MECH",
        "product_name": "Chi Tiết Bánh Răng & Khớp Nối",
        "category": "MECHANICAL",
        "target_station": "C",
        "confidence": 0.99,
        "detected_items": [{
            "product_code": "PKG_MECH",
            "product_name": "Chi Tiết Bánh Răng & Khớp Nối",
            "category": "MECHANICAL",
            "target_station": "C",
            "confidence": 0.99,
        }],
    }
    u_bc = manager.confirm_unloading_with_vision(
        image_bytes=b"", ai_service=ai_svc, station="B", task_id=tid_ab, ai_res=ai_shot2_at_b
    )
    assert u_bc["success"] is True and u_bc["unloaded"] is True
    assert u_bc["status"] == "MISSION_ADVANCED", f"Phải đi tiếp MISSION_ADVANCED thay vì về A: {u_bc}"
    assert u_bc["next_station"] == "C" and u_bc["command"]["cmd"] == "GO_TO:C"
    assert u_bc["next_mission"]["route_str"] == "B->C", f"Phải lập lộ trình ngắn nhất trực tiếp B->C: {u_bc['next_mission']}"
    # Kiểm tra chặng RETURN cuối hàng đợi vẫn quay về Trạm Tiếp Nhận A (không phải về B)
    ret_missions = [m for m in manager.mission_queue if m.get("category") == "RETURN"]
    assert len(ret_missions) == 1 and ret_missions[0]["to_node"] == "A", f"RETURN phải về A: {ret_missions}"
    print("   ✅ [16/16] Shot 2 tại trạm kho (B) tự động lập đường ngắn nhất trực tiếp tới đích mới (B->C) và giữ RETURN về A")

    # 17. Đổi danh mục sản phẩm & bố trí trạm đích động qua DB + WMSController + AI + FSM + Giữ cấu hình khi restart
    from app.controllers.wms_controller import WMSController
    wms_ctrl = WMSController(manager)
    agv_ctrl.handle_command({"action": "abort_missions"})
    agv_ctrl.handle_telemetry({
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "IDLE", "station": "A", "map_id": active_map_id
    })

    # (a) Kiểm tra class_id khớp best.pth (0=PKG_MECH, 1=PKG_ELEC, 2=PKG_TOOL) và get_products trả về target_node_id hiện tại
    code_gp, res_gp = wms_ctrl.get_products()
    assert code_gp == 200 and res_gp["map_id"] == active_map_id
    gp_map = {p["product_code"]: p for p in res_gp["products"]}
    assert gp_map["PKG_MECH"]["yolo_class_id"] == 0 and gp_map["PKG_MECH"]["target_node_id"] == "C"
    assert gp_map["PKG_ELEC"]["yolo_class_id"] == 1 and gp_map["PKG_ELEC"]["target_node_id"] == "B"
    assert gp_map["PKG_TOOL"]["yolo_class_id"] == 2 and gp_map["PKG_TOOL"]["target_node_id"] == "D"

    # (b) Đổi PKG_ELEC (Class 1) sang Kho D và đổi tên hiển thị -> thay thế trạm B cũ ngay lập tức (dù B gần hơn D: 1.51m < 1.54m)
    code_sp, res_sp = wms_ctrl.save_product({
        "product_code": "PKG_ELEC",
        "product_name": "Bo Mạch Nguồn Tùy Biến",
        "yolo_class_id": 1,
        "category": "ELECTRONIC",
        "target_node_id": "D",
    })
    assert code_sp == 200 and res_sp["status"] == "success", res_sp

    code_gp2, res_gp2 = wms_ctrl.get_products()
    gp2_map = {p["product_code"]: p for p in res_gp2["products"]}
    assert gp2_map["PKG_ELEC"]["target_node_id"] == "D", f"Phải đổi sang Kho D: {gp2_map['PKG_ELEC']}"
    assert gp2_map["PKG_ELEC"]["target_nodes"] == ["D"], f"Không được giữ luật B cũ: {gp2_map['PKG_ELEC']}"

    # (c) AI CV Ver3 (Class 1 = rectangular_box) & Camera Shot 1 phải tra DB ra tên mới + đi thẳng tới Kho D
    mock_box_ai = object.__new__(AIProductClassifierService)
    mock_box_ai.yolo_path = Path("yolo11n_v5b_best.pt")
    mock_box_ai.checkpoint_path = Path("best.pth")
    mock_box_ai.workflow_manager = manager
    mock_box_ai.is_placeholder_mode = False
    mock_box_ai.engine_name = "CV_Ver3_YOLO11n_MobileNetV3_Geometry"
    mock_box_ai.recognizer = SimpleNamespace(
        recognize_scene=lambda img_bgr: ([{
            "object_id": 1,
            "bbox": [10, 10, 110, 110],
            "class_name": "rectangular_box",
            "confidence": 0.96,
            "decision_type": "MOBILENET_DIRECT",
        }], img_bgr, 11.0)
    )
    cam_dyn = CameraController(manager, mock_box_ai)
    boundary_dyn = "----BoundaryDynAlloc"
    body_dyn = (
        f"--{boundary_dyn}\r\nContent-Disposition: form-data; name=\"station\"\r\n\r\nA\r\n"
        f"--{boundary_dyn}\r\nContent-Disposition: form-data; name=\"mode\"\r\n\r\nCAPTURE\r\n"
        f"--{boundary_dyn}\r\nContent-Disposition: form-data; name=\"image\"; filename=\"box.jpg\"\r\n"
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + real_jpg_bytes + f"\r\n--{boundary_dyn}--\r\n".encode("utf-8")
    code_dyn, resp_dyn = cam_dyn.handle_upload(f"multipart/form-data; boundary={boundary_dyn}", body_dyn)
    assert code_dyn == 200 and resp_dyn["status"] == "success"
    assert resp_dyn["ai_result"]["product_name"] == "Bo Mạch Nguồn Tùy Biến"
    assert resp_dyn["ai_result"]["target_station"] == "D"
    assert resp_dyn["dispatch"]["target_node"] == "D"
    assert resp_dyn["dispatch"]["command"]["cmd"] == "GO_TO:D", resp_dyn["dispatch"]
    agv_ctrl.handle_command({"action": "abort_missions"})

    # (d) Luật đa trạm / dự phòng qua save_rule: thêm Kho B làm kho phụ cho PKG_ELEC với priority=2 (khi D chưa đầy vẫn ưu tiên D prio=1)
    code_sr, _ = wms_ctrl.save_rule({
        "product_code": "PKG_ELEC",
        "product_category": "ELECTRONIC",
        "target_node_id": "B",
        "priority": 2,
    })
    assert code_sr == 200
    router_dyn = manager.get_router(active_map_id)
    alloc_prio1 = manager.alloc_repo.allocate_nearest_target_node(
        active_map_id, "PKG_ELEC", "ELECTRONIC", from_node="A", router=router_dyn
    )
    assert alloc_prio1["target_node_id"] == "D", f"Priority 1 (D) phải thắng Priority 2 (B) dù B gần hơn: {alloc_prio1}"

    # Khi Kho D đầy (30/30), tự động chuyển sang kho phụ B (priority=2)
    manager.inv_repo.update_stock(active_map_id, "D", "PKG_ELEC", delta=30)
    alloc_prio2 = manager.alloc_repo.allocate_nearest_target_node(
        active_map_id, "PKG_ELEC", "ELECTRONIC", from_node="A", router=router_dyn
    )
    assert alloc_prio2["target_node_id"] == "B", f"Khi D đầy phải chuyển sang kho phụ B: {alloc_prio2}"
    manager.inv_repo.update_stock(active_map_id, "D", "PKG_ELEC", delta=-30)

    # (e) Chặn trùng Model Class ID (400) và chặn xóa mặt hàng đang có lịch sử nhiệm vụ (400)
    code_dup_cls, _ = wms_ctrl.save_product({
        "product_code": "PKG_FOOD", "product_name": "Thực phẩm", "yolo_class_id": 1, "category": "FOOD", "target_node_id": "C"
    })
    assert code_dup_cls == 400, "Phải báo lỗi 400 khi trùng Model Class ID với PKG_ELEC"
    code_del_fk, _ = wms_ctrl.delete_product({"product_code": "PKG_ELEC"})
    assert code_del_fk == 400, "Phải báo lỗi 400 thân thiện thay vì crash FK khi xóa sản phẩm đã có task"

    # (f) seed_database(test_db, reset=False) khi khởi động lại server phải giữ nguyên sản phẩm & bố trí trạm D vừa sửa
    seed_database(test_db, reset=False)
    reloaded_gp = {p["product_code"]: p for p in WMSController(WMSWorkflowManager(test_db)).get_products()[1]["products"]}
    assert reloaded_gp["PKG_ELEC"]["product_name"] == "Bo Mạch Nguồn Tùy Biến"
    assert reloaded_gp["PKG_ELEC"]["target_node_id"] == "D"
    print("   ✅ [17/22] Danh mục sản phẩm & bố trí trạm động hoạt động end-to-end và bền vững qua restart")

    # 18. Map-as-Code Validator (tools/sync_map.py) & parity hash giữa JSON - Server - Firmware Header
    repo_root = Path(__file__).resolve().parent.parent.parent
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    from tools.sync_map import (
        validate_and_expand_map as sync_validate_map,
        compute_canonical_map_hash as sync_compute_hash,
    )
    from database.repositories import compute_map_hash as db_compute_hash

    for preset_name in ("saban_154_151.json", "map_exp_6nodes.json"):
        cfg_p = json.loads((repo_root / "config" / "maps" / preset_name).read_text(encoding="utf-8"))
        m_p, n_p, e_p = sync_validate_map(cfg_p)
        h_cli = sync_compute_hash(m_p["map_id"], n_p, e_p)
        h_db = manager.map_repo.get_map_hash(m_p["map_id"])
        assert h_cli == h_db, f"Hash mismatch {preset_name}: CLI={h_cli}, DB={h_db}"

    # Phát hiện đồ thị cụt một chiều / đứt trạm / dist_m bất hợp lý
    bad_unreach = {
        "map_id": "BAD_ISLAND", "name": "Bad Island", "width_m": 2.0, "length_m": 2.0, "home_node": "A",
        "nodes": [
            {"node_id": "A", "type": "INBOUND", "x": 0.0, "y": 0.0},
            {"node_id": "B", "type": "STATION", "x": 0.0, "y": 1.0},
            {"node_id": "Z", "type": "STATION", "x": 2.0, "y": 2.0},
        ],
        "segments": [{"from": "A", "to": "B", "dist_m": 1.0, "heading": 90.0, "two_way": True}],
    }
    for bad_cfg in (
        bad_unreach,
        {
            "map_id": "BAD_ONEWAY", "name": "Dead End", "width_m": 2.0, "length_m": 2.0, "home_node": "A",
            "nodes": [
                {"node_id": "A", "type": "INBOUND", "x": 0.0, "y": 0.0},
                {"node_id": "B", "type": "STATION", "x": 1.0, "y": 0.0},
            ],
            "segments": [{"from": "A", "to": "B", "dist_m": 1.0, "heading": 0.0, "two_way": False}],
        },
        {
            "map_id": "BAD_DIST", "name": "Bad Dist", "width_m": 3.0, "length_m": 3.0, "home_node": "A",
            "nodes": [
                {"node_id": "A", "type": "INBOUND", "x": 0.0, "y": 0.0},
                {"node_id": "B", "type": "STATION", "x": 2.0, "y": 0.0},
            ],
            "segments": [{"from": "A", "to": "B", "dist_m": 0.5, "heading": 0.0, "two_way": True}],
        },
    ):
        failed = False
        try:
            sync_validate_map(bad_cfg)
        except ValueError:
            failed = True
        assert failed is True, f"Validator phải từ chối map lỗi: {bad_cfg['map_id']}"
    print("   ✅ [18/22] Validator Map-as-Code & đồng bộ MAP_HASH CLI == DB hoạt động chính xác")

    # 19. set_active_map với ID không tồn tại phải giữ nguyên bản đồ hiện tại (không rơi về 0 active maps)
    assert manager.map_repo.set_active_map("NON_EXISTENT_MAP_999") is False
    cur_active = manager.map_repo.get_active_map()
    assert cur_active is not None and cur_active["map_id"] == active_map_id
    print("   ✅ [19/22] set_active_map không hợp lệ giữ nguyên active map hiện tại")

    # 20. Non-destructive UPSERT: cập nhật map KHÔNG làm mất tồn kho & allocation_rules; chặn xóa node đang còn hàng
    b_before = {it["product_code"]: it["quantity"] for it in manager.inv_repo.get_node_inventory(active_map_id, "B")}
    before_elec = b_before.get("PKG_ELEC", 0)
    manager.inv_repo.update_stock(active_map_id, "B", "PKG_ELEC", delta=3)
    expected_elec = before_elec + 3
    saban_cfg = json.loads((repo_root / "config" / "maps" / "saban_154_151.json").read_text(encoding="utf-8"))
    saban_updated = json.loads(json.dumps(saban_cfg))
    saban_updated["description"] = "Updated non-destructively"
    saban_updated["nodes"].append({"node_id": "E", "name": "Trạm Phụ E", "type": "STATION", "x": 1.54, "y": 0.75, "max_capacity": 20})
    saban_updated["segments"].append({"from": "C", "to": "E", "dist_m": 0.76, "two_way": True, "speed_limit": 25.0})
    manager.map_repo.upsert_map_config(saban_updated, activate=True)

    b_stock = {it["product_code"]: it["quantity"] for it in manager.inv_repo.get_node_inventory(active_map_id, "B")}
    assert b_stock.get("PKG_ELEC") == expected_elec, f"Tồn kho trạm B bị mất sau upsert_map_config: {b_stock}"
    rules_after = manager.alloc_repo.get_all_rules(active_map_id)
    assert len(rules_after) >= 4, f"Quy tắc phân bổ bị mất sau upsert_map_config: {rules_after}"

    # Thử xóa node B đang có tồn kho > 0 -> phải bị từ chối bằng ValueError & giữ nguyên DB
    saban_drop_b = json.loads(json.dumps(saban_updated))
    saban_drop_b["nodes"] = [n for n in saban_drop_b["nodes"] if n["node_id"] != "B"]
    saban_drop_b["segments"] = [
        {"from": "A", "to": "D", "dist_m": 1.54, "two_way": True, "speed_limit": 25.0},
        {"from": "D", "to": "C", "dist_m": 1.51, "two_way": True, "speed_limit": 25.0},
        {"from": "C", "to": "E", "dist_m": 0.76, "two_way": True, "speed_limit": 25.0},
    ]
    blocked_drop = False
    try:
        manager.map_repo.upsert_map_config(saban_drop_b, activate=True)
    except ValueError:
        blocked_drop = True
    assert blocked_drop is True, "Phải chặn xóa trạm B khi vẫn còn tồn kho > 0"
    b_stock_after = {it["product_code"]: it["quantity"] for it in manager.inv_repo.get_node_inventory(active_map_id, "B")}
    assert b_stock_after.get("PKG_ELEC") == expected_elec

    # Khôi phục SABAN_154_151 gốc (xóa node E rỗng thành công mà vẫn giữ tồn kho ở B)
    manager.map_repo.upsert_map_config(saban_cfg, activate=True)
    b_stock_final = {it["product_code"]: it["quantity"] for it in manager.inv_repo.get_node_inventory(active_map_id, "B")}
    assert b_stock_final.get("PKG_ELEC") == expected_elec
    manager.inv_repo.update_stock(active_map_id, "B", "PKG_ELEC", delta=-3)
    print("   ✅ [20/22] Non-destructive UPSERT bảo toàn inventory/rules & chặn xóa node còn hàng")

    # 21. Cổng Map-as-Code LOCK (403 mặc định) & chống đổi map giữa nhiệm vụ (409) khi mở dev mode
    import app.controllers.wms_controller as wms_ctrl_mod
    wms_lock_ctrl = WMSController(manager, agv_controller=agv_ctrl)
    code_403_sw, res_403_sw = wms_lock_ctrl.switch_map({"map_id": "MAP_EXP_6NODES"})
    code_403_im, _ = wms_lock_ctrl.import_map(saban_cfg)
    assert code_403_sw == 403 and res_403_sw.get("code") == "MAP_LOCKED_BY_CODE"
    assert code_403_im == 403

    orig_allow = wms_ctrl_mod.ALLOW_RUNTIME_MAP_CHANGE
    try:
        wms_ctrl_mod.ALLOW_RUNTIME_MAP_CHANGE = True
        agv_ctrl.trail = [[0.1, 0.2]]
        # Đang rảnh -> switch thành công & xóa trail cũ
        code_ok_sw, res_ok_sw = wms_lock_ctrl.switch_map({"map_id": "MAP_EXP_6NODES"})
        assert code_ok_sw == 200 and res_ok_sw["map"]["map_id"] == "MAP_EXP_6NODES"
        assert agv_ctrl.trail == [] and res_ok_sw["map_hash"] == manager.map_repo.get_map_hash("MAP_EXP_6NODES")

        # Đang có mission chạy -> trả về 409 Conflict
        agv_ctrl.handle_telemetry({
            "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
            "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A", "map_id": "MAP_EXP_6NODES"
        })
        busy_disp = manager.handle_inbound_package(map_id="MAP_EXP_6NODES", product_code="PKG_ELEC", ai_confidence=0.95)
        assert busy_disp["success"] is True
        code_409, _ = wms_lock_ctrl.switch_map({"map_id": active_map_id})
        assert code_409 == 409
        agv_ctrl.handle_command({"action": "abort_missions"})
    finally:
        wms_ctrl_mod.ALLOW_RUNTIME_MAP_CHANGE = orig_allow
    print("   ✅ [21/22] Khóa Map-as-Code 403 mặc định & chặn chuyển map khi đang chạy (409 Conflict)")

    # 22. Trạm gốc tùy biến không phải 'A' (home_node='H1' tại (1.0, 2.0)) + phát hiện lệch MAP_HASH
    custom_home_map = {
        "map_id": "CUSTOM_HOME_H1",
        "name": "Bản đồ Gốc H1",
        "width_m": 3.0,
        "length_m": 3.0,
        "home_node": "H1",
        "nodes": [
            {"node_id": "H1", "name": "Cổng Nhập H1", "type": "INBOUND", "x": 1.0, "y": 2.0, "max_capacity": 50},
            {"node_id": "S1", "name": "Kệ S1", "type": "STATION", "x": 2.5, "y": 2.0, "max_capacity": 30},
            {"node_id": "S2", "name": "Kệ S2", "type": "STATION", "x": 2.5, "y": 0.5, "max_capacity": 30},
        ],
        "segments": [
            {"from": "H1", "to": "S1", "dist_m": 1.5, "two_way": True, "speed_limit": 25.0},
            {"from": "S1", "to": "S2", "dist_m": 1.5, "two_way": True, "speed_limit": 25.0},
            {"from": "S2", "to": "H1", "dist_m": 2.12, "two_way": True, "speed_limit": 25.0},
        ],
        "default_rules": [
            {"product_category": "ELECTRONIC", "target_node_id": "S1", "priority": 1},
            {"product_category": "*", "target_node_id": "S2", "priority": 9},
        ],
    }
    manager.map_repo.upsert_map_config(custom_home_map, activate=True)
    assert manager.get_home_node("CUSTOM_HOME_H1") == "H1"

    # Kiểm tra map_hash mismatch warning khi telemetry gửi map/hash cũ
    agv_ctrl.handle_telemetry({
        "map_id": "SABAN_154_151", "map_hash": "deadbeef",
        "x": 1.0, "y": 2.0, "yaw": 0.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "H1"
    })
    dash_mismatch = agv_ctrl.get_dashboard_state({"online": False})
    assert dash_mismatch["map_sync"]["mismatch"] is True

    # Khi telemetry khớp hash -> mismatch=False & auto Shot 1 nhận diện được trạm gốc H1 (1.0, 2.0)
    expected_h1_hash = manager.map_repo.get_map_hash("CUSTOM_HOME_H1")
    _, tel_h1 = agv_ctrl.handle_telemetry({
        "map_id": "CUSTOM_HOME_H1", "map_hash": expected_h1_hash,
        "x": 1.0, "y": 2.0, "yaw": 0.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "H1"
    })
    assert tel_h1["home_node"] == "H1" and tel_h1["map_hash"] == expected_h1_hash
    dash_ok = agv_ctrl.get_dashboard_state({"online": True})
    assert dash_ok["map_sync"]["mismatch"] is False

    cam_h1 = CameraController(manager, ai_svc)
    cam_h1.agv_controller = agv_ctrl
    cam_h1.record_poll()
    cam_h1.first_poll_time = time.time() - 5.0
    agv_ctrl.camera_controller = cam_h1
    agv_ctrl.first_seen_time = time.time() - 5.0
    assert cam_h1.check_auto_inbound_capture(agv_ctrl) is True
    assert cam_h1.pending_cam_metadata.get("station") == "H1"

    # Dispatch từ H1 -> S1 rồi Shot 2 dỡ hàng hết phải sinh lệnh GO_TO:H1 (không phải GO_TO:A)
    disp_h1 = manager.handle_inbound_package(map_id="CUSTOM_HOME_H1", product_code="PKG_ELEC", ai_confidence=0.95)
    assert disp_h1["success"] is True and disp_h1["optimal_sequence"][0] == "H1" and disp_h1["target_node"] == "S1"
    assert disp_h1["optimal_sequence"][-1] == "H1" and disp_h1["first_mission"]["from_node"] == "H1"
    cmd_h1_id = disp_h1["command"]["cmd_id"]
    agv_ctrl.handle_telemetry({
        "map_id": "CUSTOM_HOME_H1", "map_hash": expected_h1_hash,
        "x": 2.5, "y": 2.0, "yaw": 0.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "AT_STATION", "station": "S1", "ack_id": cmd_h1_id
    })
    unload_h1 = manager.confirm_unloading_with_vision(
        b"", ai_svc, station="S1",
        ai_res={"decision_status": "NO_OBJECT", "product_code": "NONE", "detected_items": []}
    )
    assert unload_h1["status"] == "ALL_CARGO_UNLOADED_RETURNING"
    assert unload_h1["next_station"] == "H1"
    assert unload_h1["command"]["cmd"] == "GO_TO:H1"
    agv_ctrl.handle_command({"action": "abort_missions"})
    print("   ✅ [22/24] Hỗ trợ trạm gốc tùy biến (home_node='H1') end-to-end & cảnh báo lệch MAP_HASH")

    # 23. Chặn Shot 1 trùng lặp khi đang có Task/Mission giao hàng hoạt động hoặc AGV đang chạy (lỗi 02:34:45 -> 02:34:51)
    manager.map_repo.upsert_map_config(saban_cfg, activate=True)
    agv_ctrl.handle_telemetry({
        "map_id": active_map_id,
        "map_hash": manager.map_repo.get_map_hash(active_map_id),
        "x": 0.0, "y": 0.0, "yaw": 90.0, "speed": 0.0,
        "voltage": 12.4, "ina_ok": True, "state": "STOPPED", "station": "A"
    })
    cam_guard_ai = object.__new__(AIProductClassifierService)
    cam_guard_ai.yolo_path = Path("yolo11n_v5b_best.pt")
    cam_guard_ai.checkpoint_path = Path("best.pth")
    cam_guard_ai.workflow_manager = manager
    cam_guard_ai.is_placeholder_mode = False
    cam_guard_ai.engine_name = "CV_Ver3_YOLO11n_MobileNetV3_Geometry"
    cam_guard_ai.recognizer = SimpleNamespace(
        recognize_scene=lambda img_bgr: ([{
            "object_id": 1, "bbox": [10, 10, 110, 110],
            "class_name": "triangular_prism", "confidence": 0.98, "decision_type": "MOBILENET_DIRECT"
        }], img_bgr, 10.0)
    )
    cam_dup = CameraController(manager, cam_guard_ai)
    cam_dup.agv_controller = agv_ctrl
    agv_ctrl.camera_controller = cam_dup

    ct_dup = "multipart/form-data; boundary=----BoundaryDupShot1"
    body_dup = (
        "------BoundaryDupShot1\r\nContent-Disposition: form-data; name=\"station\"\r\n\r\nA\r\n"
        "------BoundaryDupShot1\r\nContent-Disposition: form-data; name=\"mode\"\r\n\r\nCAPTURE\r\n"
        "------BoundaryDupShot1\r\nContent-Disposition: form-data; name=\"image\"; filename=\"s1.jpg\"\r\n"
        "Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + real_jpg_bytes + b"\r\n------BoundaryDupShot1--\r\n"

    c_first, r_first = cam_dup.handle_upload(ct_dup, body_dup)
    assert c_first == 200 and r_first["status"] == "success", r_first
    first_tid = r_first["dispatch"]["task_id"]
    first_cmd_id = r_first["dispatch"]["command"]["cmd_id"]

    # Giả lập AGV đã nhận lệnh và đang chạy RUNNING_FAST (tốc độ > 35 cm/s) giữa đường A -> B
    agv_ctrl.handle_telemetry({
        "map_id": active_map_id,
        "x": 0.0, "y": 1.07, "yaw": 90.0, "speed": 35.4,
        "voltage": 12.2, "ina_ok": True, "state": "RUNNING_FAST", "station": "",
        "ack_id": first_cmd_id
    })
    tasks_cnt_before = len(manager.task_repo.get_recent_tasks(limit=20))
    c_cmd_blk, r_cmd_blk = agv_ctrl.handle_command({"action": "capture_now", "station": "A", "mode": "CAPTURE"})
    assert c_cmd_blk == 200 and r_cmd_blk.get("blocked") is True and cam_dup.pending_cam_cmd is None, r_cmd_blk

    c_dup, r_dup = cam_dup.handle_upload(ct_dup, body_dup)
    tasks_cnt_after = len(manager.task_repo.get_recent_tasks(limit=20))
    assert c_dup == 200 and r_dup.get("blocked") is True and r_dup["status"] == "warning", r_dup
    assert tasks_cnt_after == tasks_cnt_before, "Không được tạo task trùng khi Shot 1 đến lúc xe đang chạy!"
    assert manager.active_mission and manager.active_mission["task_id"] == first_tid
    assert manager.agv_fsm.pending_command is None, "Không được ghi đè pending_command giữa chừng!"
    agv_ctrl.handle_command({"action": "abort_missions"})
    print("   ✅ [23/24] Chặn thành công Shot 1 trùng lặp qua cả API capture_now và handle_upload khi AGV đang chạy")

    # 24. Kiểm thử tách phổ Hồng ấm (H=165..173) vs Đỏ (H<12 | H>=174) không tách nhầm khay kit đơn lẻ (lỗi Shot 2 @ B)
    try:
        from weights.slam_cv_recognizer import SLAMCVRecognizer as _CVRec
        import cv2 as _cv2
        import numpy as _np2

        # Tạo khung hình chứa duy nhất 1 khối Hồng ấm (H=167..168 trong OpenCV) giống Shot 2 tại Kho B
        warm_pink_hsv = _np2.full((600, 600, 3), (0, 0, 220), dtype=_np2.uint8)
        warm_pink_hsv[120:540, 200:460] = (167, 130, 140)
        warm_pink_bgr = _cv2.cvtColor(warm_pink_hsv, _cv2.COLOR_HSV2BGR)
        is_kit_wp, counts_wp = _CVRec._detect_physical_tray_kit(warm_pink_bgr, num_raw_boxes=1)
        assert is_kit_wp is False, f"Một khối Hồng ấm (H=167) không được kích hoạt nhầm kit đa vật thể: {counts_wp}"
        masks_wp = _CVRec._tray_color_masks(warm_pink_hsv)
        assert int(_np2.sum(masks_wp["rectangular_box"])) > 80000 and int(_np2.sum(masks_wp["cylinder"])) == 0
        print("   ✅ [24/24] Quang phổ Hồng ấm (H=165..173) không tách nhầm thành Đỏ/Kit đa vật thể")
    except ImportError:
        print("   ℹ️ [24/24] Bỏ qua kiểm thử OpenCV HSV trong môi trường không có cv2")

    # 25. Regression TSP tie-breaker (mission 12:19:25): tổng lộ trình hòa 6.10m giữa A->C->B->A và A->B->C->A
    # Khi camera liệt kê món đi C trước món đi B, xe phải ưu tiên dỡ hàng ở B trên đường đi (delivery_dist 3.05m < 4.59m)
    from database.db_connection import get_db as _get_tsp_db
    from database.seed_data import seed_database as _seed_tsp_db

    tsp_db = test_db.parent / "test_tsp_tiebreak_wms.db"
    if tsp_db.exists():
        tsp_db.unlink()
    try:
        _seed_tsp_db(tsp_db, reset=True)
        with _get_tsp_db(tsp_db) as conn:
            conn.execute(
                "UPDATE allocation_rules SET target_node_id = 'C' "
                "WHERE map_id = 'SABAN_154_151' AND (product_category = 'TOOL' OR product_code = 'PKG_TOOL')"
            )
            conn.execute(
                "UPDATE allocation_rules SET target_node_id = 'B' "
                "WHERE map_id = 'SABAN_154_151' AND (product_category = 'ELECTRONIC' OR product_code = 'PKG_ELEC')"
            )
        tsp_manager = WMSWorkflowManager(tsp_db, agv_id="AGV_Ver2")
        tsp_manager.handle_telemetry({
            "map_id": "SABAN_154_151", "x": 0.0, "y": 0.0, "yaw": 90.0, "dist": 0.0,
            "speed": 0.0, "state": "AT_STATION_A", "voltage": 12.4, "current": 0.2,
            "battery_pct": 95.0, "ack_id": 0
        })
        tsp_res = tsp_manager.plan_and_dispatch_mission_queue(
            map_id="SABAN_154_151",
            detected_items=[
                {"product_code": "PKG_TOOL", "confidence": 0.98},
                {"product_code": "PKG_ELEC", "confidence": 0.98},
            ],
            from_node="A",
        )
        assert tsp_res.get("success") is True, f"Dispatch TSP 25 failed: {tsp_res}"
        assert tsp_res.get("optimal_sequence") == ["A", "B", "C", "A"], (
            f"Expected ['A', 'B', 'C', 'A'], got {tsp_res.get('optimal_sequence')}"
        )
        assert tsp_res.get("total_distance_m") == 6.1, f"Expected 6.10m, got {tsp_res.get('total_distance_m')}"
        first_mission = tsp_res["first_mission"]
        assert first_mission["to_node"] == "B", f"Expected first drop at B, got {first_mission}"
        assert first_mission["route_str"] == "A->B", f"Expected route A->B, got {first_mission}"
        print("   ✅ [25/25] TSP hòa tổng quãng đường 6.10m ưu tiên dỡ tại B (A->B->C->A) thay vì chạy qua C trước")
    finally:
        if tsp_db.exists():
            try:
                tsp_db.unlink()
            except Exception:
                pass

    # 26. Kiểm thử chuỗi câu thoại lấy hàng & gỡ hàng đúng quy định (Voice Prompts Protocol)
    voice_db = test_db.parent / "test_voice_protocol_wms.db"
    if voice_db.exists():
        voice_db.unlink()
    try:
        _seed_tsp_db(voice_db, reset=True)
        with _get_tsp_db(voice_db) as conn:
            conn.execute(
                "UPDATE allocation_rules SET target_node_id = 'C' "
                "WHERE map_id = 'SABAN_154_151' AND (product_category = 'TOOL' OR product_code = 'PKG_TOOL')"
            )
            conn.execute(
                "UPDATE allocation_rules SET target_node_id = 'B' "
                "WHERE map_id = 'SABAN_154_151' AND (product_category = 'ELECTRONIC' OR product_code = 'PKG_ELEC')"
            )
        vm = WMSWorkflowManager(voice_db, agv_id="AGV_Ver2")
        vm.handle_telemetry({
            "map_id": "SABAN_154_151", "x": 0.0, "y": 0.0, "yaw": 90.0, "dist": 0.0,
            "speed": 0.0, "state": "AT_STATION_A", "station": "A", "voltage": 12.4, "ack_id": 0
        })
        assert vm.emit_voice_cue("WAIT_LOAD|A", min_interval_sec=28.0, dedupe_key="WAIT_LOAD") > 0
        assert vm.current_voice_cue == "WAIT_LOAD|A"
        assert vm.emit_voice_cue("RETRY_LOAD", min_interval_sec=8.0, dedupe_key="RETRY_LOAD") > 0
        assert vm.current_voice_cue == "RETRY_LOAD"

        d_multi = vm.plan_and_dispatch_mission_queue(
            map_id="SABAN_154_151",
            detected_items=[
                {"product_code": "PKG_ELEC", "confidence": 0.98},
                {"product_code": "PKG_TOOL", "confidence": 0.98},
            ],
            from_node="A",
        )
        assert d_multi.get("success") is True
        cmd_b = d_multi["command"]["cmd_id"]
        assert vm.current_voice_cue == "DISPATCH|B|MULTI", f"Got {vm.current_voice_cue}"
        seq_after_dispatch = vm.voice_seq

        vm.handle_telemetry({"map_id": "SABAN_154_151", "x": 0.0, "y": 0.5, "speed": 20.0, "state": "RUNNING_FAST", "station": "B", "voltage": 12.3, "ack_id": cmd_b})
        t_b = vm.handle_telemetry({"map_id": "SABAN_154_151", "x": 0.0, "y": 1.51, "speed": 0.0, "state": "AT_STATION", "station": "B", "voltage": 12.3, "ack_id": cmd_b})
        assert t_b["voice_cue"] == "UNLOAD_MULTI|B|PKG_ELEC", f"Expected UNLOAD_MULTI|B|PKG_ELEC, got {t_b}"
        assert t_b["voice_seq"] > seq_after_dispatch

        u_rev = vm.confirm_unloading_with_vision(
            image_bytes=b"", ai_service=None, station="B",
            ai_res={"decision_status": "NEEDS_REVIEW", "detected_items": []}
        )
        assert u_rev["status"] == "NEEDS_REVIEW" and vm.current_voice_cue == "RETRY_UNLOAD"

        u_still = vm.confirm_unloading_with_vision(
            image_bytes=b"", ai_service=None, station="B",
            ai_res={"decision_status": "CLASSIFIED", "product_code": "PKG_ELEC", "detected_items": [{"product_code": "PKG_ELEC", "confidence": 0.96}]}
        )
        assert u_still["status"] == "CARGO_STILL_PRESENT" and vm.current_voice_cue == "STILL_PRESENT|B|PKG_ELEC", f"Got u_still={u_still}, cue={vm.current_voice_cue}"

        u_adv = vm.confirm_unloading_with_vision(
            image_bytes=b"", ai_service=None, station="B",
            ai_res={"decision_status": "CLASSIFIED", "product_code": "PKG_TOOL", "detected_items": [{"product_code": "PKG_TOOL", "confidence": 0.96}]}
        )
        assert u_adv["status"] == "MISSION_ADVANCED" and vm.current_voice_cue == "ADVANCE|C|PKG_TOOL", f"Got u_adv={u_adv}, cue={vm.current_voice_cue}"
        cmd_c = u_adv["command"]["cmd_id"]

        vm.handle_telemetry({"map_id": "SABAN_154_151", "x": 0.7, "y": 1.51, "speed": 20.0, "state": "RUNNING_FAST", "station": "C", "voltage": 12.2, "ack_id": cmd_c})
        t_c = vm.handle_telemetry({"map_id": "SABAN_154_151", "x": 1.54, "y": 1.51, "speed": 0.0, "state": "AT_STATION", "station": "C", "voltage": 12.2, "ack_id": cmd_c})
        assert t_c["voice_cue"] == "UNLOAD_SINGLE|C|PKG_TOOL", f"Expected UNLOAD_SINGLE|C|PKG_TOOL, got {t_c}"

        u_ret = vm.confirm_unloading_with_vision(
            image_bytes=b"", ai_service=None, station="C",
            ai_res={"decision_status": "NO_OBJECT", "product_code": "NONE", "detected_items": []}
        )
        assert u_ret["status"] == "ALL_CARGO_UNLOADED_RETURNING" and vm.current_voice_cue == "RETURN_HOME|A"
        cmd_ret = u_ret["command"]["cmd_id"]

        t_home = vm.handle_telemetry({"map_id": "SABAN_154_151", "x": 0.0, "y": 0.0, "speed": 0.0, "state": "AT_STATION", "station": "A", "voltage": 12.2, "ack_id": cmd_ret})
        assert t_home["voice_cue"] == "HOME_READY|A", f"Expected HOME_READY|A, got {t_home}"
        assert vm.emit_voice_cue("WAIT_LOAD|A", min_interval_sec=28.0, dedupe_key="WAIT_LOAD") == 0
        print("   ✅ [26/26] Chuỗi câu thoại lấy hàng & gỡ hàng đúng quy định (WAIT_LOAD -> DISPATCH -> UNLOAD_MULTI -> STILL_PRESENT -> ADVANCE -> UNLOAD_SINGLE -> RETURN_HOME -> HOME_READY)")
    finally:
        if voice_db.exists():
            try:
                voice_db.unlink()
            except Exception:
                pass

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
