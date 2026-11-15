"""
===================================================================================
BỘ ĐIỀU PHỐI VẬN HÀNH TỔNG THỂ (WMS WORKFLOW & FSM MANAGER V2.1)
KẾT NỐI: AI INBOUND -> PHÂN BỔ KHO -> ĐỊNH TUYẾN LINE -> ĐIỀU PHỐI AGV -> TĂNG KHO
===================================================================================
"""

from typing import Dict, Any, Optional, List
import time
from pathlib import Path

import sys
CURRENT_DIR = Path(__file__).resolve().parent
DB_DIR = CURRENT_DIR.parent / "database"
if str(DB_DIR) not in sys.path:
    sys.path.insert(0, str(DB_DIR))

try:
    from database.db_connection import DB_PATH
    from database.repositories import (
        MapRepository,
        InventoryRepository,
        AllocationEngineRepository,
        ProductRepository,
        TaskRepository,
        FleetRepository,
        TelemetryRepository
    )
except ImportError:
    from db_connection import DB_PATH
    from repositories import (
        MapRepository,
        InventoryRepository,
        AllocationEngineRepository,
        ProductRepository,
        TaskRepository,
        FleetRepository,
        TelemetryRepository
    )

try:
    from .fsm_definitions import TaskState, AGVState
    from .router import TopologicalRouter
    from .task_fsm import TaskStateMachine
    from .agv_fsm import AGVStateMachine
except ImportError:
    from fsm_definitions import TaskState, AGVState
    from router import TopologicalRouter
    from task_fsm import TaskStateMachine
    from agv_fsm import AGVStateMachine

class WMSWorkflowManager:
    """Bộ điều phối trung tâm quản lý luồng khép kín của hệ thống"""
    def __init__(self, db_file: Path = DB_PATH, agv_id: str = "AGV_Ver2"):
        self.db_file = db_file
        self.agv_id = agv_id

        # Khởi tạo các Repositories
        self.map_repo = MapRepository(db_file)
        self.inv_repo = InventoryRepository(db_file)
        self.alloc_repo = AllocationEngineRepository(db_file)
        self.prod_repo = ProductRepository(db_file)
        self.task_repo = TaskRepository(db_file)
        self.fleet_repo = FleetRepository(db_file)
        self.telemetry_repo = TelemetryRepository(db_file)

        # Khởi tạo các State Machines
        self.task_fsm = TaskStateMachine(self.task_repo, self.inv_repo, self.map_repo)
        self.agv_fsm = AGVStateMachine(agv_id, self.fleet_repo)

        # Bộ đếm thời gian dỡ hàng (Dwell timer)
        self.active_unloading_tasks: Dict[int, float] = {} # task_id -> finish_time
        # Ánh xạ cmd_id -> task_id
        self.cmd_to_task: Dict[int, int] = {}

        # Hàng đợi nhiệm vụ đa trạm (Multi-stop Mission Queue)
        self.mission_queue: List[Dict[str, Any]] = []
        self.active_mission: Optional[Dict[str, Any]] = None
        self.completed_missions: List[Dict[str, Any]] = []

        # Tích hợp Camera Controller điều khiển chụp ảnh từ xa
        self.camera_controller = None
        self.last_cam_shot2_trigger: Dict[int, float] = {} # task_id -> last_trigger_timestamp

    def get_router(self, map_id: str) -> TopologicalRouter:
        """Tạo bộ định tuyến với cấu hình các đoạn line thực tế của bản đồ"""
        segments = self.map_repo.get_segments(map_id)
        return TopologicalRouter(segments)

    def plan_and_dispatch_mission_queue(
        self,
        map_id: str,
        detected_items: List[Dict[str, Any]],
        from_node: str = "A",
        image_path: str = ""
    ) -> Dict[str, Any]:
        """
        LẬP HÀNG ĐỢI NHIỆM VỤ ĐA TRẠM TỐI ƯU (MULTI-STOP MISSION QUEUE OPTIMIZATION):
          1. Kiểm tra an toàn xe AGV (Pin > 10.5V, xe ONLINE)
          2. Phân loại toàn bộ sản phẩm và ánh xạ tới trạm chuyên biệt:
             - Hộp chữ nhật (PKG_ELEC) -> Kho B
             - Trụ tròn (PKG_MECH) -> Kho C
             - Lăng trụ tam giác (PKG_TOOL) -> Kho D
          3. Sử dụng thuật toán Dijkstra tối ưu hoán vị hành trình (TSP Traveling Path):
             Tìm thứ tự ghé thăm các trạm sao cho tổng quãng đường di chuyển là NGẮN NHẤT.
          4. Khởi tạo danh sách Mission Queue (Hàng đợi nhiệm vụ liên tiếp)
          5. Khóa đoạn line chặng đầu và phát lệnh GO_TO trạm đầu tiên cho AGV
        """
        import itertools

        # 1. Kiểm tra an toàn xe
        agv = self.fleet_repo.get_agv(self.agv_id)
        if not agv or agv.get("status") == "OFFLINE":
            return {
                "success": False,
                "error": "AGV hiện đang ngoại tuyến (OFFLINE), không thể nhận lệnh!",
                "status": "AGV_OFFLINE"
            }
        voltage = float(agv.get("voltage_v", 0.0))
        if voltage < 6.8 or (8.8 < voltage < 10.5):
            min_v = 6.8 if voltage < 8.8 else 10.5
            return {
                "success": False,
                "error": f"Pin AGV quá thấp ({voltage}V < {min_v}V), từ chối nhận hàng!",
                "status": "BATTERY_TOO_LOW"
            }

        if not detected_items:
            return {
                "success": False,
                "error": "Không có sản phẩm nào trên khay hàng (NO_OBJECT)",
                "status": "NO_OBJECT"
            }

        router = self.get_router(map_id)

        # 2. Phân loại và gán trạm đích cho từng sản phẩm ("tại một điểm trạm sẽ có một loại hàng hóa")
        station_to_items: Dict[str, List[Dict[str, Any]]] = {}
        for item in detected_items:
            p_code = item.get("product_code", "PKG_ELEC")
            conf = float(item.get("confidence", 0.95))
            if conf < 0.50:
                continue

            target_st = item.get("target_station") or item.get("target_node")
            if not target_st:
                prod = self.prod_repo.get_by_code(p_code)
                alloc = self.alloc_repo.allocate_nearest_target_node(
                    map_id, p_code, prod["category"] if prod else "ELECTRONIC",
                    from_node=from_node, router=router, current_agv_id=self.agv_id
                )
                target_st = alloc["target_node_id"] if alloc else "B"

            if target_st not in station_to_items:
                station_to_items[target_st] = []
            station_to_items[target_st].append(item)

        if not station_to_items:
            # Nếu tất cả các sản phẩm đều có conf < 50%
            p_code = detected_items[0].get("product_code", "PKG_ELEC")
            task_id = self.task_repo.create_task(
                map_id=map_id, product_code=p_code,
                product_name="Không rõ", from_node=from_node, to_node="NONE", route_path="NONE",
                ai_confidence=float(detected_items[0].get("confidence", 0.0)), image_path=image_path,
                assigned_agv=self.agv_id
            )
            self.task_fsm.transition_to_rejected(task_id, reason="AI_CONFIDENCE_LOW")
            return {
                "success": False,
                "task_id": task_id,
                "status": TaskState.REJECTED_UNKNOWN.value,
                "message": "AI nhận diện không đạt độ tin cậy tối thiểu (Conf < 50%)"
            }

        target_stations = list(station_to_items.keys())

        # 3. Tìm đường đi tốt nhất (Dijkstra Permutation Optimization)
        best_perm = None
        min_total_dist = float('inf')
        best_legs = []

        for perm in itertools.permutations(target_stations):
            curr = from_node
            dist_sum = 0.0
            legs = []
            is_valid = True
            for next_st in perm:
                path = router.find_shortest_path(curr, next_st, current_agv_id=self.agv_id)
                if not path:
                    is_valid = False
                    break
                dist_sum += path["total_distance_m"]
                legs.append({"from": curr, "to": next_st, "path": path})
                curr = next_st

            # Thêm chặng quay về trạm xuất phát
            ret_path = router.find_shortest_path(curr, from_node, current_agv_id=self.agv_id)
            if ret_path:
                dist_sum += ret_path["total_distance_m"]
            else:
                is_valid = False

            if is_valid and dist_sum < min_total_dist:
                min_total_dist = dist_sum
                best_perm = perm
                best_legs = legs

        if not best_perm or not best_legs:
            return {
                "success": False,
                "error": "Không thể tìm được đường đi kết nối đến các trạm kho đích!",
                "status": "ROUTING_FAILED"
            }

        # 4. Khởi tạo Mission Queue
        self.mission_queue.clear()
        self.completed_missions.clear()

        for idx, leg in enumerate(best_legs):
            dest_st = leg["to"]
            items_for_st = station_to_items[dest_st]
            primary_item = items_for_st[0]
            p_code = primary_item["product_code"]
            p_name = primary_item.get("product_name") or p_code
            cat = primary_item.get("category", "GENERAL")

            task_id = self.task_repo.create_task(
                map_id=map_id,
                product_code=p_code,
                product_name=p_name,
                from_node=leg["from"],
                to_node=dest_st,
                route_path=leg["path"]["route_str"],
                ai_confidence=float(primary_item.get("confidence", 0.95)),
                image_path=image_path,
                assigned_agv=self.agv_id
            )

            mission = {
                "mission_id": idx + 1,
                "task_id": task_id,
                "from_node": leg["from"],
                "to_node": dest_st,
                "product_code": p_code,
                "product_name": p_name,
                "category": cat,
                "route_str": leg["path"]["route_str"],
                "distance_m": leg["path"]["total_distance_m"],
                "segments": leg["path"]["segments"],
                "items": items_for_st,
                "status": "QUEUED"
            }
            self.mission_queue.append(mission)

        # Thêm Mission Return về Trạm xuất phát sau khi dỡ xong hết
        last_st = best_perm[-1]
        ret_path = router.find_shortest_path(last_st, from_node, current_agv_id=self.agv_id)
        if ret_path and last_st != from_node:
            self.mission_queue.append({
                "mission_id": len(self.mission_queue) + 1,
                "task_id": 0,
                "from_node": last_st,
                "to_node": from_node,
                "product_code": "NONE",
                "product_name": "Quay về Trạm Tiếp Nhận A",
                "category": "RETURN",
                "route_str": ret_path["route_str"],
                "distance_m": ret_path["total_distance_m"],
                "segments": ret_path["segments"],
                "items": [],
                "status": "QUEUED"
            })

        # 5. Kích hoạt Mission đầu tiên trong hàng đợi
        first_mission = self.mission_queue[0]
        first_mission["status"] = "ACTIVE"
        self.active_mission = first_mission

        self.task_repo.update_status(first_mission["task_id"], TaskState.DISPATCHED.value)
        self.map_repo.lock_segments(map_id, first_mission["segments"], agv_id=self.agv_id)

        cmd_action = f"GO_TO:{first_mission['to_node']}"
        cmd_obj = self.agv_fsm.queue_command(cmd_action, {"task_id": first_mission["task_id"], "route": first_mission["route_str"]})
        self.cmd_to_task[cmd_obj["cmd_id"]] = first_mission["task_id"]

        optimal_sequence = [from_node] + list(best_perm) + [from_node]

        return {
            "success": True,
            "mode": "MISSION_QUEUE",
            "total_missions": len(self.mission_queue),
            "optimal_sequence": optimal_sequence,
            "total_distance_m": round(min_total_dist, 2),
            "mission_queue": self.mission_queue,
            "first_mission": first_mission,
            "target_node": first_mission["to_node"],
            "task_id": first_mission["task_id"],
            "route_path": first_mission["route_str"],
            "distance_m": first_mission["distance_m"],
            "locked_segments": first_mission["segments"],
            "command": cmd_obj
        }

    def handle_inbound_package(self, map_id: str, product_code: str, 
                               ai_confidence: float = 0.95, from_node: str = "A", 
                               image_path: str = "") -> Dict[str, Any]:
        """Tương thích đơn lẻ: đóng gói thành danh sách 1 sản phẩm và gọi plan_and_dispatch_mission_queue"""
        prod = self.prod_repo.get_by_code(product_code)
        item = {
            "product_code": product_code,
            "product_name": prod["product_name"] if prod else "Không rõ",
            "category": prod["category"] if prod else "ELECTRONIC",
            "confidence": ai_confidence
        }
        return self.plan_and_dispatch_mission_queue(
            map_id=map_id,
            detected_items=[item],
            from_node=from_node,
            image_path=image_path
        )

    def handle_telemetry(self, telemetry_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Nghiệp vụ xử lý gói tin Telemetry gửi lên từ ESP32-S3 (Tần số 5Hz - 10Hz):
          1. Cập nhật FSM của xe và lưu telemetry vào database
          2. Kiểm tra ACK id để chuyển trạng thái Task sang IN_TRANSIT
          3. Kiểm tra xe đến đích để chuyển Task sang DELIVERED và bắt đầu đếm Dwell time
          4. Kiểm tra hoàn tất Dwell time -> chuyển Task COMPLETED và TĂNG TỒN KHO (+1)
          5. Giải phóng khóa các đoạn line
          6. Phản hồi lệnh chờ gửi xuống xe nếu có
        """
        now = time.time()
        map_id = telemetry_data.get("map_id", "SABAN_154_151")
        reported_state = str(telemetry_data.get("state", "IDLE")).upper()
        ack_id = int(telemetry_data.get("ack_id", 0))

        # 1. Cập nhật FSM của xe
        agv_state = self.agv_fsm.update_from_telemetry(telemetry_data)

        # Cập nhật DB Fleet & Telemetry Log (Tự động hỗ trợ Pin 2S 6.4V-8.4V và 3S 9.9V-12.6V)
        v = float(telemetry_data.get("voltage", 0.0))
        b_pct = telemetry_data.get("battery_pct")
        if b_pct is None:
            if v <= 3.0:
                b_pct = 0.0
            elif v <= 8.8:
                # Đường cong xả Pin 2S Li-ion (6.4V = 0%, 8.4V = 100%)
                b_pct = max(0.0, min(100.0, round(((v - 6.4) / (8.4 - 6.4)) * 100.0, 1)))
            else:
                # Đường cong xả Pin 3S LiPo/Li-ion (9.9V = 0%, 12.6V = 100%)
                b_pct = max(0.0, min(100.0, round(((v - 9.9) / (12.6 - 9.9)) * 100.0, 1)))
        else:
            b_pct = float(b_pct)

        self.fleet_repo.update_telemetry(
            agv_id=self.agv_id,
            x=float(telemetry_data.get("x", 0.0)),
            y=float(telemetry_data.get("y", 0.0)),
            yaw=float(telemetry_data.get("yaw", 90.0)),
            speed=float(telemetry_data.get("speed", 0.0)),
            voltage=v,
            current=float(telemetry_data.get("current", 0.0)),
            battery_pct=b_pct,
            status=agv_state.value,
            ack_id=ack_id
        )

        # 2. Xử lý các Task đang chạy
        recent_tasks = self.task_repo.get_recent_tasks(limit=5)
        for task in recent_tasks:
            t_id = task["task_id"]
            t_status = task["status"]

            # (a) Nếu task đang DISPATCHED mà xe đã ACK -> chuyển IN_TRANSIT
            is_acked = (ack_id in self.cmd_to_task and self.cmd_to_task[ack_id] == t_id) or (ack_id > 0 and task.get("assigned_agv") == self.agv_id)
            if t_status == TaskState.DISPATCHED.value and is_acked:
                self.task_fsm.transition_to_in_transit(t_id)

            # (b) Nếu xe báo đã đến trạm (AT_STATION) và task đang IN_TRANSIT -> chuyển DELIVERED
            if t_status == TaskState.IN_TRANSIT.value and "STATION" in reported_state:
                self.task_fsm.transition_to_delivered(t_id)
                dest_st = task.get("to_node", "")
                if self.camera_controller:
                    # Kích hoạt Shot 2 tự động: Phát lệnh chụp khay hàng để kiểm tra dỡ hàng
                    self.camera_controller.queue_command(
                        "CAPTURE_NOW",
                        metadata={"station": dest_st, "mode": "VERIFY_UNLOAD", "task_id": t_id}
                    )
                    self.last_cam_shot2_trigger[t_id] = now
                else:
                    # Fallback dwell timer nếu không kích hoạt camera thị giác
                    self.active_unloading_tasks[t_id] = now + float(task.get("dwell_time_sec", 5.0))

            # (c) Nếu xe đang dừng đỗ tại trạm (DELIVERED/UNLOADING) và dùng Camera xác nhận:
            # Tự động chụp lại sau mỗi 6.0 giây nếu hàng chưa dỡ hoặc chưa có ảnh mới
            if t_status in (TaskState.DELIVERED.value, TaskState.UNLOADING.value) and self.camera_controller:
                last_trig = self.last_cam_shot2_trigger.get(t_id, 0.0)
                has_pending = bool(getattr(self.camera_controller, "pending_cam_cmd", None))
                if not has_pending and (now - last_trig >= 6.0):
                    dest_st = task.get("to_node", "")
                    self.camera_controller.queue_command(
                        "CAPTURE_NOW",
                        metadata={"station": dest_st, "mode": "VERIFY_UNLOAD", "task_id": t_id}
                    )
                    self.last_cam_shot2_trigger[t_id] = now

        # 3. Kiểm tra các Task đang dỡ hàng đã hết thời gian Dwell time chưa
        finished_tasks = []
        for t_id, finish_time in self.active_unloading_tasks.items():
            if now >= finish_time:
                # Hoàn tất dỡ hàng: Chuyển COMPLETED và TĂNG TỒN KHO (+1)!
                self.task_fsm.transition_to_completed(t_id)
                # Mở khóa các đoạn line của bản đồ
                t_obj = self.task_repo.get_by_id(t_id)
                if t_obj:
                    # Giải phóng toàn bộ khóa do xe này nắm giữ
                    segments = self.map_repo.get_segments(t_obj["map_id"])
                    segs_to_unlock = [s["segment_id"] for s in segments if s.get("locked_by_agv") == self.agv_id]
                    self.map_repo.unlock_segments(t_obj["map_id"], segs_to_unlock)
                finished_tasks.append(t_id)

        for t_id in finished_tasks:
            del self.active_unloading_tasks[t_id]
            # Nếu có active_mission tương ứng với task vừa dỡ xong qua Dwell timer:
            if self.active_mission and self.active_mission.get("task_id") == t_id:
                self.active_mission["status"] = "COMPLETED"
                self.completed_missions.append(self.active_mission)
                self.active_mission = None

                # Lấy mission tiếp theo trong queue (kể cả mission Return về A)
                next_mission = None
                for m in self.mission_queue:
                    if m["status"] == "QUEUED":
                        next_mission = m
                        break

                if next_mission:
                    next_mission["status"] = "ACTIVE"
                    self.active_mission = next_mission
                    next_to = next_mission["to_node"]
                    next_task_id = next_mission.get("task_id", 0)

                    if next_task_id > 0:
                        self.task_repo.update_status(next_task_id, TaskState.DISPATCHED.value)

                    if next_mission.get("segments"):
                        self.map_repo.lock_segments(map_id, next_mission["segments"], agv_id=self.agv_id)

                    cmd_action = f"GO_TO:{next_to}"
                    cmd_obj = self.agv_fsm.queue_command(cmd_action, {
                        "task_id": next_task_id,
                        "route": next_mission.get("route_str", ""),
                        "mission_id": next_mission["mission_id"]
                    })
                    if next_task_id > 0:
                        self.cmd_to_task[cmd_obj["cmd_id"]] = next_task_id

        # Kiểm tra hoàn thành Mission Return khi xe đã quay về Trạm Tiếp Nhận A:
        reported_station = str(telemetry_data.get("station", "")).upper()
        if self.active_mission and self.active_mission.get("to_node") == "A" and self.active_mission.get("category") == "RETURN":
            is_return_acked = (ack_id in self.cmd_to_task) or (self.agv_fsm.pending_command is None) or (ack_id > 0)
            if ("STATION" in reported_state or reported_station == "A" or reported_state in ("IDLE", "STOPPED")) and is_return_acked:
                self.active_mission["status"] = "COMPLETED"
                self.completed_missions.append(self.active_mission)
                self.active_mission = None
                self.map_repo.unlock_all_by_agv(self.agv_id, map_id)
                self.agv_fsm.current_state = AGVState.IDLE
                print(f"🎯 [FSM MANAGER]: AGV {self.agv_id} đã quay trở về Trạm Tiếp Nhận A an toàn! Khay hàng trống sẵn sàng cho chu trình nhận hàng mới.")

        # Giải phóng khóa đường mồ côi nếu xe không còn Task nào đang hoạt động (dùng lại recent_tasks tránh truy vấn lặp)
        has_active_task = any(
            t["status"] in (
                TaskState.DISPATCHED.value,
                TaskState.IN_TRANSIT.value,
                TaskState.DELIVERED.value,
                TaskState.UNLOADING.value
            )
            for t in recent_tasks
        )
        if not has_active_task and not self.active_unloading_tasks and reported_state in ("STOPPED", "AT_STATION", "IDLE"):
            self.map_repo.unlock_all_by_agv(self.agv_id, map_id)

        # 4. Chuẩn bị gói lệnh phản hồi cho ESP32 (Kèm thông số kích thước Map đang kích hoạt để dẫn đường)
        active_map_cfg = self.map_repo.get_active_map() or {}
        map_w = float(active_map_cfg.get("width_m", 1.54))
        map_h = float(active_map_cfg.get("length_m", 1.51))
        cmd_obj = self.agv_fsm.pending_command
        return {
            "status": "ok",
            "agv_state": agv_state.value,
            "map_w": map_w,
            "map_h": map_h,
            "cmd": cmd_obj["cmd"] if cmd_obj else None,
            "cmd_id": cmd_obj["cmd_id"] if cmd_obj else 0
        }

    def confirm_unloading_with_vision(self, image_bytes: bytes, ai_service, 
                                      station: str = "", task_id: Optional[int] = None,
                                      ai_res: Optional[Dict[str, Any]] = None,
                                      product_hint: str = "") -> Dict[str, Any]:
        """
        Quy trình xác nhận dỡ hàng bằng thị giác AI (Shot 2 Unload Verification):
          1. AI kiểm tra ảnh khay hàng (YOLO + MobileNetV3)
          2. Kiểm tra xem mặt hàng chuyên biệt của trạm này đã được dỡ xuống chưa
          3. Nếu đã dỡ:
             - Chuyển Task sang COMPLETED
             - Cập nhật tăng tồn kho (+1) tại kho đích
             - Giải phóng các đoạn line đã khóa của chặng hiện tại
             - Tự động lấy MISSION TIẾP THEO trong hàng đợi và phát lệnh GO_TO
             - Nếu đã dỡ hết toàn bộ hàng trong hàng đợi -> Tự động quay về Trạm A
          4. Nếu mặt hàng của trạm này VẪN CÒN trên xe:
             - Cảnh báo hàng chưa dỡ xong
             - Xe tiếp tục giữ phanh đỗ tại trạm, TUYỆT ĐỐI không cho chạy
             - Tồn kho chưa tăng
        """
        now = time.time()
        # 1. Gọi AI phân loại ảnh nếu chưa truyền kết quả sẵn
        if ai_res is None:
            ai_res = ai_service.classify_image(image_bytes, product_hint=product_hint)
        if isinstance(ai_res, dict) and "annotated_bytes" in ai_res:
            ai_res = {k: v for k, v in ai_res.items() if k != "annotated_bytes"}

        dec_status = ai_res.get("decision_status", "UNKNOWN")
        detected_items = ai_res.get("detected_items", [])
        remaining_codes = [i["product_code"] for i in detected_items]

        # 2. Xác định nhiệm vụ hiện tại tại trạm này
        cur_mission = self.active_mission
        expected_product = ""
        expected_prod_name = ""
        dest_node = station
        t_id = task_id

        if cur_mission and (not station or cur_mission.get("to_node") == station):
            expected_product = cur_mission.get("product_code", "")
            expected_prod_name = cur_mission.get("product_name", expected_product)
            dest_node = cur_mission.get("to_node", station)
            t_id = cur_mission.get("task_id", task_id)
        else:
            # Tra cứu task đang chờ dỡ hàng
            recent_tasks = self.task_repo.get_recent_tasks(limit=10)
            for t in recent_tasks:
                if t["status"] in (TaskState.DELIVERED.value, TaskState.UNLOADING.value, TaskState.IN_TRANSIT.value):
                    if not station or t.get("to_node") == station:
                        t_id = t["task_id"]
                        expected_product = t["product_code"]
                        expected_prod_name = t["product_name"]
                        dest_node = t["to_node"]
                        break

        # 3. Kiểm tra kiện hàng của trạm này đã được dỡ xuống chưa:
        is_empty_tray = (dec_status == "NO_OBJECT" or not detected_items or ai_res.get("product_code") in ("NONE", "NO_OBJECT", None, ""))
        cargo_unloaded = is_empty_tray or (expected_product and expected_product not in remaining_codes)

        if not cargo_unloaded:
            # Hàng của trạm này VẪN CÒN trên xe!
            if t_id and t_id in self.active_unloading_tasks:
                self.active_unloading_tasks[t_id] = now + 120.0

            return {
                "success": False,
                "unloaded": False,
                "status": "CARGO_STILL_PRESENT",
                "message": f"Cảnh báo: Kiện hàng '{expected_prod_name}' ({expected_product}) vẫn còn trên xe tại trạm {dest_node}. Xe tiếp tục dừng đỗ chờ dỡ hàng, không được phép di chuyển sang trạm kế tiếp!",
                "task_id": t_id,
                "station": dest_node,
                "expected_product": expected_product,
                "remaining_products": remaining_codes,
                "ai_result": ai_res
            }

        # 4. Xác nhận dỡ hàng thành công cho trạm hiện tại
        if t_id:
            self.task_fsm.transition_to_completed(t_id)
            if t_id in self.active_unloading_tasks:
                del self.active_unloading_tasks[t_id]
            if t_id in self.last_cam_shot2_trigger:
                del self.last_cam_shot2_trigger[t_id]

        if cur_mission:
            cur_mission["status"] = "COMPLETED"
            self.completed_missions.append(cur_mission)

        # Mở khóa các đoạn line của chặng vừa xong
        active_map_cfg = self.map_repo.get_active_map() or {}
        map_id = active_map_cfg.get("map_id", "SABAN_154_151")
        segments = self.map_repo.get_segments(map_id)
        segs_to_unlock = [s["segment_id"] for s in segments if s.get("locked_by_agv") == self.agv_id]
        if segs_to_unlock:
            self.map_repo.unlock_segments(map_id, segs_to_unlock)

        # 5. Kiểm tra Hàng đợi Mission tiếp theo & Cơ chế Quay Về Trạm A khi Hết Hàng
        if is_empty_tray:
            # =========================================================================
            # CƠ CHẾ 1: KHAY HÀNG ĐÃ TRỐNG HOÀN TOÀN (HẾT HÀNG TRÊN XE)
            # Ngay lập tức hủy các chặng giao còn lại (nếu có) và điều hướng quay về Trạm A
            # =========================================================================
            for m in self.mission_queue:
                if m["status"] == "QUEUED" and m.get("category") != "RETURN":
                    m["status"] = "SKIPPED_EMPTY_TRAY"
                    if m.get("task_id", 0) > 0:
                        self.task_repo.update_status(m["task_id"], TaskState.COMPLETED.value)

            # Nếu xe đang ở sẵn Trạm A -> Hoàn tất toàn bộ chu trình
            if dest_node == "A":
                self.active_mission = None
                self.map_repo.unlock_all_by_agv(self.agv_id, map_id)
                self.agv_fsm.current_state = AGVState.IDLE
                return {
                    "success": True,
                    "unloaded": True,
                    "status": "COMPLETED_AT_HOME",
                    "message": "Khay hàng đã trống hoàn toàn và AGV đang ở sẵn Trạm A. Chu trình hoàn tất, sẵn sàng cho đợt hàng mới!",
                    "station": "A",
                    "next_station": "A",
                    "ai_result": ai_res
                }

            # Lập lộ trình tối ưu từ trạm hiện tại quay về Trạm A
            router = TopologicalRouter(self.map_repo.get_segments(map_id))
            ret_path = router.find_shortest_path(dest_node, "A", current_agv_id=self.agv_id)
            ret_segments = ret_path["segments"] if ret_path else []
            ret_route = ret_path["route_str"] if ret_path else f"{dest_node}->A"
            ret_dist = ret_path["total_distance_m"] if ret_path else 0.0

            # Khóa các đoạn line trên đường quay về
            if ret_segments:
                self.map_repo.lock_segments(map_id, ret_segments, agv_id=self.agv_id)

            # Tìm hoặc tạo Return Mission trong hàng đợi
            ret_mission = None
            for m in self.mission_queue:
                if m.get("category") == "RETURN" and m["status"] in ("QUEUED", "ACTIVE"):
                    ret_mission = m
                    break

            if not ret_mission:
                ret_mission = {
                    "mission_id": len(self.mission_queue) + 1,
                    "task_id": 0,
                    "from_node": dest_node,
                    "to_node": "A",
                    "product_code": "NONE",
                    "product_name": "Quay về Trạm Tiếp Nhận A",
                    "category": "RETURN",
                    "route_str": ret_route,
                    "distance_m": ret_dist,
                    "segments": ret_segments,
                    "items": [],
                    "status": "ACTIVE"
                }
                self.mission_queue.append(ret_mission)
            else:
                ret_mission["from_node"] = dest_node
                ret_mission["to_node"] = "A"
                ret_mission["route_str"] = ret_route
                ret_mission["segments"] = ret_segments
                ret_mission["status"] = "ACTIVE"

            self.active_mission = ret_mission

            # Phát lệnh GO_TO:A về cho xe AGV
            cmd_action = "GO_TO:A"
            cmd_obj = self.agv_fsm.queue_command(cmd_action, {
                "task_id": 0,
                "mission_id": ret_mission["mission_id"],
                "route": ret_route,
                "mode": "RETURN_HOME",
                "unloaded": True
            })

            return {
                "success": True,
                "unloaded": True,
                "status": "ALL_CARGO_UNLOADED_RETURNING",
                "message": f"Toàn bộ hàng đã được dỡ xuống! Khay hàng trống 100%. Xe AGV tự động kích hoạt lộ trình quay về Trạm Tiếp Nhận A ({ret_route}).",
                "station": dest_node,
                "next_station": "A",
                "next_mission": ret_mission,
                "command": cmd_obj,
                "ai_result": ai_res
            }

        # =========================================================================
        # CƠ CHẾ 2: TRÊN XE VẪN CÒN HÀNG CHO CÁC TRẠM KẾ TIẾP
        # Tiến tới Mission tiếp theo trong hàng đợi
        # =========================================================================
        next_mission = None
        for m in self.mission_queue:
            if m["status"] == "QUEUED":
                next_mission = m
                break

        if next_mission:
            next_mission["status"] = "ACTIVE"
            self.active_mission = next_mission
            next_to = next_mission["to_node"]
            next_task_id = next_mission.get("task_id", 0)

            if next_task_id > 0:
                self.task_repo.update_status(next_task_id, TaskState.DISPATCHED.value)

            # Khóa đoạn line chặng tiếp
            if next_mission.get("segments"):
                self.map_repo.lock_segments(map_id, next_mission["segments"], agv_id=self.agv_id)

            # Phát lệnh di chuyển sang trạm kế tiếp
            cmd_action = f"GO_TO:{next_to}"
            cmd_obj = self.agv_fsm.queue_command(cmd_action, {
                "task_id": next_task_id,
                "route": next_mission.get("route_str", ""),
                "mission_id": next_mission["mission_id"]
            })
            if next_task_id > 0:
                self.cmd_to_task[cmd_obj["cmd_id"]] = next_task_id

            is_return = (next_mission.get("category") == "RETURN" or next_mission.get("product_code") == "NONE")
            msg = f"Đã dỡ toàn bộ hàng trên xe! AGV nhận lệnh quay trở về Trạm {next_to}." if is_return else f"Xác nhận đã dỡ {expected_prod_name} tại Trạm {dest_node}! Tồn kho +1. Xe được cấp lệnh chạy sang Trạm {next_to} ({next_mission['product_name']})."

            return {
                "success": True,
                "unloaded": True,
                "status": "ALL_CARGO_UNLOADED_RETURNING" if is_return else "MISSION_ADVANCED",
                "message": msg,
                "station": dest_node,
                "next_station": next_to,
                "next_mission": next_mission,
                "command": cmd_obj,
                "ai_result": ai_res
            }
        else:
            # Fallback nếu không còn mission nào -> Lập đường về A
            router = TopologicalRouter(self.map_repo.get_segments(map_id))
            ret_path = router.find_shortest_path(dest_node, "A", current_agv_id=self.agv_id)
            ret_segments = ret_path["segments"] if ret_path else []
            ret_route = ret_path["route_str"] if ret_path else f"{dest_node}->A"
            if ret_segments:
                self.map_repo.lock_segments(map_id, ret_segments, agv_id=self.agv_id)

            cmd_action = "GO_TO:A"
            cmd_obj = self.agv_fsm.queue_command(cmd_action, {
                "mode": "RETURN_HOME", 
                "unloaded": True,
                "route": ret_route
            })
            return {
                "success": True,
                "unloaded": True,
                "status": "ALL_CARGO_UNLOADED_RETURNING",
                "message": f"Xác nhận dỡ hàng thành công! Khay hàng trống. Tồn kho tại trạm {dest_node} đã tăng +1. Xe được cấp lệnh quay về Trạm A ({ret_route}).",
                "station": dest_node,
                "next_station": "A",
                "command": cmd_obj,
                "ai_result": ai_res
            }

    def get_mission_queue_status(self) -> Dict[str, Any]:
        """Lấy trạng thái hiện tại của Hàng đợi Mission"""
        return {
            "active_mission": self.active_mission,
            "mission_queue": self.mission_queue,
            "completed_missions": self.completed_missions,
            "total_missions": len(self.mission_queue),
            "completed_count": len(self.completed_missions),
            "remaining_count": sum(1 for m in self.mission_queue if m["status"] == "QUEUED")
        }
