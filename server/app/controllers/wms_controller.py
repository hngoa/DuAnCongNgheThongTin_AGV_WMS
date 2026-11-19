"""
===================================================================================
CONTROLLER QUẢN LÝ KHO WMS & XUẤT DỮ LIỆU BÁO CÁO (WMS CONTROLLER)
CUNG CẤP API BẢN ĐỒ LINE, TỒN KHO ĐA SẢN PHẨM, DANH SÁCH TASK VÀ XUẤT CSV
===================================================================================
"""

from typing import Dict, Any, Tuple, Optional
import csv
import io
import sqlite3
from datetime import datetime
from app.config import ALLOW_RUNTIME_MAP_CHANGE, DEFAULT_MAP_ID

class WMSController:
    def __init__(self, workflow_manager, agv_controller=None):
        self.workflow_manager = workflow_manager
        self.agv_controller = agv_controller

    def _has_active_work(self) -> Optional[str]:
        wf = self.workflow_manager
        if wf.active_mission is not None:
            return f"AGV đang thực thi mission tới Trạm {wf.active_mission.get('to_node', '?')}"
        if getattr(wf, "active_unloading_tasks", None):
            return "Đang có tác vụ chờ xác nhận dỡ hàng tại trạm đích"
        if getattr(wf, "manual_route_target", None):
            return f"AGV đang chạy lộ trình thủ công tới Trạm {wf.manual_route_target}"
        recent_tasks = wf.task_repo.get_recent_tasks(limit=10)
        busy_task = next(
            (t for t in recent_tasks if t["status"] in ("DISPATCHED", "IN_TRANSIT", "DELIVERED", "UNLOADING")),
            None,
        )
        if busy_task:
            return f"Task #{busy_task['task_id']} đang ở trạng thái {busy_task['status']}"
        return None

    def get_map_data(self, map_id: Optional[str] = None) -> Tuple[int, Dict[str, Any]]:
        """Trả về thông tin bản đồ hiện hoạt (hoặc map_id chỉ định), nodes, segments và map_hash."""
        repo = self.workflow_manager.map_repo
        active_map = repo.get_active_map() or {"map_id": DEFAULT_MAP_ID, "name": "Bản đồ Mặc định"}
        if map_id:
            cur_map_id = str(map_id).strip().upper()
            map_meta = repo.get_map_by_id(cur_map_id)
            if not map_meta:
                return 404, {"status": "error", "message": f"Không tìm thấy bản đồ '{cur_map_id}'"}
        else:
            cur_map_id = str(active_map.get("map_id", DEFAULT_MAP_ID)).strip().upper()
            map_meta = active_map

        nodes = repo.get_nodes(cur_map_id)
        segments = repo.get_segments(cur_map_id)
        map_hash = repo.get_map_hash(cur_map_id)
        home_node = self.workflow_manager.get_home_node(cur_map_id)
        return 200, {
            "status": "success",
            "map": map_meta,
            "nodes": nodes,
            "segments": segments,
            "map_hash": map_hash,
            "home_node": home_node,
            "allow_runtime_map_change": ALLOW_RUNTIME_MAP_CHANGE,
        }

    def get_all_maps(self) -> Tuple[int, Dict[str, Any]]:
        """Lấy danh sách tất cả các bản đồ có trong hệ thống kèm trạng thái Map-as-Code."""
        repo = self.workflow_manager.map_repo
        maps = repo.get_all_maps()
        active_map = repo.get_active_map()
        active_id = active_map["map_id"] if active_map else DEFAULT_MAP_ID
        return 200, {
            "status": "success",
            "active_map_id": active_id,
            "active_map_hash": repo.get_map_hash(active_id),
            "active_home_node": self.workflow_manager.get_home_node(active_id),
            "allow_runtime_map_change": ALLOW_RUNTIME_MAP_CHANGE,
            "maps": maps,
        }

    def switch_map(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Chuyển đổi bản đồ hoạt động (chỉ cho phép khi AGV_ALLOW_RUNTIME_MAP_CHANGE=true và không có nhiệm vụ chạy)."""
        if not ALLOW_RUNTIME_MAP_CHANGE:
            return 403, {
                "status": "error",
                "code": "MAP_LOCKED_BY_CODE",
                "message": "Bản đồ đang KHÓA theo cơ chế Map-as-Code. Vui lòng cập nhật config/active_map.json và chạy: python tools/sync_map.py --map <file> --apply-db",
            }

        target_map_id = str(body_json.get("map_id", "")).strip().upper()
        if not target_map_id:
            return 400, {"status": "error", "message": "Thiếu mã bản đồ (map_id)"}

        busy_reason = self._has_active_work()
        if busy_reason:
            return 409, {"status": "error", "message": f"Không thể chuyển bản đồ lúc này: {busy_reason}"}

        repo = self.workflow_manager.map_repo
        if not repo.get_map_by_id(target_map_id):
            return 404, {"status": "error", "message": f"Không tìm thấy bản đồ '{target_map_id}'"}

        old_active = repo.get_active_map()
        if old_active:
            repo.unlock_all_by_agv(self.workflow_manager.agv_id, old_active["map_id"])

        success = repo.set_active_map(target_map_id, sync_fleet=True)
        if not success:
            return 500, {"status": "error", "message": "Không thể chuyển bản đồ"}

        if self.agv_controller is not None:
            self.agv_controller.trail = []
        self.workflow_manager.last_reported_station = self.workflow_manager.get_home_node(target_map_id)
        return self.get_map_data(target_map_id)

    def import_map(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Nạp cấu hình bản đồ JSON (chỉ cho phép khi AGV_ALLOW_RUNTIME_MAP_CHANGE=true và không có nhiệm vụ chạy)."""
        if not ALLOW_RUNTIME_MAP_CHANGE:
            return 403, {
                "status": "error",
                "code": "MAP_LOCKED_BY_CODE",
                "message": "Nạp bản đồ trực tiếp từ Web UI đang KHÓA (Map-as-Code). Hãy chỉnh file JSON trong config/maps/ và chạy python tools/sync_map.py",
            }

        if not body_json:
            return 400, {"status": "error", "message": "Dữ liệu JSON rỗng"}

        busy_reason = self._has_active_work()
        if busy_reason:
            return 409, {"status": "error", "message": f"Không thể cập nhật bản đồ lúc này: {busy_reason}"}

        repo = self.workflow_manager.map_repo
        old_active = repo.get_active_map()
        try:
            res = repo.upsert_map_config(body_json, activate=True)
        except ValueError as e:
            return 400, {"status": "error", "message": str(e)}

        if old_active and old_active["map_id"] != res["map_id"]:
            repo.unlock_all_by_agv(self.workflow_manager.agv_id, old_active["map_id"])
        if self.agv_controller is not None:
            self.agv_controller.trail = []
        self.workflow_manager.last_reported_station = res["home_node"]
        return self.get_map_data(res["map_id"])

    def export_map(self, map_id: Optional[str] = None) -> Tuple[int, Dict[str, Any]]:
        """Xuất file JSON cấu hình của bản đồ"""
        active_map = self.workflow_manager.map_repo.get_active_map()
        target_id = str(map_id).strip().upper() if map_id else (active_map["map_id"] if active_map else DEFAULT_MAP_ID)
        cfg = self.workflow_manager.map_repo.export_map_config(target_id)
        if not cfg:
            return 404, {"status": "error", "message": "Không tìm thấy bản đồ"}
        return 200, {"status": "success", "config": cfg}

    def get_map_template(self) -> Tuple[int, Dict[str, Any]]:
        """Trả về mẫu JSON chuẩn Map-as-Code để người dùng tham khảo hoặc dùng với tools/sync_map.py"""
        template = {
            "map_id": "MAP_CUSTOM_01",
            "name": "Bản Đồ Nhà Kho Mới 2.0m x 1.5m",
            "width_m": 2.0,
            "length_m": 1.5,
            "home_node": "A",
            "description": "Bản đồ tự định nghĩa gồm 4 trạm hình chữ nhật",
            "nodes": [
                {"node_id": "A", "name": "Trạm Nhập Hàng A", "type": "INBOUND", "x": 0.0, "y": 0.0, "color": "#10b981", "max_capacity": 100},
                {"node_id": "B", "name": "Kho Linh Kiện B", "type": "STATION", "x": 0.0, "y": 1.5, "color": "#38bdf8", "max_capacity": 30},
                {"node_id": "C", "name": "Kho Thành Phẩm C", "type": "STATION", "x": 2.0, "y": 1.5, "color": "#a855f7", "max_capacity": 30},
                {"node_id": "D", "name": "Kho Đóng Gói D", "type": "STATION", "x": 2.0, "y": 0.0, "color": "#f59e0b", "max_capacity": 30}
            ],
            "segments": [
                {"from": "A", "to": "B", "dist_m": 1.5, "heading": 90.0, "two_way": True, "speed_limit": 25.0},
                {"from": "B", "to": "C", "dist_m": 2.0, "heading": 0.0, "two_way": True, "speed_limit": 25.0},
                {"from": "C", "to": "D", "dist_m": 1.5, "heading": -90.0, "two_way": True, "speed_limit": 25.0},
                {"from": "D", "to": "A", "dist_m": 2.0, "heading": 180.0, "two_way": True, "speed_limit": 25.0}
            ]
        }
        return 200, {"status": "success", "template": template}

    def get_inventory(self, map_id: Optional[str] = None) -> Tuple[int, Dict[str, Any]]:
        """Lấy tồn kho đa mặt hàng của từng trạm kho theo bản đồ hiện hoạt"""
        active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID}
        cur_map_id = map_id or active_map.get("map_id", DEFAULT_MAP_ID)
        nodes = self.workflow_manager.map_repo.get_nodes(cur_map_id)
        inv_by_node = {}
        for n in nodes:
            nid = n["node_id"]
            items = self.workflow_manager.inv_repo.get_node_inventory(cur_map_id, nid)
            cap = self.workflow_manager.inv_repo.get_node_capacity(cur_map_id, nid)
            inv_by_node[nid] = {
                "node_name": n["name"],
                "node_type": n["node_type"],
                "color": n["color"],
                "capacity": cap,
                "items": items
            }
        return 200, {"status": "success", "map_id": cur_map_id, "inventory": inv_by_node}

    def get_tasks(self, limit: int = 20) -> Tuple[int, Dict[str, Any]]:
        """Lấy danh sách các nhiệm vụ điều phối gần nhất"""
        tasks = self.workflow_manager.task_repo.get_recent_tasks(limit=limit)
        return 200, {"status": "success", "total": len(tasks), "tasks": tasks}

    def get_mission_queue(self) -> Tuple[int, Dict[str, Any]]:
        """Lấy thông tin hàng đợi nhiệm vụ đa trạm hiện thời"""
        data = self.workflow_manager.get_mission_queue_status()
        return 200, {"status": "success", "data": data}

    def get_products(self) -> Tuple[int, Dict[str, Any]]:
        """Lấy danh mục sản phẩm, ánh xạ CV Ver3 và trạm đích hiện tại trên bản đồ active"""
        active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID}
        map_id = active_map.get("map_id", DEFAULT_MAP_ID)
        products = self.workflow_manager.prod_repo.get_all()

        enriched = []
        for p in products:
            item = dict(p)
            rules = self.workflow_manager.alloc_repo.get_rules_for_product(
                map_id, item["product_code"], item["category"]
            )
            non_fallback = [r for r in rules if r.get("product_category") != "*"]
            display_rules = non_fallback or rules
            target_nodes = []
            for r in display_rules:
                tn = r["target_node_id"]
                if tn not in target_nodes:
                    target_nodes.append(tn)
            item["target_node_id"] = target_nodes[0] if target_nodes else ""
            item["target_nodes"] = target_nodes
            item["has_custom_rule"] = any(r.get("product_code") == item["product_code"] for r in rules)
            enriched.append(item)

        return 200, {"status": "success", "map_id": map_id, "products": enriched}

    def save_product(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Thêm hoặc cập nhật mặt hàng mới và cập nhật trực tiếp trạm đích ưu tiên"""
        p_code = str(body_json.get("product_code", "")).strip().upper()
        p_name = str(body_json.get("product_name", "")).strip()
        try:
            yolo_id = int(body_json.get("yolo_class_id", 0))
        except (TypeError, ValueError):
            return 400, {"status": "error", "message": "Model Class ID phải là số nguyên hợp lệ"}
        cat = str(body_json.get("category", "GENERAL")).strip().upper()
        desc = str(body_json.get("description", "")).strip()
        target_station = str(body_json.get("target_node_id", "")).strip().upper()

        if not p_code or not p_name:
            return 400, {"status": "error", "message": "Vui lòng nhập đầy đủ Mã hàng và Tên sản phẩm"}
        if yolo_id < 0:
            return 400, {"status": "error", "message": "Model Class ID phải >= 0"}

        existing_owner = self.workflow_manager.prod_repo.get_by_class_id(yolo_id)
        if existing_owner and existing_owner.get("product_code") != p_code:
            return 400, {
                "status": "error",
                "message": (
                    f"Model Class ID {yolo_id} đang được gán cho '{existing_owner['product_code']}' "
                    f"({existing_owner['product_name']}). Hãy sửa trực tiếp mặt hàng đó hoặc chọn Class ID khác!"
                ),
            }

        active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID}
        map_id = active_map.get("map_id", DEFAULT_MAP_ID)
        if target_station:
            valid_nodes = {n["node_id"].upper() for n in self.workflow_manager.map_repo.get_nodes(map_id)}
            if target_station not in valid_nodes:
                return 400, {
                    "status": "error",
                    "message": f"Trạm kho '{target_station}' không tồn tại trên bản đồ {map_id}",
                }

        self.workflow_manager.prod_repo.save_product(p_code, p_name, yolo_id, cat, desc)

        if target_station:
            self.workflow_manager.alloc_repo.replace_product_target_rule(
                map_id=map_id,
                product_code=p_code,
                category=cat,
                target_node_id=target_station,
                priority=1,
            )

        return 200, {"status": "success", "message": f"Đã lưu mặt hàng {p_code} (Kho đích: {target_station or 'Theo nhóm'})!"}

    def delete_product(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Xóa mặt hàng khỏi danh mục và dọn sạch quy tắc gắn riêng nếu không bị ràng buộc"""
        p_code = str(body_json.get("product_code", "")).strip().upper()
        if not p_code:
            return 400, {"status": "error", "message": "Thiếu mã sản phẩm"}
        try:
            self.workflow_manager.prod_repo.delete_product(p_code)
            self.workflow_manager.alloc_repo.delete_rules_for_product(p_code)
        except sqlite3.IntegrityError:
            return 400, {
                "status": "error",
                "message": f"Không thể xóa '{p_code}' vì đã có lịch sử nhiệm vụ hoặc tồn kho tham chiếu. Hãy dùng nút Sửa để đổi tên/nhóm/trạm đích!"
            }
        return 200, {"status": "success", "message": f"Đã xóa mặt hàng {p_code}"}

    def get_rules(self) -> Tuple[int, Dict[str, Any]]:
        """Lấy danh sách các luật phân bổ kho hiện tại"""
        active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID}
        map_id = active_map.get("map_id", DEFAULT_MAP_ID)
        rules = self.workflow_manager.alloc_repo.get_all_rules(map_id)
        return 200, {"status": "success", "map_id": map_id, "rules": rules}

    def save_rule(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Thêm hoặc chỉnh sửa quy tắc phân bổ kho (hỗ trợ đa trạm & dự phòng)"""
        active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID}
        map_id = str(body_json.get("map_id", active_map.get("map_id", DEFAULT_MAP_ID))).strip()
        cat = str(body_json.get("product_category", "*")).strip().upper() or "*"
        target = str(body_json.get("target_node_id", "B")).strip().upper()
        raw_code = body_json.get("product_code")
        p_code = str(raw_code).strip().upper() if raw_code and str(raw_code).strip() else None
        try:
            prio = max(1, int(body_json.get("priority", 1)))
        except (TypeError, ValueError):
            prio = 1

        valid_nodes = {n["node_id"].upper() for n in self.workflow_manager.map_repo.get_nodes(map_id)}
        if target not in valid_nodes:
            return 400, {"status": "error", "message": f"Trạm kho '{target}' không tồn tại trên bản đồ {map_id}"}

        if p_code:
            prod = self.workflow_manager.prod_repo.get_by_code(p_code)
            if prod and prod.get("category"):
                cat = prod["category"]

        self.workflow_manager.alloc_repo.save_rule(map_id, cat, target, p_code, prio, replace_product_rules=False)
        return 200, {"status": "success", "message": f"Đã lưu quy tắc phân bổ -> Kho {target}"}

    def delete_rule(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        rule_id = int(body_json.get("rule_id", 0))
        if rule_id <= 0:
            return 400, {"status": "error", "message": "Mã quy tắc không hợp lệ"}
        self.workflow_manager.alloc_repo.delete_rule(rule_id)
        return 200, {"status": "success", "message": f"Đã xóa quy tắc #{rule_id}"}

    def export_csv(self) -> Tuple[int, str, bytes]:
        """Xuất nhật ký đo kiểm telemetry ra file CSV phục vụ báo cáo đồ án"""
        logs = self.workflow_manager.telemetry_repo.get_recent(limit=5000)
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow([
            "Timestamp", "AGV_ID", "Map_ID", "X_m", "Y_m", "Yaw_deg", "TotalDist_m", 
            "Speed_cm_s", "LineMask", "LineErr", "Voltage_V", "Current_A", 
            "Power_W", "Energy_mAh", "State", "Task_ID"
        ])
        for r in logs:
            writer.writerow([
                r["timestamp"], r["agv_id"], r.get("map_id", ""), r["x"], r["y"], r["yaw"], 
                r["dist"], r["speed"], r["line_mask"], r["line_err"], r["voltage"], 
                r["current"], r["power"], r["energy_mah"], r["state"], r.get("task_id", "")
            ])

        csv_bytes = output.getvalue().encode("utf-8")
        filename = f"agv_wms_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        return 200, filename, csv_bytes
