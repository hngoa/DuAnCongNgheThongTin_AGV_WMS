"""
===================================================================================
CONTROLLER QUẢN LÝ KHO WMS & XUẤT DỮ LIỆU BÁO CÁO (WMS CONTROLLER)
CUNG CẤP API BẢN ĐỒ LINE, TỒN KHO ĐA SẢN PHẨM, DANH SÁCH TASK VÀ XUẤT CSV
===================================================================================
"""

from typing import Dict, Any, Tuple, Optional
import csv
import io
from datetime import datetime
from app.config import DEFAULT_MAP_ID

class WMSController:
    def __init__(self, workflow_manager):
        self.workflow_manager = workflow_manager

    def get_map_data(self, map_id: Optional[str] = None) -> Tuple[int, Dict[str, Any]]:
        """Trả về thông tin bản đồ hiện hoạt, danh sách Điểm nút và các đoạn line đo đạc thực tế"""
        active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID, "name": "Bản đồ Mặc định"}
        cur_map_id = map_id or active_map.get("map_id", DEFAULT_MAP_ID)
        nodes = self.workflow_manager.map_repo.get_nodes(cur_map_id)
        segments = self.workflow_manager.map_repo.get_segments(cur_map_id)
        return 200, {
            "status": "success",
            "map": active_map,
            "nodes": nodes,
            "segments": segments
        }

    def get_all_maps(self) -> Tuple[int, Dict[str, Any]]:
        """Lấy danh sách tất cả các bản đồ có trong hệ thống"""
        maps = self.workflow_manager.map_repo.get_all_maps()
        active_map = self.workflow_manager.map_repo.get_active_map()
        return 200, {
            "status": "success",
            "active_map_id": active_map["map_id"] if active_map else DEFAULT_MAP_ID,
            "maps": maps
        }

    def switch_map(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Chuyển đổi bản đồ hoạt động trực tiếp từ Web UI"""
        target_map_id = str(body_json.get("map_id", "")).strip()
        if not target_map_id:
            return 400, {"status": "error", "message": "Thiếu mã bản đồ (map_id)"}

        success = self.workflow_manager.map_repo.set_active_map(target_map_id)
        if not success:
            return 500, {"status": "error", "message": "Không thể chuyển bản đồ"}

        # Trả về cấu hình bản đồ mới vừa kích hoạt
        return self.get_map_data(target_map_id)

    def import_map(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Nhận file hoặc cấu trúc JSON cấu hình bản đồ tùy biến và nạp vào hệ thống"""
        if not body_json:
            return 400, {"status": "error", "message": "Dữ liệu JSON rỗng"}
        success = self.workflow_manager.map_repo.import_map_config(body_json)
        if not success:
            return 400, {"status": "error", "message": "Cấu trúc JSON không hợp lệ (cần map_id, nodes, segments)"}
        
        map_id = body_json.get("map_id", "").strip().upper()
        return self.get_map_data(map_id)

    def export_map(self, map_id: Optional[str] = None) -> Tuple[int, Dict[str, Any]]:
        """Xuất file JSON cấu hình của bản đồ"""
        active_map = self.workflow_manager.map_repo.get_active_map()
        target_id = map_id or (active_map["map_id"] if active_map else DEFAULT_MAP_ID)
        cfg = self.workflow_manager.map_repo.export_map_config(target_id)
        if not cfg:
            return 404, {"status": "error", "message": "Không tìm thấy bản đồ"}
        return 200, {"status": "success", "config": cfg}

    def get_map_template(self) -> Tuple[int, Dict[str, Any]]:
        """Trả về mẫu JSON chuẩn để người dùng tải về chỉnh sửa theo ý muốn"""
        template = {
            "map_id": "MAP_CUSTOM_01",
            "name": "Bản Đồ Nhà Kho Mới 2.0m x 1.5m",
            "width_m": 2.0,
            "length_m": 1.5,
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
        """Lấy danh mục sản phẩm và ánh xạ YOLO"""
        products = self.workflow_manager.prod_repo.get_all()
        return 200, {"status": "success", "products": products}

    def save_product(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Thêm hoặc cập nhật mặt hàng mới và tự động gán luật phân bổ kho nếu có"""
        p_code = str(body_json.get("product_code", "")).strip().upper()
        p_name = str(body_json.get("product_name", "")).strip()
        yolo_id = int(body_json.get("yolo_class_id", 0))
        cat = str(body_json.get("category", "GENERAL")).strip().upper()
        desc = str(body_json.get("description", "")).strip()
        target_station = str(body_json.get("target_node_id", "")).strip().upper()

        if not p_code or not p_name:
            return 400, {"status": "error", "message": "Vui lòng nhập đầy đủ Mã hàng và Tên sản phẩm"}

        self.workflow_manager.prod_repo.save_product(p_code, p_name, yolo_id, cat, desc)

        # Nếu người dùng chọn sẵn kho đích phân bổ ưu tiên -> Lưu luôn vào allocation_rules
        if target_station:
            active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID}
            map_id = active_map.get("map_id", DEFAULT_MAP_ID)
            self.workflow_manager.alloc_repo.save_rule(
                map_id=map_id,
                product_category=cat,
                target_node_id=target_station,
                product_code=p_code,
                priority=1
            )

        return 200, {"status": "success", "message": f"Đã lưu mặt hàng {p_code} thành công!"}

    def delete_product(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Xóa mặt hàng khỏi danh mục"""
        p_code = str(body_json.get("product_code", "")).strip().upper()
        if not p_code:
            return 400, {"status": "error", "message": "Thiếu mã sản phẩm"}
        self.workflow_manager.prod_repo.delete_product(p_code)
        return 200, {"status": "success", "message": f"Đã xóa mặt hàng {p_code}"}

    def get_rules(self) -> Tuple[int, Dict[str, Any]]:
        """Lấy danh sách các luật phân bổ kho hiện tại"""
        active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID}
        map_id = active_map.get("map_id", DEFAULT_MAP_ID)
        rules = self.workflow_manager.alloc_repo.get_all_rules(map_id)
        return 200, {"status": "success", "map_id": map_id, "rules": rules}

    def save_rule(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Thêm hoặc chỉnh sửa quy tắc phân bổ kho"""
        active_map = self.workflow_manager.map_repo.get_active_map() or {"map_id": DEFAULT_MAP_ID}
        map_id = str(body_json.get("map_id", active_map.get("map_id", DEFAULT_MAP_ID)))
        cat = str(body_json.get("product_category", "*")).strip().upper()
        target = str(body_json.get("target_node_id", "B")).strip().upper()
        p_code = body_json.get("product_code")
        prio = int(body_json.get("priority", 1))

        self.workflow_manager.alloc_repo.save_rule(map_id, cat, target, p_code, prio)
        return 200, {"status": "success", "message": "Đã lưu quy tắc phân bổ thành công"}

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
