"""
===================================================================================
TẦNG TRUY CẬP DỮ LIỆU ĐỘNG (DYNAMIC REPOSITORY PATTERN V2.1)
HỖ TRỢ: ĐA BẢN ĐỒ LINE, TỒN KHO ĐA MẶT HÀNG, PHÂN BỔ KHO THÔNG MINH
===================================================================================
"""

from typing import List, Dict, Any, Optional
from datetime import datetime
from pathlib import Path
from uuid import uuid4
import sys
SERVER_DIR = Path(__file__).resolve().parent.parent
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from database.db_connection import get_db, DB_PATH

class MapRepository:
    """Quản lý các bản đồ mạng lưới line (Topological Maps), Nodes và Segments"""
    def __init__(self, db_file: Path = DB_PATH):
        self.db_file = db_file

    def get_active_map(self) -> Optional[Dict[str, Any]]:
        """Lấy bản đồ hiện đang được kích hoạt"""
        with get_db(self.db_file) as conn:
            row = conn.execute("SELECT * FROM maps WHERE is_active = 1 LIMIT 1;").fetchone()
            return dict(row) if row else None

    def get_all_maps(self) -> List[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            rows = conn.execute("SELECT * FROM maps ORDER BY created_at DESC;").fetchall()
            return [dict(r) for r in rows]

    def set_active_map(self, map_id: str) -> bool:
        """Kích hoạt một bản đồ mới mà không cần sửa code"""
        with get_db(self.db_file) as conn:
            conn.execute("UPDATE maps SET is_active = 0;")
            conn.execute("UPDATE maps SET is_active = 1 WHERE map_id = ?;", (map_id,))
            return True

    def get_nodes(self, map_id: str) -> List[Dict[str, Any]]:
        """Lấy toàn bộ các điểm nút (trạm dừng, ngã tư, trạm sạc) của bản đồ"""
        with get_db(self.db_file) as conn:
            rows = conn.execute("SELECT * FROM map_nodes WHERE map_id = ? ORDER BY node_id ASC;", (map_id,)).fetchall()
            return [dict(r) for r in rows]

    def get_node(self, map_id: str, node_id: str) -> Optional[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            row = conn.execute("SELECT * FROM map_nodes WHERE map_id = ? AND node_id = ?;", (map_id, node_id)).fetchone()
            return dict(row) if row else None

    def get_segments(self, map_id: str) -> List[Dict[str, Any]]:
        """Lấy danh sách các đoạn line kết nối đo đạc thực tế"""
        with get_db(self.db_file) as conn:
            rows = conn.execute("SELECT * FROM line_segments WHERE map_id = ?;", (map_id,)).fetchall()
            return [dict(r) for r in rows]

    def lock_segments(self, map_id: str, segment_ids: List[str], agv_id: str) -> bool:
        """Khóa các đoạn line để chống va chạm giữa các AGV"""
        with get_db(self.db_file) as conn:
            for sid in segment_ids:
                conn.execute("""
                    UPDATE line_segments 
                    SET is_locked = 1, locked_by_agv = ?, updated_at = datetime('now', 'localtime')
                    WHERE map_id = ? AND segment_id = ?;
                """, (agv_id, map_id, sid))
            return True

    def unlock_segments(self, map_id: str, segment_ids: List[str]) -> bool:
        """Mở khóa các đoạn line sau khi xe đã đi qua"""
        with get_db(self.db_file) as conn:
            for sid in segment_ids:
                conn.execute("""
                    UPDATE line_segments 
                    SET is_locked = 0, locked_by_agv = NULL, updated_at = datetime('now', 'localtime')
                    WHERE map_id = ? AND segment_id = ?;
                """, (map_id, sid))
            return True

    def unlock_all_by_agv(self, agv_id: str, map_id: Optional[str] = None) -> bool:
        """Giải phóng toàn bộ các đoạn line đang bị khóa bởi một AGV cụ thể"""
        with get_db(self.db_file) as conn:
            if map_id:
                conn.execute("""
                    UPDATE line_segments 
                    SET is_locked = 0, locked_by_agv = NULL, updated_at = datetime('now', 'localtime')
                    WHERE map_id = ? AND locked_by_agv = ?;
                """, (map_id, agv_id))
            else:
                conn.execute("""
                    UPDATE line_segments 
                    SET is_locked = 0, locked_by_agv = NULL, updated_at = datetime('now', 'localtime')
                    WHERE locked_by_agv = ?;
                """, (agv_id,))
            return True

    def import_map_config(self, config: Dict[str, Any]) -> bool:
        """
        Nạp cấu hình bản đồ tùy biến linh hoạt từ JSON:
          - map_id, name, width_m, length_m, description
          - nodes: [ {node_id, name, type, x, y, color, max_capacity}, ... ]
          - segments: [ {from, to, dist_m, heading, two_way, speed_limit}, ... ]
        """
        map_id = str(config.get("map_id", "")).strip().upper()
        name = str(config.get("name", "Bản đồ Tùy biến")).strip()
        width_m = float(config.get("width_m", 2.0))
        length_m = float(config.get("length_m", 2.0))
        desc = str(config.get("description", "Bản đồ nạp linh hoạt qua JSON")).strip()
        nodes = config.get("nodes", [])
        segments = config.get("segments", [])

        if not map_id or not nodes or not segments:
            return False

        with get_db(self.db_file) as conn:
            # 1. Lưu thông tin bản đồ
            conn.execute("""
                INSERT INTO maps (map_id, name, width_m, length_m, description, is_active)
                VALUES (?, ?, ?, ?, ?, 1)
                ON CONFLICT(map_id) DO UPDATE SET
                    name = excluded.name,
                    width_m = excluded.width_m,
                    length_m = excluded.length_m,
                    description = excluded.description;
            """, (map_id, name, width_m, length_m, desc))

            # Đặt map này làm active
            conn.execute("UPDATE maps SET is_active = 0 WHERE map_id != ?;", (map_id,))
            conn.execute("UPDATE maps SET is_active = 1 WHERE map_id = ?;", (map_id,))

            # 2. Xóa các node và segment cũ của map này nếu có
            conn.execute("DELETE FROM line_segments WHERE map_id = ?;", (map_id,))
            conn.execute("DELETE FROM map_nodes WHERE map_id = ?;", (map_id,))

            # 3. Nạp nodes mới
            for n in nodes:
                nid = str(n.get("node_id", "")).strip().upper()
                nname = str(n.get("name", f"Trạm {nid}")).strip()
                ntype = str(n.get("type", "STATION")).strip().upper()
                nx = float(n.get("x", 0.0))
                ny = float(n.get("y", 0.0))
                ncolor = str(n.get("color", "#38bdf8")).strip()
                ncap = int(n.get("max_capacity", 30))
                is_occ = 1 if nid == "A" else 0
                occ_agv = "AGV_Ver2" if nid == "A" else None

                conn.execute("""
                    INSERT INTO map_nodes (map_id, node_id, name, node_type, x, y, color, max_capacity, is_occupied, occupied_by_agv)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
                """, (map_id, nid, nname, ntype, nx, ny, ncolor, ncap, is_occ, occ_agv))

            # 4. Nạp segments mới
            seen_segs = set()
            for s in segments:
                from_n = str(s.get("from") or s.get("from_node", "")).strip().upper()
                to_n = str(s.get("to") or s.get("to_node", "")).strip().upper()
                dist_m = float(s.get("dist_m") or s.get("distance_m", 1.0))
                heading = float(s.get("heading") or s.get("heading_deg", 0.0))
                two_way = bool(s.get("two_way", True))
                spd = float(s.get("speed_limit") or s.get("speed_limit_cm_s", 25.0))

                sid_fwd = f"SEG_{from_n}{to_n}"
                if sid_fwd not in seen_segs:
                    seen_segs.add(sid_fwd)
                    conn.execute("""
                        INSERT INTO line_segments (map_id, segment_id, from_node, to_node, distance_m, direction, heading_deg, speed_limit_cm_s)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?);
                    """, (map_id, sid_fwd, from_n, to_n, dist_m, "TWO_WAY" if two_way else "ONE_WAY", heading, spd))

                if two_way:
                    sid_rev = f"SEG_{to_n}{from_n}"
                    if sid_rev not in seen_segs:
                        seen_segs.add(sid_rev)
                        rev_heading = (heading + 180.0) % 360.0
                        if rev_heading > 180.0: rev_heading -= 360.0
                        conn.execute("""
                            INSERT INTO line_segments (map_id, segment_id, from_node, to_node, distance_m, direction, heading_deg, speed_limit_cm_s)
                            VALUES (?, ?, ?, ?, ?, ?, ?, ?);
                        """, (map_id, sid_rev, to_n, from_n, dist_m, "TWO_WAY", rev_heading, spd))

            # 5. Khởi tạo luật phân bổ dự phòng cho map mới nếu chưa có
            conn.execute("""
                INSERT OR IGNORE INTO allocation_rules (map_id, product_category, product_code, target_node_id, priority, is_active)
                SELECT ?, '*', NULL, node_id, 9, 1 FROM map_nodes WHERE map_id = ? AND node_id != 'A' LIMIT 1;
            """, (map_id, map_id))

            return True

    def export_map_config(self, map_id: str) -> Optional[Dict[str, Any]]:
        """Xuất toàn bộ cấu trúc bản đồ ra định dạng JSON chuẩn"""
        with get_db(self.db_file) as conn:
            m_row = conn.execute("SELECT * FROM maps WHERE map_id = ?;", (map_id,)).fetchone()
            if not m_row: return None
            nodes = conn.execute("SELECT * FROM map_nodes WHERE map_id = ? ORDER BY node_id ASC;", (map_id,)).fetchall()
            segs = conn.execute("SELECT * FROM line_segments WHERE map_id = ?;", (map_id,)).fetchall()

            return {
                "map_id": m_row["map_id"],
                "name": m_row["name"],
                "width_m": m_row["width_m"],
                "length_m": m_row["length_m"],
                "description": m_row["description"],
                "nodes": [
                    {
                        "node_id": n["node_id"], "name": n["name"], "type": n["node_type"],
                        "x": n["x"], "y": n["y"], "color": n["color"], "max_capacity": n["max_capacity"]
                    } for n in nodes
                ],
                "segments": [
                    {
                        "from": s["from_node"], "to": s["to_node"], "dist_m": s["distance_m"],
                        "heading": s["heading_deg"], "speed_limit": s["speed_limit_cm_s"],
                        "two_way": (s["direction"] == "TWO_WAY")
                    } for s in segs
                ]
            }


class InventoryRepository:
    """Quản lý tồn kho chi tiết từng mặt hàng tại mỗi trạm (Multi-Product Stock)"""
    def __init__(self, db_file: Path = DB_PATH):
        self.db_file = db_file

    def get_node_inventory(self, map_id: str, node_id: str) -> List[Dict[str, Any]]:
        """Lấy chi tiết danh sách các mặt hàng đang có tại một trạm"""
        with get_db(self.db_file) as conn:
            rows = conn.execute("""
                SELECT i.product_code, p.product_name, p.category, i.quantity, i.updated_at
                FROM inventory i
                JOIN products p ON i.product_code = p.product_code
                WHERE i.map_id = ? AND i.node_id = ?;
            """, (map_id, node_id)).fetchall()
            return [dict(r) for r in rows]

    def get_node_capacity(self, map_id: str, node_id: str) -> Dict[str, Any]:
        """Tính tổng số kiện hàng thực tế và kiểm tra sức chứa còn lại của trạm"""
        with get_db(self.db_file) as conn:
            # Lấy max capacity của trạm
            node_row = conn.execute("SELECT max_capacity FROM map_nodes WHERE map_id = ? AND node_id = ?;", 
                                    (map_id, node_id)).fetchone()
            if not node_row:
                return {"has_space": False, "error": "Không tìm thấy trạm"}

            max_cap = node_row["max_capacity"] or 30
            # Tổng các kiện hàng thuộc mọi loại sản phẩm ở trạm này
            stock_row = conn.execute("""
                SELECT COALESCE(SUM(quantity), 0) AS total_stock 
                FROM inventory 
                WHERE map_id = ? AND node_id = ?;
            """, (map_id, node_id)).fetchone()
            total_stock = stock_row["total_stock"] if stock_row else 0
            
            return {
                "has_space": total_stock < max_cap,
                "current_stock": total_stock,
                "max_capacity": max_cap,
                "available_slots": max_cap - total_stock
            }

    def update_stock(self, map_id: str, node_id: str, product_code: str, delta: int = 1) -> bool:
        """Tăng hoặc giảm tồn kho của một sản phẩm cụ thể tại trạm"""
        with get_db(self.db_file) as conn:
            conn.execute("""
                INSERT INTO inventory (map_id, node_id, product_code, quantity)
                VALUES (?, ?, ?, MAX(0, ?))
                ON CONFLICT(map_id, node_id, product_code) DO UPDATE SET
                    quantity = MAX(0, inventory.quantity + ?),
                    updated_at = datetime('now', 'localtime');
            """, (map_id, node_id, product_code, delta, delta))
            return True


class AllocationEngineRepository:
    """Bộ quy tắc phân bổ kho thông minh (Dynamic Storage Allocation Engine)"""
    def __init__(self, db_file: Path = DB_PATH):
        self.db_file = db_file
        self.inv_repo = InventoryRepository(db_file)

    def get_all_rules(self, map_id: Optional[str] = None) -> List[Dict[str, Any]]:
        """Lấy danh sách các luật phân bổ kho hiện có"""
        with get_db(self.db_file) as conn:
            if map_id:
                rows = conn.execute("""
                    SELECT r.*, m.name as map_name 
                    FROM allocation_rules r
                    LEFT JOIN maps m ON r.map_id = m.map_id
                    WHERE r.map_id = ?
                    ORDER BY r.priority ASC;
                """, (map_id,)).fetchall()
            else:
                rows = conn.execute("""
                    SELECT r.*, m.name as map_name 
                    FROM allocation_rules r
                    LEFT JOIN maps m ON r.map_id = m.map_id
                    ORDER BY r.map_id ASC, r.priority ASC;
                """, ()).fetchall()
            return [dict(r) for r in rows]

    def save_rule(self, map_id: str, product_category: str, target_node_id: str, 
                  product_code: Optional[str] = None, priority: int = 1,
                  replace_product_rules: bool = False) -> bool:
        """Thêm hoặc cập nhật luật phân bổ kho đích"""
        m_id = map_id.strip()
        cat = (product_category or "*").strip().upper()
        target = target_node_id.strip().upper()
        p_code = product_code.strip().upper() if product_code and str(product_code).strip() else None
        prio = max(1, int(priority))

        with get_db(self.db_file) as conn:
            if p_code and replace_product_rules:
                conn.execute("""
                    DELETE FROM allocation_rules 
                    WHERE map_id = ? AND product_code = ?;
                """, (m_id, p_code))
            elif p_code:
                conn.execute("""
                    DELETE FROM allocation_rules 
                    WHERE map_id = ? AND product_code = ? AND target_node_id = ?;
                """, (m_id, p_code, target))
            else:
                conn.execute("""
                    DELETE FROM allocation_rules 
                    WHERE map_id = ? AND product_category = ? AND product_code IS NULL AND target_node_id = ?;
                """, (m_id, cat, target))

            conn.execute("""
                INSERT INTO allocation_rules (map_id, product_category, product_code, target_node_id, priority, is_active)
                VALUES (?, ?, ?, ?, ?, 1);
            """, (m_id, cat, p_code, target, prio))
            return True

    def replace_product_target_rule(self, map_id: str, product_code: str, category: str,
                                    target_node_id: str, priority: int = 1) -> bool:
        """Thay thế trực tiếp trạm đích cho một mã sản phẩm trên bản đồ.

        Xóa mọi luật cũ gắn riêng cho product_code trên map_id (và nếu không có sản phẩm nào khác
        dùng chung category, dọn luôn luật mặc định theo category cũ để tránh xung đột kho cũ),
        rồi lưu quy tắc trạm đích mới.
        """
        m_id = map_id.strip()
        p_code = product_code.strip().upper()
        cat = (category or "GENERAL").strip().upper()
        target = target_node_id.strip().upper()
        prio = max(1, int(priority))

        with get_db(self.db_file) as conn:
            conn.execute(
                "DELETE FROM allocation_rules WHERE map_id = ? AND product_code = ?;",
                (m_id, p_code),
            )
            other_same_cat = conn.execute(
                "SELECT COUNT(*) FROM products WHERE category = ? AND product_code != ?;",
                (cat, p_code),
            ).fetchone()[0]
            if other_same_cat == 0 and cat != "*":
                conn.execute(
                    "DELETE FROM allocation_rules WHERE map_id = ? AND product_category = ? AND (product_code IS NULL OR product_code = '');",
                    (m_id, cat),
                )
            conn.execute("""
                INSERT INTO allocation_rules (map_id, product_category, product_code, target_node_id, priority, is_active)
                VALUES (?, ?, ?, ?, ?, 1);
            """, (m_id, cat, p_code, target, prio))
            return True

    def delete_rules_for_product(self, product_code: str, map_id: Optional[str] = None) -> bool:
        """Xóa toàn bộ quy tắc phân bổ gắn với một mã sản phẩm."""
        p_code = product_code.strip().upper()
        with get_db(self.db_file) as conn:
            if map_id:
                conn.execute(
                    "DELETE FROM allocation_rules WHERE map_id = ? AND product_code = ?;",
                    (map_id.strip(), p_code),
                )
            else:
                conn.execute(
                    "DELETE FROM allocation_rules WHERE product_code = ?;",
                    (p_code,),
                )
            return True

    def get_rules_for_product(self, map_id: str, product_code: str, category: str) -> List[Dict[str, Any]]:
        """Lấy toàn bộ luật khớp trực tiếp hoặc theo nhóm cho một sản phẩm theo thứ tự ưu tiên."""
        p_code = (product_code or "").strip().upper()
        cat = (category or "").strip().upper()
        with get_db(self.db_file) as conn:
            rows = conn.execute("""
                SELECT * FROM allocation_rules
                WHERE map_id = ? AND is_active = 1
                  AND (
                    product_code = ?
                    OR ((product_code IS NULL OR product_code = '') AND (product_category = ? OR product_category = '*'))
                  )
                ORDER BY priority ASC, rule_id ASC;
            """, (map_id, p_code, cat)).fetchall()

        ranked: List[Dict[str, Any]] = []
        for r in rows:
            item = dict(r)
            if item.get("product_code") == p_code:
                tier = 0
            elif item.get("product_category") == "*":
                tier = 2
            else:
                tier = 1
            item["_tier"] = tier
            ranked.append(item)

        ranked.sort(key=lambda x: (x["_tier"], int(x.get("priority", 1)), int(x.get("rule_id", 0))))
        return ranked

    def delete_rule(self, rule_id: int) -> bool:
        """Xóa một quy tắc phân bổ"""
        with get_db(self.db_file) as conn:
            conn.execute("DELETE FROM allocation_rules WHERE rule_id = ?;", (rule_id,))
            return True

    def allocate_target_node(self, map_id: str, product_code: str, category: str) -> Optional[Dict[str, Any]]:
        """
        Xác định trạm kho đích tối ưu cho sản phẩm dựa trên:
          1. Mức độ đặc hiệu: Luật theo product_code > Luật theo product_category > Dự phòng '*'
          2. Quy tắc cấu hình ưu tiên (Priority ASC)
          3. Kiểm tra sức chứa còn lại của kho (Capacity Check)
        """
        rules = self.get_rules_for_product(map_id, product_code, category)
        for r in rules:
            target_node = r["target_node_id"]
            cap = self.inv_repo.get_node_capacity(map_id, target_node)
            if cap.get("has_space", False):
                return {
                    "target_node_id": target_node,
                    "strategy": "RULE_MATCHED",
                    "rule_id": r["rule_id"],
                    "available_slots": cap["available_slots"]
                }

        return None # Tất cả các kho đều đã đầy!

    def allocate_nearest_target_node(self, map_id: str, product_code: str, category: str, 
                                     from_node: str = "A", router: Optional[Any] = None,
                                     current_agv_id: str = "AGV_Ver2") -> Optional[Dict[str, Any]]:
        """
        Xác định trạm kho đích GẦN NHẤT cho sản phẩm dựa trên:
          1. Mức độ đặc hiệu (Luật riêng product_code > Luật nhóm category > Dự phòng '*')
          2. Độ ưu tiên (priority ASC) trong từng tầng
          3. Nếu nhiều kho hợp lệ cùng độ ưu tiên: chọn kho có khoảng cách Dijkstra gần nhất từ vị trí xe hiện tại
        """
        rules = self.get_rules_for_product(map_id, product_code, category)
        if not rules:
            return None

        candidates = []
        seen_nodes = set()
        for r in rules:
            target_node = r["target_node_id"]
            if target_node == from_node:
                continue # Không giao hàng về chính trạm xuất phát
            if target_node in seen_nodes:
                continue
            seen_nodes.add(target_node)

            # Kiểm tra trạm này có còn sức chứa không
            cap = self.inv_repo.get_node_capacity(map_id, target_node)
            if not cap.get("has_space", False):
                continue

            dist_m = 1.0
            path_info = None
            if router is not None:
                path_info = router.find_shortest_path(from_node, target_node, current_agv_id=current_agv_id)
                if not path_info:
                    continue # Đường đi bị nghẽn do khóa đoạn line hoặc không thông đường
                dist_m = float(path_info.get("total_distance_m", 1.0))

            candidates.append({
                "target_node_id": target_node,
                "distance_m": dist_m,
                "priority": int(r.get("priority", 1)),
                "tier": int(r.get("_tier", 1)),
                "rule_id": r["rule_id"],
                "available_slots": cap["available_slots"],
                "path_info": path_info
            })

        if not candidates:
            return None

        # Sắp xếp ưu tiên:
        # 1. Luật đích danh product_code (tier 0) > luật theo category (tier 1) > dự phòng '*' (tier 2)
        # 2. Độ ưu tiên cấu hình (priority ASC)
        # 3. Kho gần nhất theo Dijkstra (distance_m ASC)
        candidates.sort(key=lambda c: (c["tier"], c["priority"], c["distance_m"]))

        best = candidates[0]
        return {
            "target_node_id": best["target_node_id"],
            "strategy": "NEAREST_DISTANCE",
            "distance_m": best["distance_m"],
            "rule_id": best["rule_id"],
            "available_slots": best["available_slots"],
            "path_info": best.get("path_info")
        }


class ProductRepository:
    """Quản lý danh mục sản phẩm và ánh xạ nhãn AI MobileNetV3"""
    def __init__(self, db_file: Path = DB_PATH):
        self.db_file = db_file

    def get_all(self) -> List[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            rows = conn.execute("SELECT * FROM products ORDER BY yolo_class_id ASC;").fetchall()
            return [dict(r) for r in rows]

    def get_by_code(self, product_code: str) -> Optional[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            row = conn.execute("SELECT * FROM products WHERE product_code = ?;", (product_code,)).fetchone()
            return dict(row) if row else None

    def get_by_class_id(self, yolo_class_id: int) -> Optional[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            row = conn.execute("SELECT * FROM products WHERE yolo_class_id = ?;", (int(yolo_class_id),)).fetchone()
            return dict(row) if row else None

    def save_product(self, product_code: str, product_name: str, yolo_class_id: int, 
                     category: str, description: str = "") -> bool:
        """Thêm mới hoặc cập nhật một mặt hàng trong danh mục"""
        with get_db(self.db_file) as conn:
            conn.execute("""
                INSERT INTO products (product_code, product_name, yolo_class_id, category, description)
                VALUES (?, ?, ?, ?, ?)
                ON CONFLICT(product_code) DO UPDATE SET
                    product_name = excluded.product_name,
                    yolo_class_id = excluded.yolo_class_id,
                    category = excluded.category,
                    description = excluded.description;
            """, (product_code.strip(), product_name.strip(), int(yolo_class_id), category.strip(), description.strip()))
            return True

    def delete_product(self, product_code: str) -> bool:
        """Xóa một mặt hàng khỏi danh mục"""
        with get_db(self.db_file) as conn:
            conn.execute("DELETE FROM products WHERE product_code = ?;", (product_code.strip(),))
            return True


class TaskRepository:
    """Quản lý vòng đời nhiệm vụ điều phối (Dispatch Tasks)"""
    def __init__(self, db_file: Path = DB_PATH):
        self.db_file = db_file

    def create_task(self, map_id: str, product_code: str, product_name: str, 
                    from_node: str, to_node: str, route_path: str, 
                    ai_confidence: float = 0.0, image_path: str = "", 
                    assigned_agv: str = "AGV_Ver2") -> int:
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        task_code = f"TSK_{now_str}_{uuid4().hex[:4]}_{from_node}to{to_node}"
        
        with get_db(self.db_file) as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO dispatch_tasks (
                    task_code, map_id, product_code, product_name, from_node, to_node, 
                    route_path, assigned_agv, ai_confidence, image_path, status
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 'PENDING');
            """, (task_code, map_id, product_code, product_name, from_node, to_node, 
                  route_path, assigned_agv, ai_confidence, image_path))
            return cursor.lastrowid

    def get_by_id(self, task_id: int) -> Optional[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            row = conn.execute("SELECT * FROM dispatch_tasks WHERE task_id = ?;", (task_id,)).fetchone()
            return dict(row) if row else None

    def update_status(self, task_id: int, new_status: str, error_msg: Optional[str] = None) -> bool:
        with get_db(self.db_file) as conn:
            if new_status == 'IN_TRANSIT':
                conn.execute("""
                    UPDATE dispatch_tasks 
                    SET status = ?, started_at = datetime('now', 'localtime'), error_message = ?
                    WHERE task_id = ?;
                """, (new_status, error_msg, task_id))
            elif new_status in ('COMPLETED', 'DELIVERED', 'REJECTED_UNKNOWN', 'REJECTED_STOCK_FULL', 'ABORTED'):
                conn.execute("""
                    UPDATE dispatch_tasks 
                    SET status = ?, completed_at = datetime('now', 'localtime'), error_message = ?
                    WHERE task_id = ?;
                """, (new_status, error_msg, task_id))
            else:
                conn.execute("UPDATE dispatch_tasks SET status = ?, error_message = ? WHERE task_id = ?;", 
                             (new_status, error_msg, task_id))
            return True

    def update_dispatch_target(self, task_id: int, to_node: str, route_path: str, assigned_agv: Optional[str] = None) -> bool:
        """Cập nhật trạm đích, lộ trình và chuyển trạng thái task sang DISPATCHED."""
        with get_db(self.db_file) as conn:
            if assigned_agv:
                conn.execute("""
                    UPDATE dispatch_tasks
                    SET to_node = ?, route_path = ?, assigned_agv = ?, status = 'DISPATCHED'
                    WHERE task_id = ?;
                """, (to_node, route_path, assigned_agv, task_id))
            else:
                conn.execute("""
                    UPDATE dispatch_tasks
                    SET to_node = ?, route_path = ?, status = 'DISPATCHED'
                    WHERE task_id = ?;
                """, (to_node, route_path, task_id))
            return True

    def get_recent_tasks(self, limit: int = 20) -> List[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            rows = conn.execute("SELECT * FROM dispatch_tasks ORDER BY task_id DESC LIMIT ?;", (limit,)).fetchall()
            return [dict(r) for r in rows]


class FleetRepository:
    """Quản lý trạng thái xe AGV và năng lượng INA226"""
    def __init__(self, db_file: Path = DB_PATH):
        self.db_file = db_file

    def get_agv(self, agv_id: str = "AGV_Ver2") -> Optional[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            row = conn.execute("SELECT * FROM agv_fleet WHERE agv_id = ?;", (agv_id,)).fetchone()
            return dict(row) if row else None

    def update_telemetry(self, agv_id: str, x: float, y: float, yaw: float, 
                         speed: float, voltage: float, current: float, 
                         battery_pct: float, status: str, ack_id: int = 0) -> bool:
        with get_db(self.db_file) as conn:
            conn.execute("""
                UPDATE agv_fleet 
                SET x = ?, y = ?, yaw_deg = ?, speed_cm_s = ?, 
                    voltage_v = ?, current_a = ?, battery_pct = ?, 
                    status = ?, last_ack_id = ?, last_heartbeat = datetime('now', 'localtime')
                WHERE agv_id = ?;
            """, (x, y, yaw, speed, voltage, current, battery_pct, status, ack_id, agv_id))
            return True


class TelemetryRepository:
    """Ghi vết lịch sử chuyển động và điện năng (Data Logger)"""
    def __init__(self, db_file: Path = DB_PATH):
        self.db_file = db_file
        self._ensure_columns()

    def _ensure_columns(self):
        """Tự động bổ sung các cột station, leg vào bảng telemetry_logs nếu chưa tồn tại"""
        try:
            with get_db(self.db_file) as conn:
                cols = [r["name"] for r in conn.execute("PRAGMA table_info(telemetry_logs);").fetchall()]
                if "station" not in cols:
                    conn.execute("ALTER TABLE telemetry_logs ADD COLUMN station TEXT;")
                if "leg" not in cols:
                    conn.execute("ALTER TABLE telemetry_logs ADD COLUMN leg TEXT;")
        except Exception:
            pass

    def log(self, agv_id: str, map_id: str, x: float, y: float, yaw: float, 
            dist: float, speed: float, mask: int, err: float, v: float, 
            a: float, w: float, mah: float, state: str, task_id: Optional[int] = None,
            station: Optional[str] = None, leg: Optional[str] = None) -> bool:
        with get_db(self.db_file) as conn:
            conn.execute("""
                INSERT INTO telemetry_logs (
                    agv_id, map_id, station, leg, x, y, yaw, dist, speed, line_mask, line_err, 
                    voltage, current, power, energy_mah, state, task_id
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
            """, (agv_id, map_id, station, leg, x, y, yaw, dist, speed, mask, err, v, a, w, mah, state, task_id))
            return True

    def get_recent(self, limit: int = 200) -> List[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            rows = conn.execute("SELECT * FROM telemetry_logs ORDER BY log_id DESC LIMIT ?;", (limit,)).fetchall()
            return [dict(r) for r in rows]

    def get_logs(self, limit: int = 100, offset: int = 0, state: Optional[str] = None) -> List[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            if state and state.strip() and state != "ALL":
                rows = conn.execute("""
                    SELECT * FROM telemetry_logs 
                    WHERE state = ? 
                    ORDER BY log_id DESC LIMIT ? OFFSET ?;
                """, (state.strip(), limit, offset)).fetchall()
            else:
                rows = conn.execute("""
                    SELECT * FROM telemetry_logs 
                    ORDER BY log_id DESC LIMIT ? OFFSET ?;
                """, (limit, offset)).fetchall()
            return [dict(r) for r in rows]

    def count(self, state: Optional[str] = None) -> int:
        with get_db(self.db_file) as conn:
            if state and state.strip() and state != "ALL":
                row = conn.execute("SELECT COUNT(*) FROM telemetry_logs WHERE state = ?;", (state.strip(),)).fetchone()
            else:
                row = conn.execute("SELECT COUNT(*) FROM telemetry_logs;").fetchone()
            return row[0] if row else 0

    def clear(self) -> bool:
        with get_db(self.db_file) as conn:
            conn.execute("DELETE FROM telemetry_logs;")
            return True
