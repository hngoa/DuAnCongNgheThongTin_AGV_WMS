"""
===================================================================================
TẦNG TRUY CẬP DỮ LIỆU ĐỘNG (DYNAMIC REPOSITORY PATTERN V2.1)
HỖ TRỢ: ĐA BẢN ĐỒ LINE, TỒN KHO ĐA MẶT HÀNG, PHÂN BỔ KHO THÔNG MINH
===================================================================================
"""

from typing import List, Dict, Any, Optional, Tuple
from collections import deque
from datetime import datetime
from pathlib import Path
from uuid import uuid4
import hashlib
import json
import math
import sys
SERVER_DIR = Path(__file__).resolve().parent.parent
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from database.db_connection import get_db, DB_PATH

MAX_FIRMWARE_MAP_NODES = 16
MAX_FIRMWARE_MAP_EDGES = 64


def normalize_heading_deg(deg: float) -> float:
    """Chuẩn hóa góc la bàn về khoảng (-180.0, 180.0] giống hệt tools/sync_map.py."""
    val = float(deg) % 360.0
    if val > 180.0:
        val -= 360.0
    if val == -180.0:
        val = 180.0
    return round(val, 1)


def compute_map_hash(
    map_id: str,
    nodes: List[Dict[str, Any]],
    segments: List[Dict[str, Any]],
) -> str:
    """Tính mã băm 8 ký tự hex đồng nhất giữa JSON Map-as-Code, SQLite DB và Firmware ESP32."""
    canon_nodes = sorted(
        [
            [
                str(n["node_id"]).strip().upper(),
                round(float(n["x"]), 3),
                round(float(n["y"]), 3),
                str(n.get("type") or n.get("node_type") or "STATION").strip().upper(),
            ]
            for n in nodes
        ],
        key=lambda item: item[0],
    )
    canon_edges = sorted(
        [
            [
                str(e.get("from_node") or e.get("from", "")).strip().upper(),
                str(e.get("to_node") or e.get("to", "")).strip().upper(),
                round(float(e["distance_m"] if "distance_m" in e else e["dist_m"]), 3),
                normalize_heading_deg(float(e["heading_deg"] if "heading_deg" in e else e["heading"])),
            ]
            for e in segments
        ],
        key=lambda item: (item[0], item[1]),
    )
    payload = json.dumps(
        {"map_id": str(map_id).strip().upper(), "nodes": canon_nodes, "edges": canon_edges},
        separators=(",", ":"),
        ensure_ascii=True,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:8]


def validate_and_expand_map_config(
    config: Dict[str, Any],
) -> Tuple[Dict[str, Any], List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Kiểm tra chặt chẽ cấu hình bản đồ JSON và mở rộng các chặng hai chiều."""
    if not isinstance(config, dict):
        raise ValueError("Cấu hình bản đồ phải là đối tượng JSON")

    map_id = str(config.get("map_id", "")).strip().upper()
    if not map_id or len(map_id) > 64:
        raise ValueError("map_id không được rỗng và tối đa 64 ký tự")

    name = str(config.get("name") or f"Bản đồ {map_id}").strip()
    desc = str(config.get("description", "Bản đồ Map-as-Code")).strip()
    try:
        width_m = float(config.get("width_m", 2.0))
        length_m = float(config.get("length_m", 2.0))
    except (TypeError, ValueError) as e:
        raise ValueError(f"[{map_id}] Kích thước bản đồ không hợp lệ: {e}") from e

    if not (math.isfinite(width_m) and width_m > 0.2 and math.isfinite(length_m) and length_m > 0.2):
        raise ValueError(f"[{map_id}] width_m và length_m phải > 0.2m")

    raw_nodes = config.get("nodes")
    raw_segments = config.get("segments")
    if not isinstance(raw_nodes, list) or len(raw_nodes) < 2:
        raise ValueError(f"[{map_id}] Bản đồ phải có ít nhất 2 trạm (nodes)")
    if len(raw_nodes) > MAX_FIRMWARE_MAP_NODES:
        raise ValueError(f"[{map_id}] Số trạm ({len(raw_nodes)}) vượt giới hạn firmware {MAX_FIRMWARE_MAP_NODES}")
    if not isinstance(raw_segments, list) or len(raw_segments) < 1:
        raise ValueError(f"[{map_id}] Bản đồ phải có ít nhất 1 đoạn đường (segments)")

    allowed_types = {"INBOUND", "STATION", "INTERSECTION", "CHARGING", "PARKING"}
    norm_nodes: List[Dict[str, Any]] = []
    node_map: Dict[str, Dict[str, Any]] = {}

    for idx, n in enumerate(raw_nodes):
        if not isinstance(n, dict):
            raise ValueError(f"[{map_id}] Node #{idx} không hợp lệ")
        nid = str(n.get("node_id", "")).strip().upper()
        if not nid or len(nid) > 11 or " " in nid:
            raise ValueError(f"[{map_id}] node_id '{nid}' không hợp lệ (1..11 ký tự, không khoảng trắng)")
        if nid in node_map:
            raise ValueError(f"[{map_id}] Trùng node_id '{nid}'")

        ntype = str(n.get("type") or n.get("node_type") or "STATION").strip().upper()
        if ntype not in allowed_types:
            raise ValueError(f"[{map_id}] Node '{nid}' có type không hợp lệ: '{ntype}'")

        try:
            nx = float(n["x"])
            ny = float(n["y"])
            ncap = int(n.get("max_capacity", 30))
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"[{map_id}] Node '{nid}' thiếu hoặc sai tọa độ/sức chứa: {e}") from e

        if not (math.isfinite(nx) and math.isfinite(ny)) or ncap <= 0:
            raise ValueError(f"[{map_id}] Node '{nid}' có tọa độ/sức chứa không hợp lệ")

        nobj = {
            "node_id": nid,
            "name": str(n.get("name", f"Trạm {nid}")).strip(),
            "type": ntype,
            "x": round(nx, 3),
            "y": round(ny, 3),
            "color": str(n.get("color", "#38bdf8")).strip() or "#38bdf8",
            "max_capacity": ncap,
        }
        norm_nodes.append(nobj)
        node_map[nid] = nobj

    inbound_nodes = [n["node_id"] for n in norm_nodes if n["type"] == "INBOUND"]
    raw_home = config.get("home_node")
    if raw_home is not None and str(raw_home).strip():
        home_node = str(raw_home).strip().upper()
    elif inbound_nodes:
        home_node = inbound_nodes[0]
    elif "A" in node_map:
        home_node = "A"
    else:
        raise ValueError(f"[{map_id}] Bản đồ phải có ít nhất 1 trạm type='INBOUND' hoặc chỉ định 'home_node'")

    if home_node not in node_map:
        raise ValueError(f"[{map_id}] Không tìm thấy home_node '{home_node}' trong danh sách nodes")
    if not inbound_nodes:
        node_map[home_node]["type"] = "INBOUND"

    expanded_edges: List[Dict[str, Any]] = []
    seen_segs = set()
    adj_fwd: Dict[str, List[str]] = {nid: [] for nid in node_map}
    adj_rev: Dict[str, List[str]] = {nid: [] for nid in node_map}

    for idx, s in enumerate(raw_segments):
        if not isinstance(s, dict):
            raise ValueError(f"[{map_id}] Segment #{idx} không hợp lệ")
        from_n = str(s.get("from") or s.get("from_node", "")).strip().upper()
        to_n = str(s.get("to") or s.get("to_node", "")).strip().upper()
        if from_n not in node_map or to_n not in node_map:
            raise ValueError(f"[{map_id}] Segment #{idx} ({from_n}->{to_n}) tham chiếu trạm không tồn tại")
        if from_n == to_n:
            raise ValueError(f"[{map_id}] Segment #{idx} tự nối chính nó ({from_n}->{to_n})")

        try:
            raw_dist = s["dist_m"] if "dist_m" in s else s["distance_m"]
            dist_m = float(raw_dist)
            raw_spd = s["speed_limit"] if "speed_limit" in s else s.get("speed_limit_cm_s", 25.0)
            spd = float(raw_spd)
            has_gap = int(bool(s.get("has_gap", 0)))
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"[{map_id}] Segment ({from_n}->{to_n}) sai dist_m/speed_limit: {e}") from e

        if not (math.isfinite(dist_m) and dist_m >= 0.05):
            raise ValueError(f"[{map_id}] Segment ({from_n}->{to_n}) dist_m={dist_m} phải >= 0.05m")
        if not (math.isfinite(spd) and 5.0 <= spd <= 150.0):
            raise ValueError(f"[{map_id}] Segment ({from_n}->{to_n}) speed_limit={spd} ngoài [5, 150] cm/s")

        dx = node_map[to_n]["x"] - node_map[from_n]["x"]
        dy = node_map[to_n]["y"] - node_map[from_n]["y"]
        euclid = math.hypot(dx, dy)
        if euclid > 0.05 and dist_m < 0.85 * euclid:
            raise ValueError(
                f"[{map_id}] Segment ({from_n}->{to_n}) dist_m={dist_m:.2f}m ngắn bất thường (< 0.85 * {euclid:.2f}m)"
            )

        raw_head = s["heading"] if "heading" in s else s.get("heading_deg")
        if raw_head is None:
            head_deg = normalize_heading_deg(math.degrees(math.atan2(dy, dx)))
        else:
            try:
                head_deg = normalize_heading_deg(float(raw_head))
            except (TypeError, ValueError) as e:
                raise ValueError(f"[{map_id}] Segment ({from_n}->{to_n}) heading không hợp lệ: {e}") from e

        two_way = bool(s.get("two_way", True)) if "direction" not in s else (str(s["direction"]).upper() == "TWO_WAY")
        direction = "TWO_WAY" if two_way else "ONE_WAY"

        sid_fwd = f"SEG_{from_n}{to_n}"
        if sid_fwd not in seen_segs:
            seen_segs.add(sid_fwd)
            expanded_edges.append({
                "segment_id": sid_fwd,
                "from_node": from_n,
                "to_node": to_n,
                "distance_m": round(dist_m, 3),
                "direction": direction,
                "heading_deg": head_deg,
                "speed_limit_cm_s": round(spd, 1),
                "has_gap": has_gap,
            })
            adj_fwd[from_n].append(to_n)
            adj_rev[to_n].append(from_n)

        if two_way:
            sid_rev = f"SEG_{to_n}{from_n}"
            if sid_rev not in seen_segs:
                seen_segs.add(sid_rev)
                rev_head = normalize_heading_deg(head_deg + 180.0)
                expanded_edges.append({
                    "segment_id": sid_rev,
                    "from_node": to_n,
                    "to_node": from_n,
                    "distance_m": round(dist_m, 3),
                    "direction": "TWO_WAY",
                    "heading_deg": rev_head,
                    "speed_limit_cm_s": round(spd, 1),
                    "has_gap": has_gap,
                })
                adj_fwd[to_n].append(from_n)
                adj_rev[from_n].append(to_n)

    if len(expanded_edges) > MAX_FIRMWARE_MAP_EDGES:
        raise ValueError(f"[{map_id}] Tổng số chặng ({len(expanded_edges)}) vượt giới hạn firmware {MAX_FIRMWARE_MAP_EDGES}")

    def _bfs(adj: Dict[str, List[str]], start: str) -> set:
        visited = {start}
        q = deque([start])
        while q:
            cur = q.popleft()
            for nxt in adj.get(cur, []):
                if nxt not in visited:
                    visited.add(nxt)
                    q.append(nxt)
        return visited

    unreach_out = set(node_map.keys()) - _bfs(adj_fwd, home_node)
    if unreach_out:
        raise ValueError(f"[{map_id}] Không thể đi từ trạm gốc '{home_node}' tới: {sorted(unreach_out)}")
    unreach_back = set(node_map.keys()) - _bfs(adj_rev, home_node)
    if unreach_back:
        raise ValueError(f"[{map_id}] Không có đường quay về trạm gốc '{home_node}' từ: {sorted(unreach_back)}")

    meta = {
        "map_id": map_id,
        "name": name,
        "width_m": round(width_m, 3),
        "length_m": round(length_m, 3),
        "description": desc,
        "home_node": home_node,
        "default_rules": list(config.get("default_rules") or []),
    }
    return meta, norm_nodes, expanded_edges


class MapRepository:
    """Quản lý các bản đồ mạng lưới line (Topological Maps), Nodes và Segments"""
    def __init__(self, db_file: Path = DB_PATH):
        self.db_file = db_file
        self.last_import_error: Optional[str] = None

    def get_active_map(self) -> Optional[Dict[str, Any]]:
        """Lấy bản đồ hiện đang được kích hoạt"""
        with get_db(self.db_file) as conn:
            row = conn.execute("SELECT * FROM maps WHERE is_active = 1 LIMIT 1;").fetchone()
            return dict(row) if row else None

    def get_map_by_id(self, map_id: str) -> Optional[Dict[str, Any]]:
        target = str(map_id or "").strip().upper()
        if not target:
            return None
        with get_db(self.db_file) as conn:
            row = conn.execute("SELECT * FROM maps WHERE map_id = ? LIMIT 1;", (target,)).fetchone()
            return dict(row) if row else None

    def get_all_maps(self) -> List[Dict[str, Any]]:
        with get_db(self.db_file) as conn:
            rows = conn.execute("SELECT * FROM maps ORDER BY created_at DESC;").fetchall()
            return [dict(r) for r in rows]

    def get_home_node(self, map_id: Optional[str] = None) -> Dict[str, Any]:
        """Lấy thông tin trạm gốc (node_type='INBOUND'), fallback sang 'A' hoặc node đầu tiên."""
        target_id = str(map_id or "").strip().upper()
        if not target_id:
            active = self.get_active_map()
            target_id = str(active["map_id"]).upper() if active else "SABAN_154_151"
        nodes = self.get_nodes(target_id)
        for n in nodes:
            if str(n.get("node_type", "")).upper() == "INBOUND":
                return n
        for n in nodes:
            if str(n.get("node_id", "")).upper() == "A":
                return n
        if nodes:
            return nodes[0]
        return {"node_id": "A", "name": "Trạm A (Mặc định)", "node_type": "INBOUND", "x": 0.0, "y": 0.0}

    def get_map_hash(self, map_id: Optional[str] = None) -> str:
        """Tính map_hash hiện tại trong DB cho bản đồ chỉ định (hoặc bản đồ đang kích hoạt)."""
        target_id = str(map_id or "").strip().upper()
        if not target_id:
            active = self.get_active_map()
            if not active:
                return ""
            target_id = str(active["map_id"]).upper()
        nodes = self.get_nodes(target_id)
        segments = self.get_segments(target_id)
        if not nodes or not segments:
            return ""
        return compute_map_hash(target_id, nodes, segments)

    def set_active_map(self, map_id: str, sync_fleet: bool = True) -> bool:
        """Kích hoạt an toàn một bản đồ đã tồn tại trong DB (tránh rơi về 0 active maps)."""
        target_id = str(map_id or "").strip().upper()
        if not target_id:
            return False
        with get_db(self.db_file) as conn:
            exists = conn.execute("SELECT 1 FROM maps WHERE map_id = ?;", (target_id,)).fetchone()
            if not exists:
                return False
            conn.execute("UPDATE maps SET is_active = 0 WHERE map_id != ?;", (target_id,))
            conn.execute("UPDATE maps SET is_active = 1 WHERE map_id = ?;", (target_id,))

            if sync_fleet:
                nodes = conn.execute(
                    "SELECT node_id, node_type, x, y FROM map_nodes WHERE map_id = ? ORDER BY node_id;",
                    (target_id,),
                ).fetchall()
                node_ids = {r["node_id"] for r in nodes}
                home_row = next((r for r in nodes if r["node_type"] == "INBOUND"), None)
                if not home_row:
                    home_row = next((r for r in nodes if r["node_id"] == "A"), None) or (nodes[0] if nodes else None)
                if home_row:
                    home_id = home_row["node_id"]
                    hx, hy = float(home_row["x"]), float(home_row["y"])
                    fleet_rows = conn.execute("SELECT agv_id, current_node, status FROM agv_fleet;").fetchall()
                    for agv in fleet_rows:
                        cur_n = str(agv["current_node"] or "").upper()
                        if cur_n not in node_ids or agv["status"] == "OFFLINE":
                            conn.execute(
                                "UPDATE agv_fleet SET map_id = ?, current_node = ?, x = ?, y = ? WHERE agv_id = ?;",
                                (target_id, home_id, hx, hy, agv["agv_id"]),
                            )
                        else:
                            conn.execute(
                                "UPDATE agv_fleet SET map_id = ? WHERE agv_id = ?;",
                                (target_id, agv["agv_id"]),
                            )
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

    def upsert_map_config(self, config: Dict[str, Any], activate: bool = True) -> Dict[str, Any]:
        """
        Nạp hoặc cập nhật bản đồ an toàn (Non-destructive UPSERT):
          - Kiểm tra chặt chẽ schema, tọa độ, khoảng cách, node gốc INBOUND và tính liên thông đồ thị.
          - KHÔNG BAO GIỜ xóa trắng map_nodes (bảo toàn 100% inventory & allocation_rules hiện hữu).
          - Từ chối xóa trạm nếu trạm đó vẫn còn hàng tồn kho (> 0).
        """
        meta, norm_nodes, expanded_edges = validate_and_expand_map_config(config)
        map_id = meta["map_id"]
        home_node = meta["home_node"]
        new_node_ids = {n["node_id"] for n in norm_nodes}
        home_info = next(n for n in norm_nodes if n["node_id"] == home_node)

        with get_db(self.db_file) as conn:
            # 1. UPSERT thông tin bản đồ (tuyệt đối không dùng INSERT OR REPLACE vì gây ON DELETE CASCADE)
            conn.execute("""
                INSERT INTO maps (map_id, name, width_m, length_m, description, is_active)
                VALUES (?, ?, ?, ?, ?, ?)
                ON CONFLICT(map_id) DO UPDATE SET
                    name = excluded.name,
                    width_m = excluded.width_m,
                    length_m = excluded.length_m,
                    description = excluded.description;
            """, (map_id, meta["name"], meta["width_m"], meta["length_m"], meta["description"], 1 if activate else 0))

            if activate:
                conn.execute("UPDATE maps SET is_active = 0 WHERE map_id != ?;", (map_id,))
                conn.execute("UPDATE maps SET is_active = 1 WHERE map_id = ?;", (map_id,))

            # 2. Kiểm tra an toàn các node bị loại bỏ khỏi bản đồ cũ (chặn xóa nếu còn tồn kho > 0)
            existing_nodes = {
                r["node_id"]
                for r in conn.execute("SELECT node_id FROM map_nodes WHERE map_id = ?;", (map_id,)).fetchall()
            }
            removed_nodes = existing_nodes - new_node_ids
            for rid in sorted(removed_nodes):
                qty_row = conn.execute(
                    "SELECT COALESCE(SUM(quantity), 0) FROM inventory WHERE map_id = ? AND node_id = ?;",
                    (map_id, rid),
                ).fetchone()
                if qty_row and int(qty_row[0]) > 0:
                    raise ValueError(
                        f"Không thể xóa trạm '{rid}' của bản đồ '{map_id}' vì vẫn còn {int(qty_row[0])} kiện hàng tồn kho."
                    )

            # Xóa trước các line_segments không còn tồn tại trong cấu hình mới (hoặc trỏ tới node bị xóa)
            existing_segs = {
                r["segment_id"]
                for r in conn.execute("SELECT segment_id FROM line_segments WHERE map_id = ?;", (map_id,)).fetchall()
            }
            new_seg_ids = {e["segment_id"] for e in expanded_edges}
            for sid in existing_segs - new_seg_ids:
                conn.execute(
                    "DELETE FROM line_segments WHERE map_id = ? AND segment_id = ?;",
                    (map_id, sid),
                )

            # Xóa các node thừa đã xác nhận không còn tồn kho (> 0)
            for rid in removed_nodes:
                conn.execute(
                    "DELETE FROM map_nodes WHERE map_id = ? AND node_id = ?;",
                    (map_id, rid),
                )

            # 3. UPSERT từng trạm (bảo toàn khóa ngoại inventory & allocation_rules của các trạm giữ lại)
            is_fresh_map = (len(existing_nodes) == 0)
            for n in norm_nodes:
                nid = n["node_id"]
                is_occ = 1 if (is_fresh_map and nid == home_node) else 0
                occ_agv = "AGV_Ver2" if (is_fresh_map and nid == home_node) else None
                conn.execute("""
                    INSERT INTO map_nodes (map_id, node_id, name, node_type, x, y, color, max_capacity, is_occupied, occupied_by_agv)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ON CONFLICT(map_id, node_id) DO UPDATE SET
                        name = excluded.name,
                        node_type = excluded.node_type,
                        x = excluded.x,
                        y = excluded.y,
                        color = excluded.color,
                        max_capacity = excluded.max_capacity;
                """, (map_id, nid, n["name"], n["type"], n["x"], n["y"], n["color"], n["max_capacity"], is_occ, occ_agv))

            # 4. UPSERT các đoạn line đã chuẩn hóa từ validate_and_expand_map_config
            for e in expanded_edges:
                conn.execute("""
                    INSERT INTO line_segments (
                        map_id, segment_id, from_node, to_node, distance_m, direction,
                        heading_deg, speed_limit_cm_s, has_gap
                    )
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ON CONFLICT(map_id, segment_id) DO UPDATE SET
                        from_node = excluded.from_node,
                        to_node = excluded.to_node,
                        distance_m = excluded.distance_m,
                        direction = excluded.direction,
                        heading_deg = excluded.heading_deg,
                        speed_limit_cm_s = excluded.speed_limit_cm_s,
                        has_gap = excluded.has_gap;
                """, (
                    map_id,
                    e["segment_id"],
                    e["from_node"],
                    e["to_node"],
                    e["distance_m"],
                    e["direction"],
                    e["heading_deg"],
                    e["speed_limit_cm_s"],
                    e["has_gap"],
                ))

            # 5. Khởi tạo luật phân bổ mặc định CHỈ khi bản đồ này chưa có bất kỳ quy tắc nào
            rules_cnt = conn.execute(
                "SELECT COUNT(*) FROM allocation_rules WHERE map_id = ?;",
                (map_id,),
            ).fetchone()[0]
            if rules_cnt == 0:
                inserted_rules = 0
                for rule in meta.get("default_rules") or []:
                    target_n = str(rule.get("target_node_id", "")).strip().upper()
                    if target_n not in new_node_ids:
                        continue
                    cat = rule.get("product_category")
                    pcode = rule.get("product_code")
                    prio = int(rule.get("priority", 1))
                    conn.execute("""
                        INSERT INTO allocation_rules (map_id, product_category, product_code, target_node_id, priority, is_active)
                        VALUES (?, ?, ?, ?, ?, 1);
                    """, (map_id, cat, pcode, target_n, prio))
                    inserted_rules += 1

                if inserted_rules == 0:
                    fallback_node = next((n["node_id"] for n in norm_nodes if n["node_id"] != home_node), home_node)
                    conn.execute("""
                        INSERT INTO allocation_rules (map_id, product_category, product_code, target_node_id, priority, is_active)
                        VALUES (?, '*', NULL, ?, 9, 1);
                    """, (map_id, fallback_node))

            # 6. Đồng bộ vị trí đội xe AGV nếu kích hoạt map này
            if activate:
                fleet_rows = conn.execute("SELECT agv_id, current_node, status FROM agv_fleet;").fetchall()
                for agv in fleet_rows:
                    cur_n = str(agv["current_node"] or "").upper()
                    if cur_n not in new_node_ids or agv["status"] == "OFFLINE":
                        conn.execute(
                            "UPDATE agv_fleet SET map_id = ?, current_node = ?, x = ?, y = ? WHERE agv_id = ?;",
                            (map_id, home_node, home_info["x"], home_info["y"], agv["agv_id"]),
                        )
                    else:
                        conn.execute(
                            "UPDATE agv_fleet SET map_id = ? WHERE agv_id = ?;",
                            (map_id, agv["agv_id"]),
                        )

        return {
            "map_id": map_id,
            "home_node": home_node,
            "map_hash": compute_map_hash(map_id, norm_nodes, expanded_edges),
            "nodes_count": len(norm_nodes),
            "edges_count": len(expanded_edges),
        }

    def import_map_config(self, config: Dict[str, Any], activate: bool = True) -> bool:
        """
        Nạp cấu hình bản đồ tùy biến an toàn (giữ tương thích trả về bool, chi tiết lỗi lưu ở last_import_error).
        """
        self.last_import_error = None
        try:
            self.upsert_map_config(config, activate=activate)
            return True
        except Exception as e:
            self.last_import_error = str(e)
            return False

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
