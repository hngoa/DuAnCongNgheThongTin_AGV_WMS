"""Single Source of Truth validator & synchronizer for AGV_Ver2 Topological Maps.

Responsibilities (Map-as-Code workflow):
1. Validate map JSON configs (schema, geometry consistency, inbound home node, graph reachability).
2. Compute deterministic MAP_HASH shared between Server (SQLite) and Firmware (ESP32-S3).
3. Generate `include/generated_map_config.h` cleanly without altering core firmware motion code.
4. Optionally sync the validated active map + catalog maps into Server SQLite DB non-destructively.

Usage:
    python tools/sync_map.py                       # Regenerate header from config/active_map.json
    python tools/sync_map.py --check               # Validate all maps & verify header is up to date
    python tools/sync_map.py --map config/maps/map_exp_6nodes.json [--apply-db]
"""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Dict, List, Optional, Tuple


ROOT_DIR = Path(__file__).resolve().parent.parent
CONFIG_DIR = ROOT_DIR / "config"
MAPS_DIR = CONFIG_DIR / "maps"
ACTIVE_MAP_PATH = CONFIG_DIR / "active_map.json"
HEADER_PATH = ROOT_DIR / "include" / "generated_map_config.h"

MAX_MAP_NODES = 16
MAX_MAP_EDGES = 64


def normalize_heading_deg(deg: float) -> float:
    """Normalize angle in degrees to (-180.0, 180.0]."""
    val = float(deg) % 360.0
    if val > 180.0:
        val -= 360.0
    if val == -180.0:
        val = 180.0
    return round(val, 1)


def _c_escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace('"', '\\"')


def validate_and_expand_map(cfg: Dict[str, Any]) -> Tuple[Dict[str, Any], List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Validate map configuration strictly and return (meta, normalized_nodes, expanded_edges)."""
    if not isinstance(cfg, dict):
        raise ValueError("Cấu hình bản đồ phải là đối tượng JSON")

    map_id = str(cfg.get("map_id", "")).strip().upper()
    name = str(cfg.get("name", "")).strip()
    if not map_id or len(map_id) > 64:
        raise ValueError("map_id không được rỗng và tối đa 64 ký tự")
    if not name:
        raise ValueError(f"[{map_id}] Thiếu tên bản đồ (name)")

    try:
        width_m = float(cfg.get("width_m", 0.0))
        length_m = float(cfg.get("length_m", 0.0))
        corridor_guard_m = float(cfg.get("corridor_guard_m", 0.85))
    except (TypeError, ValueError) as e:
        raise ValueError(f"[{map_id}] Kích thước bản đồ không hợp lệ: {e}") from e

    if not (math.isfinite(width_m) and width_m > 0.2 and math.isfinite(length_m) and length_m > 0.2):
        raise ValueError(f"[{map_id}] width_m và length_m phải > 0.2m (nhận {width_m}, {length_m})")
    if not (math.isfinite(corridor_guard_m) and 0.35 <= corridor_guard_m <= 5.0):
        raise ValueError(f"[{map_id}] corridor_guard_m phải trong [0.35, 5.0]m")

    raw_nodes = cfg.get("nodes")
    raw_segments = cfg.get("segments")
    if not isinstance(raw_nodes, list) or len(raw_nodes) < 2:
        raise ValueError(f"[{map_id}] Bản đồ phải có ít nhất 2 trạm (nodes)")
    if len(raw_nodes) > MAX_MAP_NODES:
        raise ValueError(f"[{map_id}] Số trạm ({len(raw_nodes)}) vượt giới hạn firmware MAX_MAP_NODES={MAX_MAP_NODES}")
    if not isinstance(raw_segments, list) or len(raw_segments) < 1:
        raise ValueError(f"[{map_id}] Bản đồ phải có ít nhất 1 đoạn đường (segments)")

    norm_nodes: List[Dict[str, Any]] = []
    node_map: Dict[str, Dict[str, Any]] = {}
    allowed_types = {"INBOUND", "STATION", "INTERSECTION", "CHARGING", "PARKING"}

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
    home_node = str(cfg.get("home_node") or (inbound_nodes[0] if inbound_nodes else "A")).strip().upper()
    if home_node not in node_map:
        raise ValueError(f"[{map_id}] Không tìm thấy home_node '{home_node}' trong danh sách nodes")
    if not inbound_nodes:
        node_map[home_node]["type"] = "INBOUND"

    expanded_edges: List[Dict[str, Any]] = []
    seen_seg_ids = set()
    adj_fwd: Dict[str, List[str]] = {nid: [] for nid in node_map}
    adj_rev: Dict[str, List[str]] = {nid: [] for nid in node_map}

    for idx, s in enumerate(raw_segments):
        if not isinstance(s, dict):
            raise ValueError(f"[{map_id}] Segment #{idx} không hợp lệ")
        u = str(s.get("from") or s.get("from_node", "")).strip().upper()
        v = str(s.get("to") or s.get("to_node", "")).strip().upper()
        if u not in node_map or v not in node_map:
            raise ValueError(f"[{map_id}] Segment #{idx} ({u}->{v}) tham chiếu tới trạm không tồn tại")
        if u == v:
            raise ValueError(f"[{map_id}] Segment #{idx} tự nối tới chính nó ({u}->{v})")

        try:
            dist_m = float(s.get("dist_m") if "dist_m" in s else s.get("distance_m"))
            spd = float(s.get("speed_limit") if "speed_limit" in s else s.get("speed_limit_cm_s", 25.0))
            has_gap = int(bool(s.get("has_gap", 0)))
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"[{map_id}] Segment ({u}->{v}) sai dist_m/speed_limit: {e}") from e

        if not (math.isfinite(dist_m) and dist_m >= 0.05):
            raise ValueError(f"[{map_id}] Segment ({u}->{v}) có dist_m={dist_m} phải >= 0.05m")
        if not (math.isfinite(spd) and 5.0 <= spd <= 150.0):
            raise ValueError(f"[{map_id}] Segment ({u}->{v}) có speed_limit={spd} ngoài [5, 150] cm/s")

        dx = node_map[v]["x"] - node_map[u]["x"]
        dy = node_map[v]["y"] - node_map[u]["y"]
        euclid = math.hypot(dx, dy)
        if euclid > 0.05 and dist_m < 0.85 * euclid:
            raise ValueError(
                f"[{map_id}] Segment ({u}->{v}) dist_m={dist_m:.2f}m ngắn bất thường so với khoảng cách tọa độ {euclid:.2f}m"
            )

        raw_head = s.get("heading") if "heading" in s else s.get("heading_deg")
        if raw_head is None:
            head_deg = normalize_heading_deg(math.degrees(math.atan2(dy, dx)))
        else:
            try:
                head_deg = normalize_heading_deg(float(raw_head))
            except (TypeError, ValueError) as e:
                raise ValueError(f"[{map_id}] Segment ({u}->{v}) heading không hợp lệ: {e}") from e

        two_way = bool(s.get("two_way", True)) if "direction" not in s else (str(s["direction"]).upper() == "TWO_WAY")
        direction = "TWO_WAY" if two_way else "ONE_WAY"

        sid_fwd = f"SEG_{u}{v}"
        if sid_fwd not in seen_seg_ids:
            seen_seg_ids.add(sid_fwd)
            expanded_edges.append({
                "segment_id": sid_fwd,
                "from_node": u,
                "to_node": v,
                "distance_m": round(dist_m, 3),
                "direction": direction,
                "heading_deg": head_deg,
                "speed_limit_cm_s": round(spd, 1),
                "has_gap": has_gap,
            })
            adj_fwd[u].append(v)
            adj_rev[v].append(u)

        if two_way:
            sid_rev = f"SEG_{v}{u}"
            if sid_rev not in seen_seg_ids:
                seen_seg_ids.add(sid_rev)
                rev_head = normalize_heading_deg(head_deg + 180.0)
                expanded_edges.append({
                    "segment_id": sid_rev,
                    "from_node": v,
                    "to_node": u,
                    "distance_m": round(dist_m, 3),
                    "direction": "TWO_WAY",
                    "heading_deg": rev_head,
                    "speed_limit_cm_s": round(spd, 1),
                    "has_gap": has_gap,
                })
                adj_fwd[v].append(u)
                adj_rev[u].append(v)

    if len(expanded_edges) > MAX_MAP_EDGES:
        raise ValueError(f"[{map_id}] Số cạnh định hướng ({len(expanded_edges)}) vượt giới hạn firmware {MAX_MAP_EDGES}")

    # BFS reachability from and back to home_node
    def _bfs(start: str, adj: Dict[str, List[str]]) -> set:
        visited = {start}
        q = deque([start])
        while q:
            cur = q.popleft()
            for nxt in adj.get(cur, []):
                if nxt not in visited:
                    visited.add(nxt)
                    q.append(nxt)
        return visited

    reachable_out = _bfs(home_node, adj_fwd)
    reachable_in = _bfs(home_node, adj_rev)
    for nid in node_map:
        if nid not in reachable_out:
            raise ValueError(f"[{map_id}] Trạm '{nid}' không có đường đi tới từ trạm gốc '{home_node}'")
        if nid not in reachable_in:
            raise ValueError(f"[{map_id}] Từ trạm '{nid}' không có đường quay về trạm gốc '{home_node}'")

    # Determine initial heading (prefer first outgoing edge from home_node or explicit config)
    first_home_edges = [e for e in expanded_edges if e["from_node"] == home_node]
    default_init_head = first_home_edges[0]["heading_deg"] if first_home_edges else 90.0
    init_head = normalize_heading_deg(float(cfg.get("initial_heading_deg", default_init_head)))

    # Order expanded_edges cleanly: keep original forward insertion order so loops stay contiguous
    meta = {
        "map_id": map_id,
        "map_version": str(cfg.get("map_version", "1.0.0")).strip(),
        "name": name,
        "width_m": round(width_m, 3),
        "length_m": round(length_m, 3),
        "home_node": home_node,
        "initial_heading_deg": init_head,
        "corridor_guard_m": round(corridor_guard_m, 3),
        "description": str(cfg.get("description", "")).strip(),
        "default_rules": cfg.get("default_rules", []),
    }
    return meta, norm_nodes, expanded_edges


def compute_canonical_map_hash(
    map_id: str,
    nodes: List[Dict[str, Any]],
    edges: List[Dict[str, Any]],
) -> str:
    """Compute canonical 8-hex SHA-256 hash identical for JSON configs and SQLite DB rows."""
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
                round(float(e.get("distance_m") if "distance_m" in e else e["dist_m"]), 3),
                normalize_heading_deg(float(e.get("heading_deg") if "heading_deg" in e else e["heading"])),
            ]
            for e in edges
        ],
        key=lambda item: (item[0], item[1]),
    )
    payload = json.dumps(
        {"map_id": str(map_id).strip().upper(), "nodes": canon_nodes, "edges": canon_edges},
        separators=(",", ":"),
        ensure_ascii=True,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:8]


def _order_edges_for_firmware(
    nodes: List[Dict[str, Any]],
    edges: List[Dict[str, Any]],
    home_node: str,
) -> Tuple[List[Dict[str, Any]], List[int], List[int]]:
    """Order edges deterministically and extract forward/reverse closed loop index sequences."""
    edge_lookup: Dict[Tuple[str, str], int] = {}
    ordered_edges = list(edges)
    for idx, e in enumerate(ordered_edges):
        edge_lookup[(e["from_node"], e["to_node"])] = idx

    # Try finding a simple Hamiltonian cycle starting and ending at home_node
    n_nodes = len(nodes)
    adj: Dict[str, List[str]] = {n["node_id"]: [] for n in nodes}
    for e in ordered_edges:
        if e["to_node"] not in adj[e["from_node"]]:
            adj[e["from_node"]].append(e["to_node"])

    cycle_nodes: Optional[List[str]] = None

    def _dfs(cur: str, path: List[str], visited: set) -> bool:
        nonlocal cycle_nodes
        if len(path) == n_nodes:
            if home_node in adj.get(cur, []):
                cycle_nodes = path + [home_node]
                return True
            return False
        for nxt in adj.get(cur, []):
            if nxt not in visited:
                visited.add(nxt)
                path.append(nxt)
                if _dfs(nxt, path, visited):
                    return True
                path.pop()
                visited.remove(nxt)
        return False

    _dfs(home_node, [home_node], {home_node})

    fwd_loop: List[int] = []
    rev_loop: List[int] = []
    if cycle_nodes:
        for i in range(len(cycle_nodes) - 1):
            u, v = cycle_nodes[i], cycle_nodes[i + 1]
            fwd_loop.append(edge_lookup[(u, v)])
        # Reverse cycle (if all reverse edges exist)
        rev_candidates = []
        for i in range(len(cycle_nodes) - 1, 0, -1):
            u, v = cycle_nodes[i], cycle_nodes[i - 1]
            if (u, v) in edge_lookup:
                rev_candidates.append(edge_lookup[(u, v)])
        if len(rev_candidates) == len(fwd_loop):
            rev_loop = rev_candidates

    if not fwd_loop:
        # Fallback for non-Hamiltonian graphs (e.g., T-shape / star): follow valid outgoing/return chain
        fwd_loop = [0]
    if not rev_loop:
        rev_loop = list(reversed(fwd_loop))

    return ordered_edges, fwd_loop, rev_loop


def _is_canonical_rect4(meta: Dict[str, Any], nodes: List[Dict[str, Any]]) -> bool:
    if len(nodes) != 4:
        return False
    nmap = {n["node_id"]: n for n in nodes}
    if set(nmap.keys()) != {"A", "B", "C", "D"}:
        return False
    w, h = meta["width_m"], meta["length_m"]
    eps = 0.02
    return (
        abs(nmap["A"]["x"]) <= eps and abs(nmap["A"]["y"]) <= eps
        and abs(nmap["B"]["x"]) <= eps and abs(nmap["B"]["y"] - h) <= eps
        and abs(nmap["C"]["x"] - w) <= eps and abs(nmap["C"]["y"] - h) <= eps
        and abs(nmap["D"]["x"] - w) <= eps and abs(nmap["D"]["y"]) <= eps
    )


def generate_firmware_header(cfg: Dict[str, Any]) -> str:
    """Render C++ header `include/generated_map_config.h` from validated map JSON."""
    meta, nodes, edges = validate_and_expand_map(cfg)
    map_hash = compute_canonical_map_hash(meta["map_id"], nodes, edges)
    edges, fwd_loop, rev_loop = _order_edges_for_firmware(nodes, edges, meta["home_node"])

    node_idx = {n["node_id"]: idx for idx, n in enumerate(nodes)}
    home_idx = node_idx[meta["home_node"]]
    first_edge_idx = fwd_loop[0] if fwd_loop else 0
    is_rect4 = 1 if _is_canonical_rect4(meta, nodes) else 0

    lines: List[str] = [
        "// =============================================================================",
        "// AUTO-GENERATED BY tools/sync_map.py — DO NOT EDIT MANUALLY!",
        f"// Source map: {meta['map_id']} (v{meta['map_version']}, hash={map_hash})",
        "// =============================================================================",
        "#pragma once",
        "",
        f'#define ACTIVE_MAP_ID               "{_c_escape(meta["map_id"])}"',
        f'#define ACTIVE_MAP_HASH             "{map_hash}"',
        f'#define ACTIVE_MAP_HOME_NODE_ID     "{_c_escape(meta["home_node"])}"',
        f"#define ACTIVE_MAP_WIDTH_M          {meta['width_m']:.3f}f",
        f"#define ACTIVE_MAP_LENGTH_M         {meta['length_m']:.3f}f",
        f"#define ACTIVE_CORRIDOR_GUARD_M     {meta['corridor_guard_m']:.3f}f",
        f"#define ACTIVE_INITIAL_HEADING_DEG  {meta['initial_heading_deg']:.1f}f",
        f"#define ACTIVE_HOME_STATION_IDX     {home_idx}",
        f"#define ACTIVE_FIRST_EDGE_IDX       {first_edge_idx}",
        f"#define ACTIVE_MAP_IS_RECT4         {is_rect4}",
        f"#define GENERATED_NUM_NODES         {len(nodes)}",
        f"#define GENERATED_NUM_EDGES         {len(edges)}",
        f"#define GENERATED_LOOP_LEN          {len(fwd_loop)}",
        "",
        "struct GeneratedMapNode {",
        "  const char* id;",
        "  float x;",
        "  float y;",
        "  bool isInbound;",
        "};",
        "",
        "struct GeneratedMapEdge {",
        "  int fromIdx;",
        "  int toIdx;",
        "  float distM;",
        "  float headingDeg;",
        "  float speedLimitCmS;",
        "  const char* name;",
        "};",
        "",
        f"static const GeneratedMapNode GENERATED_MAP_NODES[GENERATED_NUM_NODES] = {{",
    ]
    for n in nodes:
        is_in = "true" if n["type"] == "INBOUND" else "false"
        lines.append(
            f'  {{ "{_c_escape(n["node_id"])}", {n["x"]:.3f}f, {n["y"]:.3f}f, {is_in} }},'
        )
    lines.extend([
        "};",
        "",
        f"static const GeneratedMapEdge GENERATED_MAP_EDGES[GENERATED_NUM_EDGES] = {{",
    ])
    for e in edges:
        f_idx = node_idx[e["from_node"]]
        t_idx = node_idx[e["to_node"]]
        ename = f"{e['from_node']} -> {e['to_node']}"
        lines.append(
            f'  {{ {f_idx}, {t_idx}, {e["distance_m"]:.3f}f, {e["heading_deg"]:.1f}f, '
            f'{e["speed_limit_cm_s"]:.1f}f, "{_c_escape(ename)}" }},'
        )
    lines.extend([
        "};",
        "",
        f"static const int GENERATED_LOOP_FWD_EDGES[GENERATED_LOOP_LEN] = {{ {', '.join(str(x) for x in fwd_loop)} }};",
        f"static const int GENERATED_LOOP_REV_EDGES[GENERATED_LOOP_LEN] = {{ {', '.join(str(x) for x in rev_loop)} }};",
        "",
    ])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate & sync AGV_Ver2 map configuration to Server & Firmware.")
    parser.add_argument("--map", type=str, default=None, help="Path to map JSON to set as config/active_map.json")
    parser.add_argument("--check", action="store_true", help="Validate configs & check header sync without writing")
    parser.add_argument("--apply-db", action="store_true", help="Non-destructively sync active map into Server SQLite DB")
    args = parser.parse_args()

    # Validate all preset maps under config/maps/
    if MAPS_DIR.is_dir():
        for p in sorted(MAPS_DIR.glob("*.json")):
            cfg_p = json.loads(p.read_text(encoding="utf-8"))
            meta_p, n_p, e_p = validate_and_expand_map(cfg_p)
            h_p = compute_canonical_map_hash(meta_p["map_id"], n_p, e_p)
            print(f"[VALID] Preset {p.name}: map_id={meta_p['map_id']} ({len(n_p)} nodes, {len(e_p)} edges, hash={h_p})")

    if args.map:
        src = Path(args.map)
        if not src.is_absolute():
            src = (ROOT_DIR / src).resolve()
        if not src.is_file():
            print(f"[ERROR] Không tìm thấy file bản đồ: {src}", file=sys.stderr)
            return 1
        cfg = json.loads(src.read_text(encoding="utf-8"))
        validate_and_expand_map(cfg)
        if not args.check:
            CONFIG_DIR.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, ACTIVE_MAP_PATH)
            print(f"[UPDATED] Đã cập nhật active_map.json từ {src.name}")

    if not ACTIVE_MAP_PATH.is_file():
        print(f"[ERROR] Không tìm thấy {ACTIVE_MAP_PATH}", file=sys.stderr)
        return 1

    active_cfg = json.loads(ACTIVE_MAP_PATH.read_text(encoding="utf-8"))
    meta, nodes, edges = validate_and_expand_map(active_cfg)
    map_hash = compute_canonical_map_hash(meta["map_id"], nodes, edges)
    rendered_header = generate_firmware_header(active_cfg)

    if args.check:
        if not HEADER_PATH.is_file():
            print(f"[STALE] Thiếu file {HEADER_PATH}. Chạy: python tools/sync_map.py", file=sys.stderr)
            return 1
        current = HEADER_PATH.read_text(encoding="utf-8").replace("\r\n", "\n")
        expected = rendered_header.replace("\r\n", "\n")
        if current != expected:
            print(f"[STALE] {HEADER_PATH} không khớp với {ACTIVE_MAP_PATH}! Chạy: python tools/sync_map.py", file=sys.stderr)
            return 1
        print(f"[OK] Header {HEADER_PATH.name} khớp hoàn toàn với {meta['map_id']} (hash={map_hash}).")
        return 0

    HEADER_PATH.parent.mkdir(parents=True, exist_ok=True)
    HEADER_PATH.write_text(rendered_header, encoding="utf-8", newline="\n")
    print(f"[GENERATED] {HEADER_PATH} -> map_id={meta['map_id']} (hash={map_hash}, {len(nodes)} nodes, {len(edges)} edges)")

    if args.apply_db:
        server_dir = ROOT_DIR / "server"
        if str(server_dir) not in sys.path:
            sys.path.insert(0, str(server_dir))
        from database.seed_data import sync_active_map_db
        sync_active_map_db()
        print(f"[SYNCED DB] Đã đồng bộ an toàn bản đồ '{meta['map_id']}' vào SQLite (bảo toàn tồn kho).")

    return 0


if __name__ == "__main__":
    sys.exit(main())
