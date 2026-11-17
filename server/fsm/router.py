"""
===================================================================================
BỘ ĐỊNH TUYẾN ĐỒ THỊ LINE TÔ-PÔ (TOPOLOGICAL LINE ROUTER V2.1)
SỬ DỤNG THUẬT TOÁN DIJKSTRA TỐI ƯU KHOẢNG CÁCH & TRÁNH CUNG ĐƯỜNG BỊ KHÓA
===================================================================================
"""

from typing import List, Dict, Any, Optional
import heapq

class TopologicalRouter:
    """Bộ định tuyến đồ thị phẳng dựa trên các đoạn line đo đạc thực tế của sa bàn"""
    def __init__(self, segments: List[Dict[str, Any]]):
        self.segments = segments
        self.graph: Dict[str, List[Dict[str, Any]]] = {}
        self._build_graph()

    def _build_graph(self):
        """Xây dựng danh sách kề (Adjacency List) từ bảng line_segments"""
        self.graph = {}
        for seg in self.segments:
            u = seg["from_node"]
            v = seg["to_node"]
            dist = float(seg["distance_m"])
            is_locked = bool(seg.get("is_locked", 0))
            locked_by = seg.get("locked_by_agv")

            if u not in self.graph:
                self.graph[u] = []
            self.graph[u].append({
                "to_node": v,
                "segment_id": seg["segment_id"],
                "distance": dist,
                "heading": float(seg.get("heading_deg", 0.0)),
                "is_locked": is_locked,
                "locked_by": locked_by
            })

            # Nếu là đường 2 chiều mà chưa có bản ghi ngược lại
            if seg.get("direction") == "TWO_WAY":
                if v not in self.graph:
                    self.graph[v] = []

    def find_shortest_path(self, start_node: str, target_node: str, 
                           current_agv_id: str = "AGV_Ver2") -> Optional[Dict[str, Any]]:
        """
        Tìm lộ trình ngắn nhất từ start_node đến target_node:
          - Tránh các đoạn line đang bị xe khác khóa (is_locked == 1 và locked_by != current_agv_id)
          - Trả về danh sách Node, Segment IDs, Tổng khoảng cách (m), và chuỗi lộ trình (A->B)
        """
        if start_node == target_node:
            return {
                "nodes": [start_node],
                "segments": [],
                "total_distance_m": 0.0,
                "route_str": start_node
            }

        # Dijkstra algorithm
        distances = {node: float('inf') for node in self.graph}
        distances[start_node] = 0.0
        previous_nodes: Dict[str, Optional[str]] = {node: None for node in self.graph}
        previous_segments: Dict[str, Optional[Dict[str, Any]]] = {node: None for node in self.graph}

        pq = [(0.0, start_node)]

        while pq:
            current_dist, u = heapq.heappop(pq)

            if current_dist > distances.get(u, float('inf')):
                continue

            if u == target_node:
                break

            for edge in self.graph.get(u, []):
                v = edge["to_node"]
                dist = edge["distance"]

                # Nếu đoạn line đang bị xe khác khóa -> Bỏ qua không đi vào (Tránh đâm xe)
                if edge["is_locked"] and edge["locked_by"] not in (None, current_agv_id):
                    continue

                new_dist = current_dist + dist
                if new_dist < distances.get(v, float('inf')):
                    distances[v] = new_dist
                    previous_nodes[v] = u
                    previous_segments[v] = edge
                    heapq.heappush(pq, (new_dist, v))

        # Kiểm tra xem có đến được đích không
        if distances.get(target_node, float('inf')) == float('inf'):
            return None # Không có đường đi khả dụng (Do bị khóa hoặc không thông đường)

        # Truy vết lộ trình
        path_nodes = []
        path_segments = []
        curr = target_node
        while curr:
            path_nodes.append(curr)
            seg = previous_segments.get(curr)
            if seg:
                path_segments.append(seg["segment_id"])
            curr = previous_nodes.get(curr)

        path_nodes.reverse()
        path_segments.reverse()

        return {
            "nodes": path_nodes,
            "segments": path_segments,
            "total_distance_m": round(distances[target_node], 3),
            "route_str": "->".join(path_nodes)
        }
