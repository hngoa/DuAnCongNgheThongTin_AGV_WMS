# TÀI LIỆU ĐẶC TẢ KIẾN TRÚC & LUỒNG HOẠT ĐỘNG TOÀN HỆ THỐNG
# AGV_VER2 - WMS SMART DISPATCHING & MAP-AS-CODE ARCHITECTURE
> **Phiên bản:** 2.3 (Map-as-Code + CV Ver3 YOLO11n-MobileNetV3-Geometry + 2-Shot Vision Workflow)  
> **Cập nhật:** 2026-11-19 | Đồng bộ hoàn toàn theo kiến trúc Server, Firmware ESP32-S3 & AI CV Ver3 hiện tại

---

## 1. TỔNG QUAN KIẾN TRÚC HỆ THỐNG (SYSTEM OVERVIEW)

Hệ thống hoạt động theo mô hình **Logistics 4.0 Tập trung, Map-as-Code & Thị giác AI CV Ver3**:
- **Bản đồ Map-as-Code đồng bộ 2 chiều:** Mọi bản đồ tô-pô được định nghĩa tập trung trong `config/maps/*.json` và `config/active_map.json`, kiểm chứng bằng `tools/sync_map.py` để sinh đồng thời dữ liệu SQLite (non-destructive UPSERT) và header C++ `include/generated_map_config.h` cho ESP32-S3 kèm mã băm đối chiếu `MAP_HASH`.
- **Thị giác AI CV Ver3 (`SLAMCVRecognizer`):** Nhận diện đồng thời 1 hoặc nhiều vật thể (`cylinder`, `rectangular_box`, `triangular_prism`) qua chuỗi 3 bước **YOLO11n (`server/weights/yolo11n_v5b_best.pt`) + MobileNetV3-Small (`server/weights/best.pth`) + OpenCV Geometric Reasoning (`GeometricAnalyzer`)** cùng cơ chế tách box gộp, giải quyết box lồng nhau và đối chiếu nhất quán toàn cục (`_reconcile_scene_objects`), đi qua 3 cổng an toàn (`NO_OBJECT`, `NEEDS_REVIEW`, `CLASSIFIED`).
- **Tồn kho Đa Sản phẩm & Phân bổ Gần nhất:** Mỗi trạm có thể chứa nhiều loại hàng, kho đích được chọn qua `AllocationEngineRepository` (quy tắc theo mã hàng > nhóm ngành > `*` kết hợp Dijkstra lộ trình ngắn nhất).

```
 ┌────────────────┐       ┌────────────────┐
 │ 📱 Phone App   │       │ 🚗 AGV_Ver2    │
 │ (Android Cam)  │       │ (ESP32-S3)     │
 └───────┬────────┘       └───────┬────────┘
         │ HTTP POST + Token      │ HTTP POST Telemetry (~10Hz) + HMAC-SHA256
         ▼                        ▼
 ┌─────────────────────────────────────────┐
 │ 🖥️ WMS BACKEND SERVER (Python Standard)  │
 │   Entry: server/run_server.py (Port 5000│
 │   + mDNS RFC 6762: agv-wms.local)       │
 │ ├─ CV Ver3: YOLO11n + MNv3 + Geometry   │
 │ ├─ Dynamic Allocation & Dijkstra Router │
 │ ├─ Hierarchical FSM & Traffic Guard     │
 │ └─ SQLite WAL DB + Map-as-Code Sync     │
 └────────────────────┬────────────────────┘
                      │ HTTP REST Polling (X-Device-Token)
                      ▼
 ┌─────────────────────────────────────────┐
 │ 💻 WEB WMS DASHBOARD (Digital Twin)     │
 │ ├─ Canvas 2D theo Map-as-Code + Cảnh báo│
 │ │  lệch MAP_HASH giữa Server & Firmware │
 │ ├─ Camera Feed, BBox AI & Auto Shot 1   │
 │ ├─ Tồn kho đa mặt hàng & Nhật ký /logs  │
 └─────────────────────────────────────────┘
```

---

## 2. BẢN ĐỒ TÔ-PÔ ĐƯỜNG LINE & MAP-AS-CODE

### 2.1. Bản chất Vật lý & Quản trị Bản đồ
Xe AGV bám theo vạch kẻ line trên sàn bằng mảng 5 mắt TCRT5000, kết hợp Encoder, MPU-6050 (EKF) và cầu nối quán tính (`Dead-Reckoning Virtual Line Bridge`):
1. **Điểm nút (`map_nodes`):**
   - `INBOUND`: Trạm tiếp nhận & chụp ảnh đầu vào (`home_node`, mặc định tìm theo `node_type='INBOUND'`, không hardcode tên `'A'`).
   - `STATION`: Trạm kho dừng dỡ hàng (có `max_capacity`).
   - `INTERSECTION` / `CHARGING` / `PARKING`: Điểm giao cắt, trạm sạc hoặc điểm đỗ.
2. **Đoạn line kết nối (`line_segments`):** Chiều dài thực tế `distance_m`, hướng `ONE_WAY` / `TWO_WAY`, góc la bàn chuẩn `heading_deg`, giới hạn tốc độ `speed_limit_cm_s` và cờ `has_gap`.
3. **Khóa sửa đổi runtime mặc định (`ALLOW_RUNTIME_MAP_CHANGE = False`):** Trực tiếp đổi/nạp bản đồ qua Web UI trả về `403 MAP_LOCKED_BY_CODE` để ngăn lệch tọa độ EKF giữa Server và Firmware. Quy trình thay đổi bản đồ chuẩn thực hiện qua `tools/sync_map.py` (chi tiết tại `docs/MAP_UPDATE_WORKFLOW.md`).

### 2.2. Các bản đồ mẫu đang cấu hình
- `config/maps/saban_154_151.json`: Sa bàn Lab 4 trạm `A, B, C, D` (`1.54m x 1.51m`, 8 chặng 2 chiều).
- `config/maps/map_exp_6nodes.json`: Kho mở rộng 6 trạm `A..F` (`2.50m x 1.80m`, 12 chặng 2 chiều).

---

## 3. QUY TRÌNH 2-SHOT VISION & PHÂN BỔ KHO

### 3.1. Ánh xạ Nhãn CV sang Kho WMS
| Lớp Shape CV (`yolo11n_v5b_best.pt` + `best.pth`) | Mã hàng mặc định | Nhóm ngành | Trạm đích mặc định |
| :--- | :--- | :--- | :--- |
| `cylinder` (ID 0) | `PKG_MECH` | `MECHANICAL` | Trạm `C` |
| `rectangular_box` (ID 1) | `PKG_ELEC` | `ELECTRONIC` | Trạm `B` |
| `triangular_prism` (ID 2) | `PKG_TOOL` | `TOOL` | Trạm `D` |

*(Metadata & trạm đích thực tế được tra động từ SQLite `products` và `allocation_rules` theo bản đồ đang hoạt động.)*

### 3.2. Luồng Vận Hành Khép Kín (2-Shot Vision Workflow)
```
[1. AGV đỗ ổn định tại home_node (INBOUND)]
       │ Auto Shot 1 (CameraController.check_auto_inbound_capture)
       ▼
[2. Android Camera -> POST /api/camera/upload (Shot 1)]
       │ CV Ver3 (YOLO11n + MobileNetV3 + OpenCV Geometry + Scene Reconcile)
       ├─ NO_OBJECT / NEEDS_REVIEW -> Từ chối tạo đơn (Safety Gate)
       ▼ (CLASSIFIED)
[3. Allocation Engine + TSP Dijkstra Optimization]
       ├─ Tính hoán vị tối ưu qua các trạm đích + đường quay về home_node
       ├─ Tạo Dispatch Tasks, khóa line_segments, xếp lệnh GO_TO:<ToNode> + route_legs
       ▼
[4. ESP32-S3 nhận lệnh (ký HMAC-SHA256) -> Lăn bánh & ACK cmd_id]
       │ Task: DISPATCHED -> IN_TRANSIT -> DELIVERED -> UNLOADING
       ▼
[5. AGV tới trạm đích -> Camera Trigger Shot 2 (VERIFY_UNLOAD)]
       ├─ Hàng vẫn còn -> CARGO_STILL_PRESENT (giữ xe đứng chờ)
       ▼ (Khay trống hoặc hàng trạm hiện tại đã dỡ)
[6. Task COMPLETED -> Cộng tồn kho (+1) -> Chạy Mission kế hoặc GO_TO:<home_node>]
```

---

## 4. MÔ HÌNH DỮ LIỆU SQLITE (`server/database/schema.sql`)

- **Bản đồ & Mạng lưới:** `maps`, `map_nodes`, `line_segments`
- **Hàng hóa & Kho:** `products`, `inventory`, `allocation_rules`
- **Đội xe & Nhiệm vụ:** `agv_fleet`, `dispatch_tasks`, `telemetry_logs`, `system_events`
