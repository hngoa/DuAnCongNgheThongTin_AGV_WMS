# ĐẶC TẢ CHI TIẾT MÁY TRẠNG THÁI & LUỒNG HOẠT ĐỘNG
# AGV_VER2 WMS SYSTEM (PHIÊN BẢN V2.3)
> **Căn cứ kỹ thuật:** Quản lý vòng đời nhiệm vụ (Task FSM), máy trạng thái điều khiển thực tế trên Firmware `test_10` và khóa cung đường sa bàn.  
> **Phiên bản:** 2.3 (Map-as-Code + CV Ver3) | **Cập nhật:** 2026-11-19

---

## 1. KIẾN TRÚC MÁY TRẠNG THÁI PHÂN TẦNG (HIERARCHICAL FSM)

```
┌────────────────────────────────────────────────────────────────────────┐
│                   HỆ THỐNG MÁY TRẠNG THÁI AGV_VER2 WMS                 │
├──────────────────┬────────────────────────────┬────────────────────────┤
│ 1. TASK FSM      │ 2. AGV MOTION & SAFETY FSM │ 3. TRAFFIC SEGMENT FSM │
│ (Vòng đời        │ (Trạng thái thực thi trên  │ (Khóa đoạn line trên   │
│  nhiệm vụ kho)   │  ESP32-S3 test_10 & Server)│  bản đồ đa chặng)      │
└──────────────────┴────────────────────────────┴────────────────────────┘
```

---

## 2. CHI TIẾT CÁC MÁY TRẠNG THÁI

### 2.1. Máy Trạng Thái 1: Vòng Đời Nhiệm Vụ (`TaskStateMachine` & `WMSWorkflowManager`)

Quản lý trọn vẹn chu trình **Shot 1 (Nhận hàng tại `home_node`) $\rightarrow$ Vận chuyển $\rightarrow$ Shot 2 (Xác nhận dỡ hàng tại trạm đích)**:

```mermaid
stateDiagram-v2
    [*] --> PENDING : App Android upload ảnh Shot 1
    
    PENDING --> ALLOCATING : CV Ver3 (YOLO11n + MNv3 + Geometry) phân loại thành công (Conf >= 0.50)
    PENDING --> REJECTED_UNKNOWN : Conf < 0.50 (hoặc chặn từ Safety Gate)
    
    ALLOCATING --> DISPATCHED : Có trạm kho trống + Khóa line + Xếp lệnh GO_TO
    ALLOCATING --> REJECTED_STOCK_FULL : Không tìm được kho còn chỗ
    
    DISPATCHED --> IN_TRANSIT : ESP32 gửi ACK khớp cmd_id
    
    IN_TRANSIT --> DELIVERED : Xe chạm mốc trạm đích (AT_STATION)
    IN_TRANSIT --> ABORTED : Lỗi giữa đường / Watchdog / Hủy thủ công
    
    DELIVERED --> UNLOADING : Chờ Camera chụp xác nhận dỡ hàng (Shot 2)
    UNLOADING --> COMPLETED : Shot 2 xác nhận đã dỡ (+1 tồn kho)
    UNLOADING --> ABORTED : Quá thời gian chờ hoặc hủy nhiệm vụ
    
    COMPLETED --> [*]
    REJECTED_UNKNOWN --> [*]
    REJECTED_STOCK_FULL --> [*]
    ABORTED --> [*]
```

#### Bảng Chuyển Trạng Thái Chính:
| Trạng thái | Sự kiện kích hoạt | Điều kiện kiểm tra (Guard) | Trạng thái tiếp | Hành động |
| :--- | :--- | :--- | :--- | :--- |
| `PENDING` | `AI_CLASSIFIED` | `Conf >= 0.50`, không rơi vào `NO_OBJECT` / `NEEDS_REVIEW` | `ALLOCATING` | Tra `products` & `allocation_rules` |
| `ALLOCATING` | `TARGET_FOUND` | Có trạm còn `available_slots > 0` | `DISPATCHED` | Tìm đường TSP Dijkstra, khóa `line_segments`, cấp `GO_TO:<Node>` |
| `DISPATCHED` | `AGV_ACK_RECEIVED` | Telemetry `ack_id == cmd_id` | `IN_TRANSIT` | Cập nhật `started_at`, theo dõi lộ trình |
| `IN_TRANSIT` | `ARRIVED_AT_DEST` | Xe báo `AT_STATION` đúng `to_node` | `DELIVERED` $\rightarrow$ `UNLOADING` | Kích hoạt Camera chụp Shot 2 (`VERIFY_UNLOAD`) |
| `UNLOADING` | `VISION_UNLOAD_OK` | Khay trống hoặc hết hàng thuộc trạm hiện tại | `COMPLETED` | Cộng `+1` `inventory`, mở khóa line, chạy tiếp hoặc về `home_node` |
| `IN_TRANSIT`/`UNLOADING` | `FAULT`/`ABORT` | `GAP_TIMEOUT_STOP`, `LOST_LINE_STOP`, `ABORT` | `ABORTED` | Hủy mission kẹt, giải phóng khóa line |

---

### 2.2. Máy Trạng Thái 2: Vận Hành & Cứu Hộ Trên Firmware `test_10`

Đồng bộ giữa các sub-state thực tế trong `test/test_10_line_centric_hybrid_recovery.cpp` và Server `AGVState`:

```mermaid
stateDiagram-v2
    [*] --> OFFLINE : Server khởi động (hoặc timeout > 6.0s)
    OFFLINE --> IDLE : Nhận Telemetry hợp lệ từ ESP32-S3
    
    IDLE --> TURNING_STATION : Sai lệch góc xuất phát > 6.0° (SUB_TURNING_FOR_MISSION)
    IDLE --> RUNNING_FAST : Đúng hướng <= 6.0° (SUB_LINE_FOLLOWING)
    
    TURNING_STATION --> RUNNING_FAST : Khóa vạch IR + MPU trong ngưỡng (hoặc quét ±11°)
    RUNNING_FAST --> TURNING_LINE : Ôm cua gắt / Mất tại góc gập (SUB_CORNER_BEND_CATCH)
    TURNING_LINE --> RUNNING_FAST : Bắt lại nhánh vạch mới trong nón ±92°
    
    RUNNING_FAST --> DEAD_RECKONING_BRIDGE : Đứt vạch thẳng (quét sin có dẫn hướng MPU)
    DEAD_RECKONING_BRIDGE --> RUNNING_FAST : Tìm lại được vạch line hợp lệ
    DEAD_RECKONING_BRIDGE --> GAP_TIMEOUT_STOP : Vượt quá giới hạn cầu nối đứt vạch
    
    RUNNING_FAST --> APPROACH_CORNER : Khoảng cách đích <= ngưỡng tiếp cận trạm
    APPROACH_CORNER --> AT_STATION : Chạm vạch ngang trạm & phanh dừng (Corner Anchor)
    AT_STATION --> TURNING_STATION : Có chặng kế tiếp trong legQueue / RETURN về home_node
    
    RUNNING_FAST --> EMERGENCY_STOP : Lạc hành lang tuyến / Nhận STOP / Bấm nút
```

*(Lưu ý: Trên tầng Server `AGVState`, `RUNNING_FAST`, `TURNING_STATION`, `TURNING_LINE`, `APPROACH_CORNER`, `DEAD_RECKONING_BRIDGE` được ánh xạ tổng quát vào trạng thái di chuyển `RUNNING` và chi tiết thô vẫn được lưu đầy đủ trong `telemetry_logs.state`.)*

---

### 2.3. Máy Trạng Thái 3: Khóa Cung Đường (`line_segments.is_locked`)

```mermaid
stateDiagram-v2
    [*] --> UNLOCKED : Khởi tạo map / Server restart
    UNLOCKED --> LOCKED : lock_segments(map_id, segments, agv_id)
    LOCKED --> UNLOCKED : Hoàn tất mission / Return home / Fault recovery / Watchdog Offline
```

---

## 3. ĐỊNH TUYẾN DIJKSTRA (SERVER & FIRMWARE OFFLINE)

1. **Trên Server (`TopologicalRouter.find_shortest_path`):** Tìm đường ngắn nhất theo `distance_m` trên toàn đồ thị, bỏ qua các `segment_id` đang bị khóa bởi AGV khác, xuất `route_legs` (`from`, `to`, `dist`, `head`, `tx`, `ty`) gửi xuống ESP32.
2. **Dự phòng trên Firmware (`planAndExecuteRoute`):** Sử dụng bảng `runtimeNodes` / `runtimeEdges` (nạp từ `generated_map_config.h`, tối đa 16 trạm / 64 chặng) chạy Dijkstra $O(N^2)$ để vẫn điều hướng Point-to-Point được khi chạy Serial offline.
