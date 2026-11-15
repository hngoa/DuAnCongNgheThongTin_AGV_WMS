# ĐẶC TẢ CHI TIẾT MÁY TRẠNG THÁI & LUỒNG HOẠT ĐỘNG
# AGV_VER2 WMS SYSTEM (PHIÊN BẢN V2.1)
> **Căn cứ kỹ thuật:** Quản lý vòng đời nhiệm vụ (Task Lifecycle), máy trạng thái xe AGV và khóa cung đường sa bàn (Traffic Zone Locking).  
> **Cập nhật:** 2026-11-05

---

## 1. TỔNG QUAN KIẾN TRÚC MÁY TRẠNG THÁI PHÂN TẦNG (HIERARCHICAL FSM)

Hệ thống được thiết kế theo mô hình **Máy Trạng Thái Hữu Hạn Phân Tầng (Hierarchical Finite State Machine - HFSM)** gồm 3 máy trạng thái chạy song song và tương tác chặt chẽ với nhau:

```
┌────────────────────────────────────────────────────────────────────────┐
│                   HỆ THỐNG MÁY TRẠNG THÁI AGV_VER2 WMS                 │
├──────────────────┬────────────────────────────┬────────────────────────┤
│ 1. TASK FSM      │ 2. AGV MOTION & SAFETY FSM │ 3. TRAFFIC SEGMENT FSM │
│ (Quản lý vòng    │ (Quản lý hành vi di chuyển │ (Quản lý khóa đoạn     │
│  đời Đơn hàng)   │  và an toàn của xe AGV)    │  line sa bàn đa xe)    │
└──────────────────┴────────────────────────────┴────────────────────────┘
```

---

## 2. CHI TIẾT CÁC MÁY TRẠNG THÁI

### 2.1. Máy Trạng Thái 1: Vòng Đời Nhiệm Vụ (Task Lifecycle FSM)

Máy trạng thái này quản lý toàn bộ quá trình từ lúc kiện hàng được đặt lên xe tại Trạm Nhập A, AI nhận diện, WMS phân bổ kho, xe vận chuyển, dỡ hàng cho đến khi kết thúc.

```mermaid
stateDiagram-v2
    [*] --> PENDING : 📱 App Android upload ảnh kiện hàng
    
    PENDING --> ALLOCATING : AI nhận diện thành công (Conf >= 0.50)
    PENDING --> REJECTED_UNKNOWN : AI không rõ loại hàng (Conf < 0.50)
    
    ALLOCATING --> DISPATCHED : Tìm thấy kho đích còn chỗ trống (Available > 0)
    ALLOCATING --> REJECTED_STOCK_FULL : Tất cả các kho phù hợp đã đầy
    
    DISPATCHED --> IN_TRANSIT : ESP32 nhận lệnh và gửi ACK khớp cmd_id
    
    IN_TRANSIT --> DELIVERED : Xe chạm mốc mỏ neo trạm đích an toàn
    IN_TRANSIT --> ABORTED : Sự cố (Lạc line / Pin < 10.5V / Mất Wi-Fi)
    
    DELIVERED --> UNLOADING : Bắt đầu đếm lùi Dwell Time (5.0s)
    UNLOADING --> COMPLETED : Hết Dwell Time -> Tồn kho đích tăng +1
    
    COMPLETED --> [*]
    REJECTED_UNKNOWN --> [*]
    REJECTED_STOCK_FULL --> [*]
    ABORTED --> [*]
```

#### Bảng Ma Trận Chuyển Trạng Thái Của Task:
| Trạng thái hiện tại | Sự kiện kích hoạt (Event) | Điều kiện kiểm tra (Guard) | Trạng thái tiếp theo | Hành động thực thi (Action) |
| :--- | :--- | :--- | :--- | :--- |
| **None** | `IMAGE_UPLOADED` | File ảnh hợp lệ | `PENDING` | Lưu ảnh, kích hoạt AI Worker xử lý |
| `PENDING` | `AI_CLASSIFIED` | `Confidence >= 0.50` | `ALLOCATING` | Lưu mã sản phẩm, gọi Allocation Engine |
| `PENDING` | `AI_LOW_CONFIDENCE` | `Confidence < 0.50` | `REJECTED_UNKNOWN` | Ghi log cảnh báo lên Web Dashboard |
| `ALLOCATING` | `TARGET_FOUND` | `available_slots > 0` | `DISPATCHED` | Lập lộ trình, nạp lệnh `pending_command` |
| `ALLOCATING` | `ALL_STATIONS_FULL` | `available_slots == 0`| `REJECTED_STOCK_FULL` | Bật còi báo kho đầy trên Web |
| `DISPATCHED` | `AGV_ACK_RECEIVED` | `ack_id == cmd_id` | `IN_TRANSIT` | Khóa các đoạn line tương ứng trên sa bàn |
| `IN_TRANSIT` | `ARRIVED_AT_DEST` | Xe báo `AT_STATION` | `DELIVERED` | Kích hoạt bộ đếm thời gian Dwell Time |
| `IN_TRANSIT` | `SAFETY_TRIGGERED` | Lạc vạch hoặc Pin yếu| `ABORTED` | Phanh dừng khẩn cấp, mở khóa đường line |
| `DELIVERED` | `DWELL_TIME_EXPIRED` | Đã đỗ đủ $5.0\text{giây}$| `COMPLETED` | Tăng số lượng tồn kho $+1$, gửi WebSocket |

---

### 2.2. Máy Trạng Thái 2: Trạng Thái Vận Hành & An Toàn Xe AGV (AGV Motion & Safety FSM)

Máy trạng thái này chạy đồng bộ giữa Server và Vi điều khiển ESP32-S3:

```mermaid
stateDiagram-v2
    [*] --> OFFLINE : Khởi động server
    
    OFFLINE --> IDLE : Nhận gói tin Telemetry đầu tiên qua Wi-Fi
    
    IDLE --> STARTING : Nhận lệnh điều phối mới (GO_TO:X)
    IDLE --> LOW_BATTERY : Pin tụt < 10.5V
    
    STARTING --> LINE_FOLLOWING : Khởi động mềm PWM, khóa góc la bàn Yaw
    
    LINE_FOLLOWING --> GAP_CROSSING : Mất vạch line (Line mask = 00000)
    GAP_CROSSING --> LINE_FOLLOWING : Bắt lại vạch line bằng mảng 5 mắt
    GAP_CROSSING --> EMERGENCY_STOP : Mất vạch quá 35cm (Dead Reckoning timeout)
    
    LINE_FOLLOWING --> DECELERATING : Cách đích <= 35cm (Theo Encoder)
    DECELERATING --> AT_STATION : Chạm mốc vạch ngang trạm đích, tốc độ = 0
    
    AT_STATION --> UNLOADING : Xe đỗ yên, giữ phanh cứng
    UNLOADING --> RETURNING_HOME : Nếu bật AUTO_RETURN_HOME (Về Trạm A)
    UNLOADING --> IDLE : Nếu tắt AUTO_RETURN_HOME (Đỗ chờ đơn tiếp)
    
    RETURNING_HOME --> LINE_FOLLOWING : Chạy bám line về lại Trạm A
    
    LOW_BATTERY --> RETURNING_CHARGING : Tự động phát lệnh về Trạm Sạc
    
    LINE_FOLLOWING --> EMERGENCY_STOP : Vật cản / Mất Wi-Fi > 3.0s / Bấm E-Stop
    EMERGENCY_STOP --> IDLE : Nhân viên kiểm tra và bấm "Reset An Toàn"
```

#### Bảng Ma Trận Chuyển Trạng Thái Của Xe AGV:
| Trạng thái hiện tại | Sự kiện kích hoạt (Event) | Điều kiện kiểm tra (Guard) | Trạng thái tiếp theo | Hành vi phần cứng (ESP32 / Server) |
| :--- | :--- | :--- | :--- | :--- |
| `OFFLINE` | `HEARTBEAT_RECEIVED` | Gói HTTP hợp lệ | `IDLE` | Server bật đèn xanh Online trên Web |
| `IDLE` | `TASK_DISPATCHED` | Có lệnh `GO_TO:<Node>` | `STARTING` | Tăng dần PWM động cơ (Khởi động mềm) |
| `IDLE` | `VOLTAGE_LOW` | Pin $< 10.5\text{V}$ | `LOW_BATTERY` | Khóa nhận đơn mới, cảnh báo sạc pin |
| `STARTING` | `MOTION_INITIATED` | Tốc độ $> 5\text{cm/s}$ | `LINE_FOLLOWING` | Bật thuật toán điều khiển bám vạch PD |
| `LINE_FOLLOWING` | `LINE_LOST` | Cảm biến `mask == 0` | `GAP_CROSSING` | Chạy thẳng quán tính bằng MPU-6050 |
| `GAP_CROSSING` | `LINE_REACQUIRED` | Cảm biến `mask > 0` | `LINE_FOLLOWING` | Khóa lại tâm vạch, tiếp tục chạy PD |
| `GAP_CROSSING` | `GAP_DISTANCE_EXCEEDED`| Chạy $> 35\text{cm}$ vẫn mất| `EMERGENCY_STOP` | Phanh dừng tức thì, hú còi Buzzer |
| `LINE_FOLLOWING` | `APPROACHING_TARGET`| Khoảng cách còn $\le 35\text{cm}$| `DECELERATING` | Giảm tốc độ từ $22\text{cm/s} \rightarrow 12\text{cm/s}$ |
| `DECELERATING` | `STATION_ANCHOR_DETECTED`| Cảm biến gặp vạch ngang| `AT_STATION` | Ngắt động cơ, khóa phanh chủ động |
| `AT_STATION` | `UNLOAD_STARTED` | Đã dừng đỗ an toàn | `UNLOADING` | Giữ phanh đỗ, chờ dỡ hàng $5.0\text{s}$ |
| Bất kỳ | `WATCHDOG_TIMEOUT` | Mất tín hiệu $> 3.0\text{s}$ | `EMERGENCY_STOP` | Hardware Failsafe: Tự ngắt motor |

---

### 2.3. Máy Trạng Thái 3: Khóa Cung Đường Sa Bàn (Traffic Segment Lock FSM)

Phục vụ bài toán điều phối giao thông **nhường đường và tránh va chạm đa AGV**:

```mermaid
stateDiagram-v2
    [*] --> UNLOCKED : Khởi tạo bản đồ
    
    UNLOCKED --> RESERVED : Router cấp lộ trình cho AGV_01
    RESERVED --> ACTIVE_OCCUPIED : AGV_01 bắt đầu lăn bánh vào đoạn line
    
    ACTIVE_OCCUPIED --> RELEASING : AGV_01 chạm mốc Node tiếp theo
    RELEASING --> UNLOCKED : Giải phóng khóa an toàn cho xe khác đi
    
    ACTIVE_OCCUPIED --> EMERGENCY_LOCKED : Xe bị kẹt / Dừng khẩn cấp trên đoạn line
    EMERGENCY_LOCKED --> UNLOCKED : Giải tỏa sự cố thành công
```

---

## 3. THUẬT TOÁN ĐỊNH TUYẾN TÔ-PÔ (TOPOLOGICAL LINE ROUTING)

Hệ thống tích hợp thuật toán **Dijkstra** trên đồ thị các đoạn line (`line_segments`):

1. **Đầu vào (Input):**
   - Bản đồ hiện tại (`map_id`).
   - Điểm xuất phát (`from_node`) và Điểm đích (`to_node`).
   - Danh sách các đoạn line đang bị khóa bởi xe khác (`is_locked == 1`).
2. **Quy trình xử lý (Process):**
   - Bỏ qua các đoạn line đang bị khóa (tránh xung đột va chạm).
   - Tìm chuỗi các đoạn line có tổng chiều dài $\sum \text{distance\_m}$ là nhỏ nhất.
   - Tính toán góc quay cần thiết tại từng ngã rẽ:
     $$\Delta \theta = \text{heading\_deg}_{\text{next}} - \text{heading\_deg}_{\text{current}}$$
     - $\Delta \theta \approx 0^\circ \rightarrow$ Đi thẳng.
     - $\Delta \theta \approx 90^\circ \rightarrow$ Rẽ phải $90^\circ$.
     - $\Delta \theta \approx -90^\circ \rightarrow$ Rẽ trái $90^\circ$.
     - $\Delta \theta \approx 180^\circ \rightarrow$ Quay đầu $180^\circ$.
3. **Đầu ra (Output):**
   - Danh sách các Node đi qua: `['A', 'B']` hoặc `['A', 'B', 'C']`.
   - Danh sách các đoạn line cần khóa: `['SEG_AB']` hoặc `['SEG_AB', 'SEG_BC']`.
   - Lệnh điều khiển gửi xuống xe: `GO_TO:<Target>` kèm lộ trình chi tiết.
