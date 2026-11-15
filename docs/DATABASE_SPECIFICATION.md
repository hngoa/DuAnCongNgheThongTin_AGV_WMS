# ĐẶC TẢ CHI TIẾT CƠ SỞ DỮ LIỆU & SƠ ĐỒ QUAN HỆ (ERD)
# AGV_VER2 WMS SYSTEM (PHIÊN BẢN V2.1)
> **Căn cứ kỹ thuật:** Quản lý Đa Bản đồ Mạng lưới Line Động, Tồn kho Đa mặt hàng và Điều phối Đội xe thông minh.  
> **Cập nhật:** 2026-11-05

---

## 1. SƠ ĐỒ QUAN HỆ THỰC THỂ (ENTITY RELATIONSHIP DIAGRAM - ERD)

```mermaid
erDiagram
    MAPS ||--o{ MAP_NODES : "chứa (1:N)"
    MAPS ||--o{ LINE_SEGMENTS : "định nghĩa (1:N)"
    MAPS ||--o{ ALLOCATION_RULES : "áp dụng (1:N)"
    MAPS ||--o{ AGV_FLEET : "hoạt động trên (1:N)"
    MAPS ||--o{ DISPATCH_TASKS : "thực thi trên (1:N)"

    MAP_NODES ||--o{ LINE_SEGMENTS : "điểm đầu (1:N)"
    MAP_NODES ||--o{ LINE_SEGMENTS : "điểm cuối (1:N)"
    MAP_NODES ||--o{ INVENTORY : "lưu trữ (1:N)"
    MAP_NODES ||--o{ ALLOCATION_RULES : "trỏ tới kho đích (1:N)"
    MAP_NODES ||--o{ DISPATCH_TASKS : "trạm xuất phát (1:N)"
    MAP_NODES ||--o{ DISPATCH_TASKS : "trạm đích (1:N)"

    PRODUCTS ||--o{ INVENTORY : "mặt hàng tồn (1:N)"
    PRODUCTS ||--o{ ALLOCATION_RULES : "quy tắc phân bổ (1:N)"
    PRODUCTS ||--o{ DISPATCH_TASKS : "hàng vận chuyển (1:N)"

    AGV_FLEET ||--o{ DISPATCH_TASKS : "được giao xe (1:N)"
    AGV_FLEET ||--o{ TELEMETRY_LOGS : "phát sinh (1:N)"
    DISPATCH_TASKS ||--o{ TELEMETRY_LOGS : "theo vết (1:N)"

    MAPS {
        string map_id PK "Mã bản đồ sa bàn / nhà kho"
        string name "Tên bản đồ hiển thị"
        float width_m "Chiều rộng sa bàn (m)"
        float length_m "Chiều dài sa bàn (m)"
        int is_active "Cờ kích hoạt (1: Active)"
        datetime created_at "Thời gian tạo"
    }

    MAP_NODES {
        string map_id PK,FK "Thuộc bản đồ nào"
        string node_id PK "Mã nút (A, B, C, D...)"
        string name "Tên nút hiển thị"
        string node_type "Loại nút (INBOUND/STATION/INT)"
        float x "Tọa độ X đo trên sàn (m)"
        float y "Tọa độ Y đo trên sàn (m)"
        string color "Mã màu hiển thị UI"
        int max_capacity "Sức chứa tối đa (kiện)"
        int is_occupied "Trạng thái có xe đỗ"
        string occupied_by_agv "Mã xe đang đỗ"
    }

    LINE_SEGMENTS {
        string map_id PK,FK "Thuộc bản đồ nào"
        string segment_id PK "Mã đoạn line (SEG_AB...)"
        string from_node FK "Nút bắt đầu"
        string to_node FK "Nút kết thúc"
        float distance_m "Chiều dài đoạn line (m)"
        string direction "Hướng đi (ONE_WAY/TWO_WAY)"
        float heading_deg "Góc la bàn chuẩn (độ)"
        float speed_limit_cm_s "Tốc độ tối đa (cm/s)"
        int has_gap "Cảnh báo vạch đứt / mờ"
        int is_locked "Cờ khóa cung đường"
        string locked_by_agv "Xe đang giữ khóa"
    }

    PRODUCTS {
        string product_code PK "Mã sản phẩm (PKG_ELEC...)"
        string product_name "Tên sản phẩm hiển thị"
        int yolo_class_id UK "Class ID model YOLO (0, 1, 2)"
        string category "Nhóm ngành hàng"
        string description "Mô tả tính chất hàng"
        datetime created_at "Thời gian tạo"
    }

    INVENTORY {
        int inventory_id PK "Khóa chính tự tăng"
        string map_id FK "Thuộc bản đồ nào"
        string node_id FK "Thuộc trạm kho nào"
        string product_code FK "Mã mặt hàng"
        int quantity "Số lượng tồn kho thực tế"
        datetime updated_at "Lần cập nhật cuối"
    }

    ALLOCATION_RULES {
        int rule_id PK "Khóa chính tự tăng"
        string map_id FK "Thuộc bản đồ nào"
        string product_category "Nhóm hàng áp dụng"
        string product_code "Mặt hàng cụ thể"
        string target_node_id FK "Trạm kho đích ưu tiên"
        int priority "Độ ưu tiên (1: Cao nhất)"
        int is_active "Cờ kích hoạt quy tắc"
    }

    AGV_FLEET {
        string agv_id PK "Mã định danh xe (AGV_Ver2)"
        string map_id FK "Bản đồ đang hoạt động"
        string status "Trạng thái (IDLE/RUNNING...)"
        string current_node "Nút hiện tại đang đỗ"
        string target_node "Nút đích đang hướng tới"
        int current_task_id "ID nhiệm vụ đang chạy"
        float x "Tọa độ X thời gian thực"
        float y "Tọa độ Y thời gian thực"
        float yaw_deg "Góc hướng Yaw thời gian thực"
        float voltage_v "Điện áp pin INA226 (V)"
        float current_a "Dòng tải tiêu thụ (A)"
        float battery_pct "% Dung lượng pin LiPo (SOC)"
        int last_ack_id "ACK ID xác nhận từ xe"
        datetime last_heartbeat "Thời điểm nhận tin cuối"
    }

    DISPATCH_TASKS {
        int task_id PK "Khóa chính tự tăng"
        string task_code UK "Mã nhiệm vụ duy nhất"
        string map_id FK "Thuộc bản đồ nào"
        string product_code FK "Mã sản phẩm vận chuyển"
        string from_node FK "Trạm nạp hàng đầu vào"
        string to_node FK "Trạm dỡ hàng đích"
        string route_path "Lộ trình line (A->B...)"
        string assigned_agv FK "Xe được giao việc"
        float ai_confidence "Độ tin cậy AI YOLO (0-1)"
        string image_path "Đường dẫn ảnh chụp phone"
        string status "Trạng thái (PENDING/RUNNING...)"
        float dwell_time_sec "Thời gian đỗ dỡ hàng (s)"
        datetime started_at "Thời điểm bắt đầu chạy"
        datetime completed_at "Thời điểm hoàn tất"
    }

    TELEMETRY_LOGS {
        int log_id PK "Khóa chính tự tăng"
        string agv_id FK "Mã xe phát sinh log"
        string map_id "Bản đồ tương ứng"
        datetime timestamp "Thời điểm ghi nhận"
        float x "Tọa độ X (m)"
        float y "Tọa độ Y (m)"
        float yaw "Góc quay Yaw (độ)"
        float dist "Quãng đường lũy kế (m)"
        float speed "Vận tốc tức thời (cm/s)"
        int line_mask "Trạng thái 5 mắt line"
        float voltage "Điện áp Bus INA226 (V)"
        float current "Dòng điện tải (A)"
        float power "Công suất tải (W)"
        float energy_mah "Dung lượng tiêu thụ (mAh)"
        string state "Trạng thái firmware xe"
        int task_id FK "Nhiệm vụ đang thực hiện"
    }

    SYSTEM_EVENTS {
        int event_id PK "Khóa chính tự tăng"
        string event_type "Loại (INFO/WARN/ERROR)"
        string source "Nguồn phát sinh"
        string message "Thông điệp tóm tắt"
        string details "Chi tiết kỹ thuật"
        datetime created_at "Thời điểm tạo"
    }
```

---

## 2. TỪ ĐIỂN DỮ LIỆU CHI TIẾT (DATA DICTIONARY)

### 2.1. Nhóm 1: Quản lý Bản đồ & Mạng lưới Line Động

#### Bảng `maps` (Quản lý các bản đồ sa bàn / nhà xưởng)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `map_id` | `TEXT` | **PK** | - | Định danh duy nhất bản đồ (VD: `'SABAN_154_151'`, `'KHO_A'`). |
| `name` | `TEXT` | `NOT NULL` | - | Tên hiển thị trực quan trên Web Dashboard. |
| `width_m` | `REAL` | `NOT NULL` | - | Chiều rộng sa bàn thực tế (m). |
| `length_m` | `REAL` | `NOT NULL` | - | Chiều dài sa bàn thực tế (m). |
| `description` | `TEXT` | `NULL` | - | Mô tả đặc điểm sa bàn hoặc nhà xưởng. |
| `is_active` | `INTEGER` | `NOT NULL` | `1` | `1`: Bản đồ đang hoạt động, `0`: Bản đồ lưu trữ. |
| `created_at` | `TEXT` | `DEFAULT` | `now` | Thời điểm nạp cấu hình bản đồ. |

#### Bảng `map_nodes` (Các điểm nút trên đường line)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `map_id` | `TEXT` | **PK, FK** $\rightarrow$ `maps.map_id` | - | Thuộc bản đồ nào (Cascade Delete). |
| `node_id` | `TEXT` | **PK** | - | Mã nút trên đường line (VD: `'A'`, `'B'`, `'C'`, `'D'`, `'CHG_01'`). |
| `name` | `TEXT` | `NOT NULL` | - | Tên nút (VD: `'Trạm Nhập A'`, `'Kho Kệ 1 B'`). |
| `node_type` | `TEXT` | `NOT NULL` | `'STATION'` | Phân loại nút: `'INBOUND'`, `'STATION'`, `'INTERSECTION'`, `'CHARGING'`. |
| `x` | `REAL` | `NOT NULL` | - | Tọa độ X thực tế đo đạc dọc theo sa bàn (m). |
| `y` | `REAL` | `NOT NULL` | - | Tọa độ Y thực tế đo đạc dọc theo sa bàn (m). |
| `color` | `TEXT` | `NULL` | `'#38bdf8'` | Màu hiển thị điểm nút trên Canvas 2D Web. |
| `max_capacity` | `INTEGER` | `NOT NULL` | `30` | Sức chứa tối đa (kiện) nếu nút này là kho dỡ hàng. |
| `is_occupied` | `INTEGER` | `NOT NULL` | `0` | `1`: Đang có xe đỗ tại điểm này, `0`: Trống. |
| `occupied_by_agv`| `TEXT` | `NULL` | `NULL` | Mã định danh xe đang đỗ tại nút (VD: `'AGV_Ver2'`). |

#### Bảng `line_segments` (Các đoạn vạch line kết nối thực tế - Zone Locking)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `map_id` | `TEXT` | **PK, FK** $\rightarrow$ `maps.map_id` | - | Thuộc bản đồ nào. |
| `segment_id` | `TEXT` | **PK** | - | Mã đoạn line (VD: `'SEG_AB'`, `'SEG_BC'`). |
| `from_node` | `TEXT` | **FK** $\rightarrow$ `map_nodes.node_id` | - | Điểm nút bắt đầu của đoạn line. |
| `to_node` | `TEXT` | **FK** $\rightarrow$ `map_nodes.node_id` | - | Điểm nút kết thúc của đoạn line. |
| `distance_m` | `REAL` | `NOT NULL` | - | Khoảng cách thật đo đạc dọc theo vạch kẻ line (m). |
| `direction` | `TEXT` | `NOT NULL` | `'TWO_WAY'` | `'ONE_WAY'` (Một chiều) hoặc `'TWO_WAY'` (Hai chiều). |
| `heading_deg` | `REAL` | `NOT NULL` | - | Góc la bàn chuẩn của đoạn line để MPU-6050 khóa hướng. |
| `speed_limit_cm_s`| `REAL` | `DEFAULT` | `25.0` | Tốc độ trần cho phép trên đoạn line (cm/s). |
| `has_gap` | `INTEGER` | `NOT NULL` | `0` | `1`: Báo hiệu đoạn này có đứt/mờ vạch cần Dead Reckoning. |
| `is_locked` | `INTEGER` | `NOT NULL` | `0` | `1`: Đang bị khóa bởi xe đang chạy (Chống va chạm). |
| `locked_by_agv` | `TEXT` | `NULL` | `NULL` | Mã xe đang giữ quyền ưu tiên đi trên đoạn line này. |

---

### 2.2. Nhóm 2: Quản lý Hàng hóa, Tồn kho Đa mặt hàng & Quy tắc Phân bổ

#### Bảng `products` (Danh mục mặt hàng & Ánh xạ nhãn AI)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `product_code` | `TEXT` | **PK** | - | Mã sản phẩm (VD: `'PKG_ELEC'`, `'PKG_MECH'`). |
| `product_name` | `TEXT` | `NOT NULL` | - | Tên mặt hàng hiển thị (VD: `'Bo Mạch Vi Điều Khiển'`). |
| `yolo_class_id` | `INTEGER` | **UNIQUE, NOT NULL** | - | Class ID tương ứng trong mô hình YOLOv11 (0, 1, 2...). |
| `category` | `TEXT` | `NOT NULL` | - | Nhóm ngành hàng (`'ELECTRONIC'`, `'MECHANICAL'`, `'TOOL'`). |
| `description` | `TEXT` | `NULL` | - | Ghi chú đặc tính lưu kho (VD: hàng dễ vỡ, nhạy cảm nhiệt). |

#### Bảng `inventory` (Tồn kho chi tiết từng mặt hàng tại mỗi trạm)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `inventory_id` | `INTEGER` | **PK** | `AUTOINCREMENT` | Khóa chính tự tăng. |
| `map_id` | `TEXT` | **FK** $\rightarrow$ `maps.map_id` | - | Bản đồ tương ứng. |
| `node_id` | `TEXT` | **FK** $\rightarrow$ `map_nodes.node_id` | - | Trạm kho lưu trữ. |
| `product_code` | `TEXT` | **FK** $\rightarrow$ `products.product_code`| - | Loại mặt hàng lưu trữ. |
| `quantity` | `INTEGER` | `NOT NULL` | `0` | Số lượng kiện hàng hiện tại (Không được âm). |
| `updated_at` | `TEXT` | `DEFAULT` | `now` | Thời điểm biến động kho gần nhất. |
| **Ràng buộc:** | `UNIQUE(map_id, node_id, product_code)` | Đảm bảo mỗi mặt hàng chỉ có 1 bản ghi số lượng tại 1 trạm. |

#### Bảng `allocation_rules` (Bộ quy tắc Phân bổ Kho đích Động)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `rule_id` | `INTEGER` | **PK** | `AUTOINCREMENT` | Mã quy tắc tự tăng. |
| `map_id` | `TEXT` | **FK** $\rightarrow$ `maps.map_id` | - | Áp dụng cho bản đồ nào. |
| `product_category`| `TEXT` | `NULL` | - | Áp dụng theo nhóm ngành (VD: `'ELECTRONIC'`, hoặc `'*'` cho tất cả). |
| `product_code` | `TEXT` | `NULL` | - | Hoặc chỉ định đích danh mặt hàng cụ thể. |
| `target_node_id` | `TEXT` | **FK** $\rightarrow$ `map_nodes.node_id` | - | Trạm kho đích ưu tiên tiếp nhận. |
| `priority` | `INTEGER` | `NOT NULL` | `1` | Mức ưu tiên (1 là cao nhất; nếu trạm này đầy, xét priority 2). |
| `is_active` | `INTEGER` | `NOT NULL` | `1` | `1`: Quy tắc đang hiệu lực. |

---

### 2.3. Nhóm 3: Quản lý Đội xe, Nhiệm vụ & Telemetry

#### Bảng `agv_fleet` (Trạng thái xe & Quản lý năng lượng INA226)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `agv_id` | `TEXT` | **PK** | - | Mã định danh xe (VD: `'AGV_Ver2'`). |
| `map_id` | `TEXT` | **FK** $\rightarrow$ `maps.map_id` | - | Bản đồ xe đang hoạt động. |
| `status` | `TEXT` | `NOT NULL` | `'IDLE'` | Trạng thái: `'IDLE'`, `'RUNNING'`, `'UNLOADING'`, `'CHARGING'`, `'OFFLINE'`. |
| `current_node` | `TEXT` | `NULL` | - | Nút sa bàn xe đang đỗ. |
| `target_node` | `TEXT` | `NULL` | - | Nút đích xe đang di chuyển tới. |
| `current_task_id`| `INTEGER` | `NULL` | `NULL` | ID nhiệm vụ hiện tại đang thực thi. |
| `x`, `y` | `REAL` | `NOT NULL` | `0.0` | Tọa độ tâm trục bánh xe thời gian thực (m). |
| `yaw_deg` | `REAL` | `NOT NULL` | `90.0` | Góc hướng la bàn IMU MPU-6050 (độ). |
| `speed_cm_s` | `REAL` | `DEFAULT` | `0.0` | Tốc độ tức thời tính từ Encoder (cm/s). |
| `voltage_v` | `REAL` | `DEFAULT` | `12.6` | Điện áp Bus đọc từ cảm biến INA226 (V). |
| `current_a` | `REAL` | `DEFAULT` | `0.0` | Dòng tải tức thời từ INA226 (A). |
| `battery_pct` | `REAL` | `DEFAULT` | `100.0` | % Dung lượng pin LiPo tính toán (SOC). |
| `last_ack_id` | `INTEGER` | `DEFAULT` | `0` | Mã ACK ID mới nhất nhận từ firmware xe. |
| `last_heartbeat`| `TEXT` | `DEFAULT` | `now` | Thời điểm nhận gói tin Telemetry gần nhất (phục vụ Watchdog). |

#### Bảng `dispatch_tasks` (Nhiệm vụ điều phối vận chuyển)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `task_id` | `INTEGER` | **PK** | `AUTOINCREMENT` | ID nhiệm vụ tự tăng. |
| `task_code` | `TEXT` | **UNIQUE, NOT NULL** | - | Mã định danh nhiệm vụ (VD: `'TSK_20261105_AtoB'`). |
| `map_id` | `TEXT` | **FK** $\rightarrow$ `maps.map_id` | - | Bản đồ diễn ra nhiệm vụ. |
| `product_code` | `TEXT` | **FK** $\rightarrow$ `products.product_code`| - | Mặt hàng cần giao. |
| `from_node` | `TEXT` | **FK** $\rightarrow$ `map_nodes.node_id` | - | Nút xuất phát (thường là Trạm Nhập A). |
| `to_node` | `TEXT` | **FK** $\rightarrow$ `map_nodes.node_id` | - | Nút kho đích do Allocation Engine cấp. |
| `route_path` | `TEXT` | `NOT NULL` | - | Chuỗi lộ trình vạch line (VD: `'A->B'` hoặc `'A->B->C'`). |
| `assigned_agv` | `TEXT` | **FK** $\rightarrow$ `agv_fleet.agv_id` | `'AGV_Ver2'` | Xe được phân công thực hiện. |
| `ai_confidence` | `REAL` | `DEFAULT` | `0.0` | Độ tin cậy nhận diện từ mô hình YOLO (0.00 - 1.00). |
| `image_path` | `TEXT` | `NULL` | - | Đường dẫn ảnh chụp từ App Android. |
| `status` | `TEXT` | `NOT NULL` | `'PENDING'` | Trạng thái: `'PENDING'`, `'ASSIGNED'`, `'IN_TRANSIT'`, `'DELIVERED'`, `'COMPLETED'`, `'REJECTED'`. |
| `dwell_time_sec`| `REAL` | `DEFAULT` | `5.0` | Thời gian đỗ dỡ hàng tại kho đích (giây). |
| `started_at` | `TEXT` | `NULL` | - | Thời điểm xe bắt đầu lăn bánh. |
| `completed_at` | `TEXT` | `NULL` | - | Thời điểm dỡ hàng xong và tăng tồn kho. |

#### Bảng `telemetry_logs` (Nhật ký Data Logger Telemetry)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `log_id` | `INTEGER` | **PK** | `AUTOINCREMENT` | Khóa chính tự tăng. |
| `agv_id` | `TEXT` | **FK** $\rightarrow$ `agv_fleet.agv_id` | - | Xe phát sinh bản ghi telemetry. |
| `map_id` | `TEXT` | `NULL` | - | Bản đồ tại thời điểm ghi log. |
| `timestamp` | `TEXT` | `DEFAULT` | `now` | Mốc thời gian ghi nhận chi tiết đến phần nghìn giây. |
| `x`, `y`, `yaw` | `REAL` | `NULL` | - | Tọa độ và hướng di chuyển của xe. |
| `dist`, `speed` | `REAL` | `NULL` | - | Quãng đường tích lũy (m) và vận tốc (cm/s). |
| `line_mask` | `INTEGER` | `NULL` | - | Trạng thái bit của 5 mắt cảm biến hồng ngoại TCRT5000. |
| `voltage` | `REAL` | `NULL` | - | Điện áp Bus INA226 (V). |
| `current` | `REAL` | `NULL` | - | Dòng tải tức thời INA226 (A). |
| `power` | `REAL` | `NULL` | - | Công suất tiêu thụ tức thời INA226 (W). |
| `energy_mah` | `REAL` | `NULL` | - | Tích phân dung lượng tiêu thụ tích lũy (mAh). |
| `state` | `TEXT` | `NULL` | - | Trạng thái FSM của vi điều khiển ESP32-S3. |
| `task_id` | `INTEGER` | **FK** $\rightarrow$ `dispatch_tasks.task_id`| `NULL` | Thuộc nhiệm vụ vận chuyển nào. |

#### Bảng `system_events` (Nhật ký Sự kiện Hệ thống)
| Tên Cột | Kiểu dữ liệu | Ràng buộc | Mặc định | Ý nghĩa nghiệp vụ |
| :--- | :--- | :---: | :---: | :--- |
| `event_id` | `INTEGER` | **PK** | `AUTOINCREMENT` | ID sự kiện tự tăng. |
| `event_type` | `TEXT` | `NOT NULL` | - | `'INFO'`, `'WARNING'`, `'ERROR'`, `'SAFETY'`. |
| `source` | `TEXT` | `NOT NULL` | - | `'AI_ENGINE'`, `'WMS_ALLOCATOR'`, `'AGV_FIRMWARE'`, `'POWER_GUARD'`. |
| `message` | `TEXT` | `NOT NULL` | - | Tóm tắt sự kiện. |
| `details` | `TEXT` | `NULL` | - | Dữ liệu kỹ thuật chi tiết dạng JSON hoặc chuỗi. |
| `created_at` | `TEXT` | `DEFAULT` | `now` | Thời điểm phát sinh sự kiện. |
