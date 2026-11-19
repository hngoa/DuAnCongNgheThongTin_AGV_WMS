# ĐẶC TẢ NGHIỆP VỤ LOGIC HỆ THỐNG WMS & ĐIỀU PHỐI AGV_VER2
# (BUSINESS LOGIC SPECIFICATION & OPERATIONAL RULES)
> **Phiên bản:** 2.3 (Map-as-Code + CV Ver3 YOLO11n-MobileNetV3-Geometry + 2-Shot Verification)  
> **Cập nhật:** 2026-11-19 | Đồng bộ đúng với code Server, AI CV Ver3 & Firmware hiện hành

---

## 1. TỔNG QUAN KIẾN TRÚC NGHIỆP VỤ

Hệ thống vận hành theo 6 miền nghiệp vụ cốt lõi:

```
 ┌────────────────────────────────────────────────────────────────────────┐
 │                      WMS CORE BUSINESS ARCHITECTURE                    │
 ├──────────────────────────────────┬─────────────────────────────────────┤
 │ 1. MAP-AS-CODE TOPOLOGICAL GRAPH │ 2. DYNAMIC INBOUND ALLOCATION       │
 │    (Đồng bộ JSON-DB-FW & Hash)   │    (Ma trận quy tắc + Dijkstra TSP) │
 ├──────────────────────────────────┼─────────────────────────────────────┤
 │ 3. MULTI-PRODUCT INVENTORY       │ 4. TRAFFIC LOCK & IOT SECURITY      │
 │    (Tồn kho đa hàng, bảo toàn DB)│    (Khóa cung đường + Token/HMAC)   │
 ├──────────────────────────────────┼─────────────────────────────────────┤
 │ 5. 2-SHOT VISION & SAFETY GATES  │ 6. ENERGY GUARD & WATCHDOG (INA226) │
 └──────────────────────────────────┴─────────────────────────────────────┘
```

---

## 2. CHI TIẾT CÁC MIỀN NGHIỆP VỤ

### 2.1. Miền 1: Quản trị Bản đồ Map-as-Code & Đồng bộ An toàn
* **Quy tắc 1.1 (Single Source of Truth):**
  - Toàn bộ cấu trúc trạm (`map_nodes`), đoạn đường (`line_segments`) và luật mặc định (`default_rules`) được khai báo trong `config/maps/*.json` và kích hoạt qua `config/active_map.json`.
  - Công cụ `tools/sync_map.py` kiểm tra tính hợp lệ trước khi cho phép áp dụng: tối thiểu 2 trạm (tối đa `16` trạm, `64` chặng có hướng trên firmware), có trạm gốc `INBOUND` (`home_node`), `dist_m >= 0.05m` và `>= 0.85 * khoảng cách Euclid`, đảm bảo đi/về thông suốt (BFS 2 chiều) tới mọi trạm.
* **Quy tắc 1.2 (Bảo vệ thay đổi runtime & Bảo toàn tồn kho):**
  - Mặc định `ALLOW_RUNTIME_MAP_CHANGE = False` (trả HTTP `403 MAP_LOCKED_BY_CODE` nếu gọi `/api/wms/map/switch` hoặc `/import` từ Web UI). Chỉ mở qua biến môi trường `AGV_ALLOW_RUNTIME_MAP_CHANGE=true` trong môi trường dev/lab (vẫn chặn `409 Conflict` nếu AGV đang thực thi nhiệm vụ).
  - Cập nhật DB sử dụng **Non-destructive UPSERT** (`MapRepository.upsert_map_config`): giữ nguyên 100% `inventory` và `allocation_rules` của các trạm còn giữ lại; từ chối xóa trạm nếu trạm đó vẫn còn hàng tồn kho (`quantity > 0`).
* **Quy tắc 1.3 (Kiểm tra chéo `MAP_HASH`):**
  - Server và Firmware ESP32 tính chung một mã băm 8 ký tự SHA-256 (`compute_map_hash`). Nếu gói tin Telemetry gửi lên lệch `map_id` hoặc `map_hash`, Server log cảnh báo `[CẢNH BÁO MAP HASH]` và hiển thị badge cảnh báo trên Dashboard.

### 2.2. Miền 2: Phân bổ Kho Thông minh & Tồn kho Đa Mặt hàng
* **Quy tắc 2.1 (Tồn kho Đa sản phẩm theo Trạm):** Một trạm kho có thể chứa nhiều mã hàng khác nhau miễn tổng số lượng không vượt quá `max_capacity`.
* **Quy tắc 2.2 (Ưu tiên Phân bổ Động - `AllocationEngineRepository.allocate_nearest_target_node`):**
  - Xét các trạm đích phù hợp còn chỗ trống theo thứ tự ưu tiên:
    1. Quy tắc đích danh theo `product_code` (`priority` nhỏ hơn ưu tiên trước).
    2. Quy tắc theo `product_category`.
    3. Quy tắc dự phòng `'*'`.
  - Ở cùng mức ưu tiên, hệ thống dùng `TopologicalRouter` (Dijkstra) chọn trạm có tổng quãng đường di chuyển ngắn nhất từ trạm xuất phát.
* **Quy tắc 2.3 (Tối ưu Đa Điểm Dừng - TSP):** Khi khay có nhiều kiện hàng khác loại (Multi-box), `WMSWorkflowManager.plan_and_dispatch_mission_queue` tính hoán vị ghé các trạm có tổng quãng đường đi + quay về `home_node` ngắn nhất.

### 2.3. Miền 3: Điều phối Giao thông & Khóa Cung đường (`line_segments`)
* **Quy tắc 3.1:** Khi phát lệnh `GO_TO:<ToNode>`, Server khóa các đoạn đường trong lộ trình (`is_locked = 1, locked_by_agv = 'AGV_Ver2'`) để router tránh đường bị chiếm dụng.
* **Quy tắc 3.2:** Khóa đường được giải phóng khi hoàn tất dỡ hàng tại trạm, quay về `home_node`, hoàn tất `GO_TO` thủ công, hoặc khi Watchdog/Abort/Firmware phục hồi sự cố.

### 2.4. Miền 4: Bảo mật IoT, Idempotency & Watchdog An toàn
* **Quy tắc 4.1 (Bảo mật Đa tầng - `SecurityService`):**
  - Tách quyền theo Device Token (`X-Device-Token` RBAC cho `AGV`, `CAMERA`, `ADMIN`, cookie session ký HMAC cho Web UI) và ký toàn vẹn dữ liệu lệnh + telemetry bằng `HMAC-SHA256` (`X-Payload-Signature`, `cmd_sig`).
  - Giới hạn kích thước body (`413`), rate-limit (`429`) và chặn Path Traversal (`403`).
* **Quy tắc 4.2 (Idempotent Command & Recovery):**
  - Mọi lệnh gửi xuống ESP32 có `cmd_id` tăng dần. Firmware lưu `lastAckId` để không thực thi trùng lệnh.
  - Nếu gặp sự cố giữa đường (`GAP_TIMEOUT_STOP`, `LOST_LINE_STOP`, `EMERGENCY_STOP`) hoặc ESP32 khởi động lại về `home_node` (`ack_id = 0`), Server tự động giải phóng task treo (`ABORTED`) và khóa đường mồ côi.
* **Quy tắc 4.3 (Safety Watchdog):**
  - Server chuyển AGV sang `OFFLINE` và mở khóa cung đường nếu mất telemetry quá `AGV_WATCHDOG_TIMEOUT_SEC = 6.0s`.
  - Phía ESP32-S3 giám sát mất kết nối HTTP và dừng an toàn phần cứng khi lỗi đứt/mất line vượt ngưỡng bảo vệ.

### 2.5. Miền 5: Quản lý Năng lượng INA226 (`Energy Guard`)
* **Quy tắc 5.1:** Hỗ trợ tự nhận diện dải pin **2S** (`6.4V - 8.4V`) và **3S LiPo/Li-ion** (`9.9V - 12.6V`).
* **Quy tắc 5.2 (Chặn xuất phát khi pin yếu):** Nếu INA226 hoạt động ổn định (`ina_ok != False`) và điện áp rơi xuống vùng thấp (`3.0V < V < 6.8V` đối với 2S, `8.8V < V < 10.5V` đối với 3S, hoặc `ina_ok == True` và `V <= 3.0V`), Server từ chối nhận nhiệm vụ mới (`BATTERY_TOO_LOW`).

### 2.6. Miền 6: Quy trình Dỡ hàng 2-Shot Vision (`Shot 2 Unload Verification`)
* **Quy tắc 6.1 (Xác nhận bằng thị giác AI tại trạm đích):**
  - Khi AGV tới trạm đích (`DELIVERED` $\rightarrow$ `UNLOADING`), hệ thống kích hoạt Camera Android chụp ảnh Shot 2 (`VERIFY_UNLOAD`) thay vì chỉ chờ mù theo thời gian.
  - Nếu AI báo `NEEDS_REVIEW` hoặc phát hiện kiện hàng thuộc trạm đó **vẫn còn trên khay** (`CARGO_STILL_PRESENT`): xe tiếp tục giữ phanh đứng yên và chưa cộng tồn kho.
  - Khi xác nhận kiện hàng đã được dỡ xuống (hoặc khay trống hoàn toàn): Task chuyển `COMPLETED`, bảng `inventory` cộng `+1`, xe chạy tiếp `GO_TO` sang trạm kế trong hàng đợi hoặc tự động quay về trạm gốc `INBOUND` (`home_node`).
