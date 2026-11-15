# ĐẶC TẢ NGHIỆP VỤ LOGIC HỆ THỐNG WMS & ĐIỀU PHỐI AGV_VER2
# (BUSINESS LOGIC SPECIFICATION & OPERATIONAL RULES)
> **Phiên bản:** 2.1 (Dynamic Map & Smart Inventory Allocation)  
> **Cập nhật:** 2026-11-05 | Khắc phục hoàn toàn gán cứng Map & Sản phẩm

---

## 1. TỔNG QUAN VỀ KIẾN TRÚC MỞ RỘNG (FLEXIBLE ARCHITECTURE)

Hệ thống được thiết kế theo nguyên lý **Độc lập phần cứng & Tùy biến cấu hình (Config-Driven Architecture)** nhằm đáp ứng 2 yêu cầu cốt lõi trong thực tế logistics:

```
 ┌────────────────────────────────────────────────────────────────────────┐
 │                      WMS FLEXIBLE BUSINESS ENGINE                      │
 ├──────────────────────────────────┬─────────────────────────────────────┤
 │ 1. DYNAMIC TOPOLOGICAL LINE-MAP  │ 2. DYNAMIC INBOUND ALLOCATION       │
 │    (Bản đồ mạng lưới Line Động)  │    (Phân bổ vị trí kho thông minh)  │
 ├──────────────────────────────────┼─────────────────────────────────────┤
 │ 3. MULTI-PRODUCT INVENTORY       │ 4. TRAFFIC & ZONE RESERVATION       │
 │    (Tồn kho đa mặt hàng theo trạm│    (Khóa cung đường chống va chạm)  │
 ├──────────────────────────────────┼─────────────────────────────────────┤
 │ 5. SAFETY WATCHDOG & IDEMPOTENCY │ 6. ENERGY MANAGEMENT (INA226)       │
 └──────────────────────────────────┴─────────────────────────────────────┘
```

---

## 2. CHI TIẾT CÁC MIỀN NGHIỆP VỤ NÂNG CẤP

### 2.1. Miền 1: Quản lý Bản đồ Mạng lưới Line Động (Dynamic Topological Line-Map Engine)

* **Bản chất vật lý của xe AGV:**
  - Xe di chuyển hoàn toàn bằng cách **bám theo vạch kẻ line (Line-Following)** trên sàn.
  - Vị trí của các trạm, các góc rẽ, ngã ba, ngã tư và khoảng cách thực tế giữa các đoạn line đã được định hình sẵn khi kẻ vạch trong từng nhà kho / sa bàn cụ thể.
  - Do đó, hệ thống không được gán cứng bản đồ cố định mà phải coi mỗi nhà kho là một **Topological Graph (Đồ thị Tô-pô)**.

* **Quy tắc 1.1 (Cấu trúc Bản đồ Động):**
  - Một bản đồ (`maps`) bao gồm:
    1. **Tập hợp Điểm nút (`map_nodes`):**
       - `STATION`: Trạm kho / Điểm dừng dỡ hàng.
       - `INTERSECTION`: Điểm giao cắt vạch line (ngã 3, ngã 4), xe đi qua để đổi hướng chứ không dỡ hàng.
       - `INBOUND`: Cửa tiếp nhận hàng đầu vào / Điểm chụp ảnh sản phẩm.
       - `CHARGING`: Trạm sạc năng lượng tự động.
       - Mỗi nút có tọa độ đo đạc thực tế $(X, Y)$ và kiểu mỏ neo vạch (vạch ngang chữ T, vạch ngã tư...).
    2. **Tập hợp Đoạn vạch Line (`line_segments`):**
       - Mỗi đoạn line nối giữa 2 Node (`from_node` $\rightarrow$ `to_node`).
       - Chiều dài đo đạc thực tế $\text{distance\_m}$ (mét).
       - Hướng di chuyển: `ONE_WAY` (Một chiều) hoặc `TWO_WAY` (Hai chiều).
       - Góc phương vị chuẩn $\text{heading\_deg}$ để thuật toán Dead Reckoning khóa góc la bàn MPU-6050 chạy thẳng.
       - Cờ `has_gap`: Báo trước đoạn line này có vạch đứt / mờ để xe chủ động bật cầu nối quán tính.

* **Quy tắc 1.2 (Khả năng chuyển đổi bản đồ Zero-Code):**
  - Khi triển khai trên sa bàn hiện tại: Kích hoạt bản đồ `SABAN_154_151` (Gồm 4 trạm $A, B, C, D$ và 8 cung đường).
  - Khi mang xe sang nhà kho khác (ví dụ: kho 8 trạm chữ I, chữ T, hoặc lưới $3 \times 3$): Chỉ cần nạp file cấu hình Map mới vào Database. Toàn bộ thuật toán Dijkstra, tính lộ trình và giao diện Web Dashboard tự động vẽ đúng theo sơ đồ mới mà **không cần sửa một dòng code nào trong Server**!

---

### 2.2. Miền 2: Chiến lược Phân bổ Kho Động & Tồn kho Đa Sản phẩm (Dynamic Storage Allocation)

* **Thực tế kho vận:**
  - Không có một kho nào lại cố định vĩnh viễn "Trạm B chỉ được chứa Bo mạch". Trong thực tế, một trạm kho có thể chứa nhiều loại hàng khác nhau tùy vào sức chứa còn lại và chiến lược sắp xếp hàng.

* **Quy tắc 2.1 (Tồn kho Đa sản phẩm theo Trạm - Multi-Product Stocking):**
  - Mỗi trạm kho có sức chứa tổng thể $\text{TotalCapacity}$ (tính theo số kiện).
  - Tồn kho được quản lý chi tiết theo bảng `inventory`:
    $$\text{Trạm B} = \{\text{PKG\_ELEC}: 5 \text{ kiện}, \text{PKG\_MECH}: 2 \text{ kiện}\} \rightarrow \text{Tổng}: 7/30 \text{ kiện}$$

* **Quy tắc 2.2 (Bộ quy tắc Phân bổ Kho Đích Thông minh - Inbound Allocation Engine):**
  Khi AI phân tích ảnh và nhận diện được mã sản phẩm $\text{ProductCode}$, hệ thống WMS sẽ xác định kho đích thông qua 4 chiến lược có thể cấu hình linh hoạt:
  
  1. **Chiến lược Cấu hình Quy tắc (Configurable Rule Matrix):**
     - Cho phép quản lý kho cấu hình: *"Sản phẩm loại A ưu tiên đưa vào Kho B. Nếu Kho B còn dưới 2 chỗ trống $\rightarrow$ Chuyển sang Kho C"*.
  2. **Chiến lược Tìm Kho trống Gần nhất (Nearest Available Station):**
     - Dựa trên mạng lưới line, hệ thống tính toán khoảng cách từ Trạm nạp hàng đến tất cả các trạm kho còn chỗ trống $\rightarrow$ Chọn trạm có lộ trình ngắn nhất để tiết kiệm pin và thời gian của xe.
  3. **Chiến lược Gom cụm hàng (Consolidation / Clustering):**
     - Ưu tiên xếp sản phẩm vào trạm đang có sẵn mặt hàng cùng loại để thuận tiện cho việc gom đơn xuất kho.
  4. **Can thiệp Thủ công (Manual Override):**
     - Cho phép người vận hành trên Web Dashboard hoặc App điện thoại chọn: *"Tự động điều phối theo AI"* HOẶC *"Chỉ định giao đến Trạm X"*.

---

### 2.3. Miền 3: Nghiệp vụ Điều phối Giao thông & Khóa Cung đường (Traffic & Segment Locking)

* **Quy tắc 3.1 (Khóa đoạn đường động - Segment Reservation):**
  - Khi một nhiệm vụ được gán cho AGV:
    - Thuật toán Dijkstra tìm chuỗi các đoạn line ngắn nhất: $Seg_1 \rightarrow Seg_2 \rightarrow ... \rightarrow Seg_k$.
    - Hệ thống khóa trạng thái `is_locked = TRUE` trên các đoạn line đó (và chiều ngược lại nếu là đường 2 chiều).
  - Khi có AGV thứ 2 hoạt động: Nếu lộ trình cắt ngang đoạn line đang bị khóa $\rightarrow$ AGV thứ 2 phải dừng chờ tại điểm nút liền trước cho đến khi cung đường được giải phóng.

* **Quy tắc 3.2 (Giải phóng khóa theo từng chặng - Dynamic Segment Release):**
  - Khi AGV đã đi qua xong đoạn line $Seg_1$ và chạm mốc Node tiếp theo $\rightarrow$ Hệ thống lập tức mở khóa $Seg_1$ để các xe khác có thể lưu thông, tối ưu hóa năng lực thông hành của sa bàn.

---

### 2.4. Miền 4: Nghiệp vụ Truyền thông, ACK & Watchdog An Toàn

* **Quy tắc 4.1 (Idempotent Command & ACK):**
  - Mọi lệnh gửi xuống xe đều có mã `cmd_id` duy nhất tăng dần.
  - ESP32 lưu `last_ack_id`. Nếu nhận lại cùng một `cmd_id` cũ thì chỉ gửi lại ACK xác nhận chứ không thực thi lặp lại để tránh xe quay 2 lần hoặc chạy quá trạm.
* **Quy tắc 4.2 (Safety Watchdog):**
  - Server mất liên lạc với xe $> 2.5\text{s} \rightarrow$ Đổi trạng thái xe sang `OFFLINE`.
  - ESP32 mất phản hồi từ Server $> 3.0\text{s} \rightarrow$ Firmware tự ngắt động cơ khẩn cấp (Hardware Failsafe Stop).

---

### 2.5. Miền 5: Nghiệp vụ Quản lý Năng lượng INA226 (Energy Guard)

* **Quy tắc 5.1 (Phân cấp điện áp Pin 3S LiPo):**
  - $\ge 11.2\text{V}$ (SOC $> 50\%$): 🟢 **Bình thường** - Tiếp nhận mọi nhiệm vụ.
  - $10.5\text{V} \le V < 11.2\text{V}$ (SOC $20\% - 50\%$): 🟡 **Cảnh báo Pin vừa** - Hoàn thành nốt đơn hàng hiện tại, cảnh báo vàng trên Web.
  - $< 10.5\text{V}$ (SOC $< 20\%$): 🔴 **Nguy cấp** - Từ chối nhận đơn mới, tự động lập lộ trình đưa xe về **Trạm sạc (Charging Station)**.
  - $< 9.9\text{V}$ (SOC $0\%$): ⛔ **Cạn kiệt** - Cắt toàn bộ tải động cơ để bảo vệ pin không bị chai phồng.

---

### 2.6. Miền 6: Nghiệp vụ Hoàn tất Nhiệm vụ & Dỡ hàng (Task Completion)

* **Quy tắc 6.1 (Dwell Time dỡ hàng):**
  - Khi xe đến trạm đích, xe chuyển sang trạng thái `UNLOADING` trong $5.0\text{giây}$ để giả lập dỡ hàng.
* **Quy tắc 6.2 (Cập nhật Tồn kho):**
  - Sau thời gian dỡ hàng:
    - Bảng `inventory` tại trạm đích cập nhật tăng số lượng của loại sản phẩm đó $+1$.
    - Task chuyển sang `COMPLETED`.
    - Thông báo cập nhật tức thời qua WebSocket lên Web Dashboard.
* **Quy tắc 6.3 (Tự động hồi vị - Auto Return Home):**
  - Nếu cấu hình bật `AUTO_RETURN_HOME`: Sau $3\text{giây}$, xe tự động nhận lệnh quay về trạm xuất phát hoặc trạm sạc để sẵn sàng cho chu kỳ tiếp theo.
