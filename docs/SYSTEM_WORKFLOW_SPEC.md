# TÀI LIỆU ĐẶC TẢ KIẾN TRÚC & LUỒNG HOẠT ĐỘNG TOÀN HỆ THỐNG
# AGV_VER2 - WMS SMART DISPATCHING & DYNAMIC TOPOLOGICAL LINE-MAP
> **Phiên bản:** 2.1 (Dynamic Map & Smart Inventory Allocation)  
> **Cập nhật:** 2026-11-05 | Khắc phục hoàn toàn gán cứng Map & Gán cứng Kho sản phẩm

---

## 1. TỔNG QUAN KIẾN TRÚC HỆ THỐNG (SYSTEM OVERVIEW)

Hệ thống hoạt động theo mô hình **Logistics 4.0 Tập trung & Cấu hình Động**:
- **Bản đồ Line Độc lập:** Dựa trên mạng lưới vạch kẻ line (Topological Line Graph) được nạp động từ Database, có thể thay đổi tùy ý sang các sa bàn/kho xưởng khác mà không cần sửa code.
- **Tồn kho Đa Sản phẩm & Phân bổ Thông minh:** Mỗi trạm kho có thể chứa nhiều loại hàng hóa khác nhau. Kho đích được quyết định động thông qua Bộ quy tắc phân bổ (Allocation Engine) hoặc chỉ định thủ công.

```
 ┌────────────────┐       ┌────────────────┐
 │ 📱 Phone App   │       │ 🚗 AGV_Ver2    │
 │ (Android Cam)  │       │ (ESP32-S3)     │
 └───────┬────────┘       └───────┬────────┘
         │ Upload Ảnh             │ Telemetry & Nhận Lệnh
         ▼                        ▼
 ┌─────────────────────────────────────────┐
 │ 🖥️ WMS BACKEND SERVER (FastAPI / Py)    │
 │ ├─ Dynamic Allocation Engine (Kho đích) │
 │ ├─ Topological Line Routing (Dijkstra)  │
 │ ├─ AI YOLOv11 Product Classifier        │
 │ └─ SQLite DB (Maps, Nodes, Inventory)   │
 └────────────────────┬────────────────────┘
                      │ WebSocket / REST API
                      ▼
 ┌─────────────────────────────────────────┐
 │ 💻 WEB WMS DASHBOARD (Digital Twin)     │
 │ ├─ Vẽ đồ thị mạng lưới Line theo Map    │
 │ ├─ Live Camera Feed & Bounding Box AI   │
 │ ├─ Bảng tồn kho đa mặt hàng theo trạm  │
 └─────────────────────────────────────────┘
```

---

## 2. BẢN ĐỒ TÔ-PÔ ĐƯỜNG LINE ĐỘNG (DYNAMIC TOPOLOGICAL LINE-MAP)

### 2.1. Bản chất Vật lý của Xe AGV Bám Line
Xe AGV di chuyển dựa trên việc bám theo mạng lưới vạch kẻ line cố định trên mặt sàn. Vì vậy:
1. **Điểm nút (`map_nodes`):** Đại diện cho các vị trí đặc biệt trên đường line:
   - `INBOUND`: Cửa tiếp nhận hàng / Điểm chụp ảnh sản phẩm (Ví dụ: Trạm A).
   - `STATION`: Trạm kho / Điểm dừng dỡ hàng (Ví dụ: Kho B, C, D).
   - `INTERSECTION`: Ngã ba, ngã tư giao cắt đường line để xe rẽ hướng.
   - `CHARGING`: Trạm sạc tự động.
2. **Đoạn line kết nối (`line_segments`):**
   - Đoạn vạch kẻ line thực tế nối 2 Node, mang thông tin: Khoảng cách thật dọc theo line ($\text{distance\_m}$), góc la bàn chuẩn ($\text{heading\_deg}$), giới hạn tốc độ và cờ cảnh báo đoạn vạch đứt mờ (`has_gap`).

### 2.2. Instance Bản đồ Mẫu: Sa bàn Lab 1.54m x 1.51m (`SABAN_154_151`)
- **Node A (Inbound):** $(0.00\text{m}, 0.00\text{m})$
- **Node B (Kho 1):** $(0.00\text{m}, 1.51\text{m})$
- **Node C (Kho 2):** $(1.54\text{m}, 1.51\text{m})$
- **Node D (Kho 3):** $(1.54\text{m}, 0.00\text{m})$
- **8 Đoạn line 2 chiều:** `SEG_AB`, `SEG_BC`, `SEG_CD`, `SEG_DA` và chiều ngược lại `SEG_BA`, `SEG_CB`, `SEG_DC`, `SEG_AD`.

---

## 3. CHIẾN LƯỢC QUẢN LÝ TỒN KHO & BỘ QUY TẮC PHÂN BỔ KHO ĐÍCH

### 3.1. Tồn kho Đa mặt hàng theo Trạm (Multi-Product Stock)
- Mỗi trạm kho có giới hạn sức chứa tổng (`max_capacity`, ví dụ: 30 kiện).
- Một trạm có thể chứa hỗn hợp nhiều loại mặt hàng khác nhau:
  $$\text{Kho B} = 6 \text{ kiện Bo Mạch (PKG\_ELEC)} + 2 \text{ kiện Bánh Răng (PKG\_MECH)} = 8/30 \text{ kiện (Còn trống 22)}$$

### 3.2. Bộ quy tắc Phân bổ Kho đích Động (Dynamic Allocation Engine)
Khi AI nhận diện được sản phẩm, hệ thống không gán cứng mà tra cứu bảng `allocation_rules`:
1. **Theo Nhóm ngành hàng (Category Priority):**
   - Hàng Điện tử $\rightarrow$ Ưu tiên Kho B. Nếu Kho B đầy $\rightarrow$ Tự động chuyển kho dự phòng.
   - Hàng Cơ khí $\rightarrow$ Ưu tiên Kho C.
   - Hàng Dụng cụ $\rightarrow$ Ưu tiên Kho D.
2. **Theo Kho trống gần nhất (Nearest Free Station):**
   - Dựa trên mạng lưới line, tính toán khoảng cách ngắn nhất đến các kho còn chỗ trống.
3. **Chỉ định thủ công (Manual Override):**
   - Người vận hành trên Web Dashboard hoặc App có thể chọn kho đích bằng tay theo nhu cầu thực tế.

---

## 4. SƠ ĐỒ TUẦN TỰ HOẠT ĐỘNG (SEQUENCE DIAGRAM)

```
[Nhân viên đặt hàng tại Trạm A] -> [App Android chụp ảnh]
       │
       ▼ (POST /api/camera/upload)
[WMS Server: Nhận ảnh thô]
       │
       ▼
[AI YOLOv11: Phân loại sản phẩm (PKG_ELEC - 96%)]
       │
       ▼
[Allocation Engine: Tra cứu bản đồ & Sức chứa kho]
       ├─ Kiểm tra Kho B: 8/30 (Còn trống 22 chỗ) -> CẤP KHO B!
       │
       ▼
[Tạo Task mới trong DB SQLite (TSK_..._AtoB)]
       │
       ▼
[Khóa đoạn line thực tế SEG_AB trên sa bàn]
       │
       ▼ (Response chu kỳ Telemetry 100ms)
[ESP32 AGV: Nhận lệnh "GO_TO:B"]
       │
       ▼ (Bám line PD + Dead Reckoning)
[ESP32 AGV: Đến Kho B an toàn -> Dừng mềm]
       │
       ▼ (Dwell time 5 giây dỡ hàng)
[Server: Task COMPLETED -> Kho B: PKG_ELEC (+1)]
       │
       ▼
[Mở khóa đoạn line SEG_AB -> WebSocket đẩy lên Dashboard]
```

---

## 5. MÔ HÌNH DỮ LIỆU SQLITE (V2.1)

```sql
-- Bản đồ & Mạng lưới Line Động
maps (map_id, name, width_m, length_m, is_active)
map_nodes (map_id, node_id, name, node_type, x, y, max_capacity, is_occupied)
line_segments (map_id, segment_id, from_node, to_node, distance_m, heading_deg, is_locked)

-- Hàng hóa, Tồn kho Đa mặt hàng & Quy tắc Phân bổ
products (product_code, product_name, yolo_class_id, category)
inventory (map_id, node_id, product_code, quantity)
allocation_rules (rule_id, map_id, product_category, product_code, target_node_id, priority)

-- Quản lý Nhiệm vụ, Đội xe & Telemetry
agv_fleet (agv_id, map_id, status, current_node, x, y, yaw_deg, battery_pct, voltage_v)
dispatch_tasks (task_id, task_code, map_id, product_code, from_node, to_node, route_path, status)
telemetry_logs (log_id, agv_id, map_id, timestamp, x, y, yaw, speed, voltage, current, state)
system_events (event_id, event_type, source, message, created_at)
```
