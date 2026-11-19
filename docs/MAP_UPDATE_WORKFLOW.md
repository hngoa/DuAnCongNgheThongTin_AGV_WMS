# Quy Trình Quản Trị & Cập Nhật Bản Đồ AGV_Ver2 (Map-as-Code)

## 1. Nguyên Tắc Kiến Trúc
Để **mọi thay đổi bản đồ** (thêm/bớt trạm, thay đổi chiều dài `m`, đổi hình dạng chữ L/T/sa bàn 6+ trạm, đổi tên trạm gốc `INBOUND`) **không bao giờ phá vỡ các cơ chế di chuyển cốt lõi** trên Firmware và Server:

1. **Single Source of Truth (`config/active_map.json`)**:
   - Cấu hình bản đồ chuẩn được quản lý dưới dạng file JSON trong `config/maps/*.json` và file đang kích hoạt `config/active_map.json` (quản lý quyền qua Git).
   - Trực tiếp đổi/nạp bản đồ từ Web UI được **khóa mặc định** (`ALLOW_RUNTIME_MAP_CHANGE = False` / HTTP `403 MAP_LOCKED_BY_CODE`) nhằm tránh lệch dữ liệu giữa Server và Firmware.
2. **Bảo toàn 100% Lõi Điều Khiển & An Toàn (`test_10_line_centric_hybrid_recovery.cpp`)**:
   - Các thuật toán **EKF Pose Fusion**, **PID bám line 5-IR**, **Cascade Wheel Control**, **Dead-Reckoning Bridge**, **Corner Sensor Anchor**, **Bend Catch**, **Corner Braking**, **ZUPT Gyro** và **HMAC-SHA256** hoạt động hoàn toàn độc lập với danh sách trạm cụ thể.
   - Chúng chỉ tiêu thụ `DynamicLeg` (`distM`, `headingDeg`, `targetX`, `targetY`) do `generated_map_config.h` (offline) hoặc `route_legs` Dijkstra từ Server cung cấp.
3. **Đồng Bộ An Toàn 2 Chiều Bằng `MAP_HASH` (8 ký tự SHA-256)**:
   - Server (`MapRepository.compute_map_hash`), CLI (`tools/sync_map.py`) và Firmware (`include/generated_map_config.h` -> `ACTIVE_MAP_HASH`) tính chung một mã băm chuẩn hóa.
   - Mỗi gói tin Telemetry của ESP32 đều gửi kèm `map_id` và `map_hash`. Nếu lệch bản đồ, Server ghi log `[CẢNH BÁO MAP HASH]` và Dashboard bật pill đỏ cảnh báo ngay lập tức.

---

## 2. Quy Trình 4 Bước Cập Nhật Bản Đồ (Dành Cho Lập Trình Viên Được Phân Quyền)

1. **Chỉnh sửa hoặc tạo file bản đồ** trong `config/maps/<ten_ban_do>.json`:
   - Bắt buộc có `map_id`, `name`, `width_m > 0.2`, `length_m > 0.2`.
   - Có ít nhất 2 trạm (`<= 16` trạm), trong đó ít nhất 1 trạm `"type": "INBOUND"` (hoặc khai báo `"home_node"`).
   - Các đoạn `segments` (`<= 64` chặng có hướng) phải có `dist_m >= 0.05` (và `>= 0.85 * euclid`), đảm bảo đi được từ `home_node` tới mọi trạm và quay lại được `home_node` (đồ thị liên thông 2 chiều).
2. **Kiểm tra, sinh Header Firmware & đồng bộ SQLite không mất tồn kho**:
   ```powershell
   python tools/sync_map.py --map config/maps/<ten_ban_do>.json --apply-db
   python tools/sync_map.py --check
   ```
   - Lệnh này sẽ từ chối ngay nếu thiếu đường về trạm gốc, sai khoảng cách hình học hoặc vượt giới hạn trạm/chặng.
   - Khi cập nhật SQLite (`--apply-db`), Server thực hiện **UPSERT không phá hủy**: giữ nguyên tồn kho (`inventory`) và quy tắc phân bổ (`allocation_rules`) của các trạm còn giữ lại, và chặn xóa trạm nếu vẫn còn hàng tồn kho `> 0`.
3. **Kiểm thử hồi quy & Biên dịch lại Firmware ESP32-S3**:
   ```powershell
   python server/tests/run_all_tests.py
   pio run -e test_10_line_centric_hybrid_recovery
   ```
4. **Commit Git, nạp Firmware vào ESP32-S3 và khởi động lại Server (`python server/run_server.py`)**.

> **Lưu ý Developer Mode**: Chỉ bật biến môi trường `AGV_ALLOW_RUNTIME_MAP_CHANGE=true` khi cần thử nghiệm tạm thời đổi map trực tiếp qua API/Web UI ở môi trường lab (khi bật vẫn giữ cổng chặn `409 Conflict` nếu AGV đang thực hiện nhiệm vụ).
