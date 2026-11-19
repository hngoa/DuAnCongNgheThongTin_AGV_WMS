# KẾ HOẠCH & QUY TRÌNH KIỂM THỬ TỪNG BƯỚC (STEP-BY-STEP TESTING) - AGV_VER2

Thư mục `test/` chứa các bài kiểm thử phần cứng theo từng bước (`test_00`..`test_09`) và firmware vận hành chính thức hiện tại (`test_10_line_centric_hybrid_recovery.cpp`, được đặt làm `default_envs` trong `platformio.ini`).

---

## 📌 DANH SÁCH CÁC MÔI TRƯỜNG FIRMWARE

| Bước | Tên File Test | Mục tiêu & Chức năng kiểm thử | Trạng thái |
| :---: | :--- | :--- | :---: |
| **00** | `test_00_basic_motor_control.cpp` | **Kiểm thử động cơ căn bản:** Tiến, Lùi, Xoay Trái/Phải, kiểm tra từng bánh, Watchdog dừng 3s. | 🟡 *Test thành phần* |
| **01** | `test_01_ir_calibration.cpp` | **Hiệu chỉnh cảm biến dò line TCRT5000 (5 mắt):** Giải phóng GPIO 19/20, đảo bẹ dây, đọc sai số lệch tâm. | 🟡 *Calib IR* |
| **02** | `test_02_line_follow_pid.cpp` | **Bám line PD cơ bản:** PWM 20kHz, giảm tốc vào cua, nhớ hướng khi mất vạch. | 🟡 *Test thành phần* |
| **03** | `test_03_encoder_motor_balance.cpp` | **Kiểm thử Encoder Hall:** Đọc 4x Quadrature Decoder và tính quãng đường/vận tốc. | 🟡 *Test thành phần* |
| **04** | `test_04_mpu6050_heading_hold.cpp` | **Kiểm thử IMU MPU-6050:** Calib Gyro Z, đo góc Yaw, Heading Hold, xoay 90°/180°. | 🟡 *Test thành phần* |
| **05** | `test_05_web_telemetry_odometry.cpp` | **Tích hợp Odometry & Telemetry Web đời đầu:** Giám sát (X, Y, Yaw) ở Port 5000. | 🟡 *Lịch sử V1* |
| **06** | `test_06_encoder_1m_calibration.cpp` | **Hiệu chuẩn Encoder 1m Zero-Slip:** Phối hợp với `server/legacy/server_encoder_calib.py` (Port 5001). | 🟡 *Calib Encoder* |
| **07** | `test_07_ina226_current_sensor.cpp` | **Kiểm thử INA226:** Chẩn đoán I2C GPIO 8/9; tùy chọn kết nối `server/legacy/server_power.py` (Port 5002). | 🟡 *Calib INA226* |
| **08** | `test_08_gap_crossing_robust_line.cpp` | **Bám line & Cầu nối đứt vạch:** Dead-Reckoning Bridge, lọc nhiễu sáng 10001. | 🟡 *Lịch sử V1* |
| **09** | `test_09_bidirectional_flexible_routing.cpp` | **Điều hướng đồ thị 2 chiều đời trước:** Nền tảng chuyển sang kiến trúc lai ở `test_10`. | 🟡 *Lịch sử V1* |
| **10** | `test_10_line_centric_hybrid_recovery.cpp` | **Firmware vận hành chính thức (Default Env):** EKF + MPU Anchor + 5-IR + Bend Catch + Bridge + Cascade Wheel Control + Map-as-Code (`include/generated_map_config.h`, `MAP_HASH`, Dijkstra offline & `route_legs` Server, Token + HMAC-SHA256). | 🟢 **CHÍNH (DEFAULT)** |
| **11** | `test_11_max98357a_audio.cpp` | **Test giọng nói thật độc lập không cần PC (`MAX98357A` + Loa 8Ω 3W):** Đọc tiếng Việt 16kHz (`test_voice_samples.h`), mặc định MAX 100%, nút `BOOT` (`1 click` / `2 clicks` bậc thang `15%->100%` / `giữ 1.2s` đổi chân), tự hạ 15% & cảnh báo Brownout. | 🟡 *Test Audio* |

---

## 🛠 HƯỚNG DẪN CÁCH CHỌN VÀ NẠP BÀI TEST TRÊN VS CODE (PLATFORMIO)

1. Mở thư mục `AGV_Ver2` trên VS Code.
2. Nhìn xuống thanh trạng thái màu xanh tím ở mép dưới cửa sổ VS Code, click vào tên môi trường hiện tại (ví dụ: `env:test_10_line_centric_hybrid_recovery`).
3. Chọn môi trường bạn muốn kiểm thử:
   - `env:test_00_basic_motor_control`: Kiểm tra động cơ quay tới/lùi, rẽ, quay từng bánh.
   - `env:test_01_ir_calibration`: Cân chỉnh độ nhạy thanh dò line 5 mắt.
   - `env:test_02_line_follow_pid`: Chạy bám line tự động PD thích ứng tốc độ.
   - `env:test_03_encoder_motor_balance`: Đo quãng đường di chuyển bằng xung Encoder.
   - `env:test_04_mpu6050_heading_hold`: Kiểm tra con quay MPU-6050 & Khóa hướng chạy thẳng.
   - `env:test_05_web_telemetry_odometry`: Giám sát vị trí sa bàn Web Dashboard (Port 5000).
   - `env:test_06_encoder_1m_calibration`: Hiệu chuẩn xung Encoder chuyên dụng (Port 5001).
   - `env:test_07_ina226_current_sensor`: Chẩn đoán 2 chân GPIO 8, 9 và đo dòng/áp INA226.
   - `env:test_08_gap_crossing_robust_line`: Bám line Robust & Vượt vạch đứt đoạn / mờ / nhiễu sáng.
   - `env:test_09_bidirectional_flexible_routing`: Điều hướng đồ thị 2 chiều & Điểm-Điểm linh hoạt.
   - `env:test_10_line_centric_hybrid_recovery`: Bám line tự do (cong/zigzag) kết hợp cứu hộ đứt đoạn & khóa ZUPT MPU-6050.
   - `env:test_11_max98357a_audio`: Kiểm thử âm thanh I2S MAX98357A + loa 8Ω 3W và stress test nguồn/cảm biến.
4. Nhấn nút **PlatformIO: Upload** (biểu tượng mũi tên sang phải `->`) để nạp vào ESP32-S3.
5. Mở **Serial Monitor** (tốc độ Baud **115200**) để quan sát telemetry hoặc cho xe chạy Pin độc lập trên sàn.
