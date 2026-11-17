# KẾ HOẠCH & QUY TRÌNH KIỂM THỬ TỪNG BƯỚC (STEP-BY-STEP TESTING) - AGV_VER2

Thư mục `test/` được tổ chức thành các bài kiểm thử độc lập tương ứng với từng cụm chức năng phần cứng và thuật toán điều khiển của xe AGV.
Mỗi bài test là một file độc lập có `setup()` và `loop()`, có thể nạp trực tiếp qua PlatformIO mà không làm xáo trộn mã nguồn chính trong `src/main.cpp`.
Sau khi từng phần được kiểm thử ổn định, code sẽ được tích hợp lại thành một hệ thống hoàn chỉnh.

---

## 📌 DANH SÁCH CÁC NHIỆM VỤ KIỂM THỬ

| Bước | Tên File Test | Mục tiêu & Chức năng kiểm thử | Trạng thái |
| :---: | :--- | :--- | :---: |
| **00** | `test_00_basic_motor_control.cpp` | **Kiểm thử động cơ căn bản (Motor Check):**<br>- Điều khiển Tiến, Lùi, Xoay Trái/Phải, Bẻ lái, Dừng.<br>- Kiểm tra độc lập từng bánh Trái/Phải.<br>- Kịch bản tự động (Auto Demo), đảo chiều motor bằng lệnh.<br>- Cơ chế Watchdog tự dừng an toàn sau 3s. | 🟢 **ĐÃ HOÀN THÀNH** |
| **01** | `test_01_ir_calibration.cpp` | **Hiệu chỉnh cảm biến dò line TCRT5000 (5 mắt):**<br>- Tự động giải phóng GPIO 19, 20 khỏi USB PHY.<br>- Tự động/thủ công đảo chiều bẹ dây (Trái/Phải).<br>- Tùy biến mức logic vạch đen/trắng.<br>- Hiển thị đồ họa trực quan và tính sai số lệch tâm $e$. | 🟢 **ĐÃ HOÀN THÀNH** |
| **02** | `test_02_line_follow_pid.cpp` | **Điều khiển AGV bám line bằng thuật toán PD cải tiến:**<br>- Điều khiển L298N qua PWM 20kHz êm ái.<br>- Tốc độ thích ứng: Phóng nhanh đường thẳng (`220`), giảm tốc khi vào cua (`178`).<br>- Thuật toán PD có bộ nhớ hướng khi mất vạch, ôm cua $90^\circ$ gắt.<br>- Chỉnh tham số Kp, Kd, Speed trực tiếp qua Serial hoặc nút BOOT. | 🟢 **ĐÃ HOÀN THÀNH** |
| **03** | `test_03_encoder_motor_balance.cpp` | **Kiểm thử Encoder Hall & Quãng đường di chuyển:**<br>- Đọc ngắt xung 4x Quadrature Decoder từ động cơ JGA25-370.<br>- Tính quãng đường thực tế (Mét), vận tốc tức thời (cm/s).<br>- Tự động chạy thẳng đúng 1.5 mét rồi dừng (chuẩn theo chặng sa bàn). | 🟢 **ĐÃ HOÀN THÀNH** |
| **04** | `test_04_mpu6050_heading_hold.cpp` | **Kiểm thử IMU MPU-6050 & Khóa góc hướng không gian:**<br>- Tự động cân chỉnh điểm 0 Gyro Z (Bias Calibration).<br>- Đo chính xác góc xoay Yaw $\theta$ thời gian thực (xoay thử $90^\circ, 180^\circ$).<br>- Thuật toán Heading Hold: Khóa góc hướng chạy thẳng tắp trong không gian.<br>- Tự động xoay chuẩn xác $90^\circ$ và $180^\circ$ bằng la bàn quán tính. | 🟢 **ĐÃ HOÀN THÀNH** |
| **05** | `test_05_web_telemetry_odometry.cpp` | **Tích hợp Odometry (X, Y, Yaw) & Web Dashboard (Port 5000):**<br>- Giám sát vị trí xe thời gian thực trên sa bàn 154cm x 151cm.<br>- Bám line PD kết hợp dự đoán hãm phanh 35cm trước trạm.<br>- Khử trôi tĩnh ZUPT và tự động mỏ neo trạm dừng. | 🟢 **ĐÃ HOÀN THÀNH** |
| **06** | `test_06_encoder_1m_calibration.cpp` | **Hiệu chuẩn Encoder 1 Mét Chuyên Dụng (Zero-Slip - Port 5001):**<br>- Động cơ thả trôi tự do, đo số xung thực tế khi đẩy xe 1m có tải.<br>- Server Port 5001 tự động tính Ticks/m, Ticks/cm, đường kính lốp.<br>- Lưu lịch sử nhiều lần đo và tính trung bình vàng để nạp code. | 🟢 **ĐANG KIỂM THỬ** |
| **07** | `test_07_ina226_current_sensor.cpp` | **Kiểm thử cảm biến INA226 & Server Quản Lý Năng Lượng (Port 5002):**<br>- Chẩn đoán điện áp tĩnh HIGH/LOW trên chân SDA=GPIO 8, SCL=GPIO 9.<br>- Nhận diện Chip ID TI (0x5449 & 0x2260), Stress test 100 gói dữ liệu.<br>- Tích phân dung lượng tiêu thụ (mAh) và điện năng (Wh) thời gian thực.<br>- Web Dashboard riêng (Port 5002): Đồng hồ % Pin, Đồ thị sóng V/A/W, Xuất CSV.<br>- Hỗ trợ phím `swap`, `s`, `t`, `r`, `zero`, `cal` trên Serial Monitor. | 🟢 **ĐÃ SẴN SÀNG** |
| **08** | `test_08_gap_crossing_robust_line.cpp` | **Bám line Robust & Vượt vạch đứt đoạn / mờ / nhiễu sáng:**<br>- Cầu nối quán tính Dead-Reckoning Virtual Line Bridge.<br>- Bộ lọc độ tin cậy kháng vạch mờ chập chờn (30ms debounce).<br>- Lọc không gian loại bỏ nhiễu lóa sáng quang học (mẫu 10001).<br>- Tính năng giả lập đứt vạch 25cm bằng phím 'g' để thử nghiệm tức thì.<br>- Kế thừa trọn vẹn Odometry tâm bánh, tiếp cận trạm và khóa vạch góc xoay. | 🟢 **ĐÃ HOÀN THÀNH** |
| **09** | `test_09_bidirectional_flexible_routing.cpp` | **Điều hướng Đồ thị 2 Chiều & Điểm - Điểm linh hoạt:**<br>- Mô hình đồ thị 8 chặng 2 chiều đầy đủ (AB, BC, CD, DA & BA, CB, DC, AD).<br>- Thuật toán khóa vạch tiệm cận (|err| <= 35°) chống bắt nhầm nhánh khi quay 180°.<br>- Bộ định tuyến Điểm - Điểm (GO_TO:A, B, C, D) tự động lập lộ trình ngắn nhất.<br>- Ma trận 3 cụm nút điều khiển trên Web: Điều hướng nhanh, Điểm-Điểm, 8 Chặng 2 chiều.<br>- Tích hợp toàn diện Cầu nối quán tính vượt đứt vạch và khử trôi đa tầng. | 🟢 **ĐÃ HOÀN THÀNH** |
| **10** | `test_10_line_centric_hybrid_recovery.cpp` | **Kiến Trúc Lai Cân Bằng MPU Anchor + Dual-State Line Catch:**<br>- Hợp nhất MPU-6050 (`pureMpuYawDeg` + cổng nhất quán động học `<= 7°`, giới hạn lệch EKF `±2.5°`): giữ ổn định trạm Test 09 nhưng không bao giờ để 5-IR bẻ sai góc MPU trên nhánh chéo `<`.<br>- Nâng cấp bám line: lọc đa số 5 lần chống chói gạch bóng, tăng trọng số mẫu góc gập (`7/28/15/30 -> ±1.75`), mở khóa bánh trong `0.02 m/s`, pivot `±1.40`.<br>- Tách mất vạch thẳng (`Bridge` quét sin `0.72m`) vs mất tại đỉnh góc gập/< (`SUB_CORNER_BEND_CATCH` dùng MPU quét hẹp trong nón đích `±88°`). | 🟢 **MỚI TẠO** |
| **11** | `../src/main.cpp` | **TÍCH HỢP TOÀN DIỆN HỆ THỐNG AGV_VER2** | ⏳ *Giai đoạn cuối* |

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
4. Nhấn nút **PlatformIO: Upload** (biểu tượng mũi tên sang phải `->`) để nạp vào ESP32-S3.
5. Mở **Serial Monitor** (tốc độ Baud **115200**) để quan sát telemetry hoặc cho xe chạy Pin độc lập trên sàn.
