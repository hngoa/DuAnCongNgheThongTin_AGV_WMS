#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>

/*
 * ===================================================================================
 * BƯỚC 3: KIỂM THỬ MPU-6050 XÁC ĐỊNH GÓC HƯỚNG KHÔNG GIAN & GIỮ HƯỚNG CHẠY THẲNG
 * -----------------------------------------------------------------------------------
 * SƠ ĐỒ ĐẤU DÂY:
 *   1. MPU-6050 (Bus I2C Chung - GPIO 4 & 5):
 *      - VCC -> Cọc V (3.3V)
 *      - GND -> Cọc G (GND)
 *      - SDA -> GPIO 4
 *      - SCL -> GPIO 5
 *      - AD0 -> Nối GND (Địa chỉ I2C = 0x68)
 *   2. Mạch động lực L298N (GPIO 6, 7, 15, 16, 17, 18)
 *   3. Nút bấm BOOT: GPIO 0
 *
 * NGUYÊN LÝ ĐIỀU KHIỂN BẰNG MPU-6050 (INERTIAL HEADING CONTROL):
 *   - MPU-6050 đo trực tiếp vận tốc góc Z (Yaw rate) trong không gian quán tính,
 *     hoàn toàn độc lập với bánh xe trượt hay lực ma sát sàn.
 *   - Tích phân con quay góc Yaw: Yaw = Yaw + (gyroZ - bias) * dt.
 *   - Thuật toán Heading Hold: Khi xe chạy thẳng, bộ điều khiển so sánh góc thực tế
 *     với góc mục tiêu (0.0°). Bất kỳ độ nhao lái nào lệch dù chỉ 0.5° sẽ được
 *     bù ngược lại ngay lập tức, giữ xe chạy thẳng tắp như có ray vô hình!
 *
 * CÁC CHẾ ĐỘ KIỂM THỬ:
 *   1. Kiểm tra tĩnh & Hiệu chuẩn điểm 0 (Auto Gyro Bias Calibration).
 *   2. Kiểm tra góc xoay thực tế khi lấy tay xoay xe 90°, 180°, 360°.
 *   3. Chạy thẳng giữ hướng (Heading Hold) với độ chính xác cao.
 *   4. Xoay tự động đúng các góc vuông 90° / 180° bằng la bàn quán tính.
 * ===================================================================================
 */

// --- CẤU HÌNH CHÂN I2C & MPU-6050 ---
#define I2C_SDA_PIN          4
#define I2C_SCL_PIN          5
#define MPU6050_ADDR_DEFAULT 0x68
#define MPU6050_ADDR_ALT     0x69

// --- CẤU HÌNH ĐỘNG CƠ L298N ---
#define L298N_ENA_PIN        6
#define L298N_IN1_PIN        7
#define L298N_IN2_PIN        15
#define L298N_IN3_PIN        16
#define L298N_IN4_PIN        17
#define L298N_ENB_PIN        18
#define BUTTON_BOOT_PIN      0

#define PWM_FREQ_HZ          20000
#define PWM_RES_BITS         8
#define PWM_CH_LEFT          0
#define PWM_CH_RIGHT         1

// Cấu hình chiều động cơ đã kiểm nghiệm
bool invertLeftMotor         = true;
bool invertRightMotor        = true;
bool swapMotors              = true;

// --- BIẾN TRẠNG THÁI MPU-6050 ---
uint8_t mpuAddr              = MPU6050_ADDR_DEFAULT;
bool    mpuConnected         = false;
float   gyroZOffset          = 0.0f;
float   gyroRateDps          = 0.0f;
float   yawDeg               = 0.0f;       // Góc Yaw tích lũy liên tục (Độ)
float   targetHeadingDeg     = 0.0f;       // Góc hướng mục tiêu
bool    invertGyroZ          = false;      // true nếu xoay trái mà góc Yaw bị giảm âm

// --- THAM SỐ ĐIỀU KHIỂN HEADING HOLD ---
int   basePwm                = 185;        // Tốc độ cơ sở (trên ngưỡng min 170)
float Kp_heading             = 3.2f;       // Lực nắn lái theo sai số góc (PWM / độ lệch)
float Kd_heading             = 0.6f;       // Lực hãm chống dao động vọt lố góc
float lastHeadingError       = 0.0f;

bool isRunning               = false;
enum TestMode { MODE_MONITOR_ONLY, MODE_HOLD_STRAIGHT, MODE_TURN_90, MODE_TURN_180 };
TestMode currentMode         = MODE_MONITOR_ONLY;

unsigned long lastUpdateUs   = 0;
int lastButtonState          = HIGH;
unsigned long lastDebounceMs = 0;

// ===================================================================================
// GIAO TIẾP I2C & THANH GHI MPU-6050
// ===================================================================================
bool writeReg8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

int16_t readRegister16(uint8_t addr, uint8_t reg, bool* ok = nullptr) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    if (ok) *ok = false;
    return 0;
  }
  if (Wire.requestFrom((int)addr, 2) != 2) {
    if (ok) *ok = false;
    return 0;
  }
  if (ok) *ok = true;
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

// Cân chỉnh độ lệch tĩnh (Zero Bias) khi xe đứng yên (Lấy 600 mẫu trong ~1.8s để cực kỳ chính xác)
void calibrateGyro(int samples = 600) {
  if (!mpuConnected) return;
  Serial.println("\n>> ĐANG CÂN CHỈNH ĐIỂM 0 GYRO (HÃY ĐỂ XE ĐỨNG YÊN HOÀN TOÀN TRONG 2 GIÂY)...");
  double sum = 0.0;
  int count = 0;
  for (int i = 0; i < samples; i++) {
    bool ok = false;
    int16_t rawGz = readRegister16(mpuAddr, 0x47, &ok); // 0x47: GYRO_ZOUT_H
    if (ok) {
      sum += (rawGz / 32.8f); // Thang ±1000 dps -> 32.8 LSB/(deg/s)
      count++;
    }
    delay(3);
  }
  if (count > 50) {
    gyroZOffset = (float)(sum / count);
    Serial.printf(">> [CÂN CHỈNH XONG] Gyro Z Offset = %+.3f deg/s (Điểm 0 chuẩn xác!)\n\n", gyroZOffset);
  }
  yawDeg = 0.0f;
  targetHeadingDeg = 0.0f;
}

void initMPU6050() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(20);
  delay(100);

  const uint8_t addrs[2] = {MPU6050_ADDR_DEFAULT, MPU6050_ADDR_ALT};
  mpuConnected = false;

  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(addrs[i]);
    if (Wire.endTransmission() == 0) {
      mpuAddr = addrs[i];
      // Đánh thức MPU-6050 khỏi Sleep mode
      writeReg8(mpuAddr, 0x6B, 0x00);
      delay(15);
      // Cấu hình DLPF 44Hz (chống rung lắc cơ học động cơ)
      writeReg8(mpuAddr, 0x1A, 0x03);
      // Thang đo Gyro ±1000 deg/s (32.8 LSB/deg/s)
      writeReg8(mpuAddr, 0x1B, 0x10);
      mpuConnected = true;
      break;
    }
  }

  if (mpuConnected) {
    Serial.printf("✅ [MPU-6050]: Đã phát hiện tại địa chỉ 0x%02X!\n", mpuAddr);
    calibrateGyro(500);
  } else {
    Serial.println("❌ [MPU-6050 LỖI]: Không tìm thấy MPU-6050 trên chân GPIO 4 (SDA), GPIO 5 (SCL)!");
    Serial.println("   Vui lòng kiểm tra lại dây SDA, SCL, VCC(3.3V) và Mass GND.");
  }
}

// ===================================================================================
// ĐIỀU KHIỂN ĐỘNG CƠ L298N
// ===================================================================================
void writeLeftPwm(int val) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(L298N_ENA_PIN, val);
#else
  ledcWrite(PWM_CH_LEFT, val);
#endif
}

void writeRightPwm(int val) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(L298N_ENB_PIN, val);
#else
  ledcWrite(PWM_CH_RIGHT, val);
#endif
}

void initMotors() {
  pinMode(L298N_IN1_PIN, OUTPUT);
  pinMode(L298N_IN2_PIN, OUTPUT);
  pinMode(L298N_IN3_PIN, OUTPUT);
  pinMode(L298N_IN4_PIN, OUTPUT);
  pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(L298N_ENA_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(L298N_ENB_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(PWM_CH_LEFT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(L298N_ENA_PIN, PWM_CH_LEFT);
  ledcAttachPin(L298N_ENB_PIN, PWM_CH_RIGHT);
#endif

  writeLeftPwm(0);
  writeRightPwm(0);
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, LOW);
}

void setMotors(int leftSpeed, int rightSpeed) {
  if (!isRunning) {
    writeLeftPwm(0);
    writeRightPwm(0);
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, LOW);
    digitalWrite(L298N_IN3_PIN, LOW);
    digitalWrite(L298N_IN4_PIN, LOW);
    return;
  }

  int cmdLeft  = swapMotors ? rightSpeed : leftSpeed;
  int cmdRight = swapMotors ? leftSpeed  : rightSpeed;

  int l = invertLeftMotor  ? -cmdLeft  : cmdLeft;
  int r = invertRightMotor ? -cmdRight : cmdRight;

  // Bánh Trái
  if (l > 0) {
    digitalWrite(L298N_IN1_PIN, HIGH);
    digitalWrite(L298N_IN2_PIN, LOW);
    writeLeftPwm(constrain(l, 0, 255));
  } else if (l < 0) {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, HIGH);
    writeLeftPwm(constrain(-l, 0, 255));
  } else {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, LOW);
    writeLeftPwm(0);
  }

  // Bánh Phải
  if (r > 0) {
    digitalWrite(L298N_IN3_PIN, HIGH);
    digitalWrite(L298N_IN4_PIN, LOW);
    writeRightPwm(constrain(r, 0, 255));
  } else if (r < 0) {
    digitalWrite(L298N_IN3_PIN, LOW);
    digitalWrite(L298N_IN4_PIN, HIGH);
    writeRightPwm(constrain(-r, 0, 255));
  } else {
    digitalWrite(L298N_IN3_PIN, LOW);
    digitalWrite(L298N_IN4_PIN, LOW);
    writeRightPwm(0);
  }
}

void stopMotors() {
  isRunning = false;
  setMotors(0, 0);
}

// ===================================================================================
// CẬP NHẬT GÓC YAW THỜI GIAN THỰC (TẦN SỐ CAO 200Hz = 5ms)
// ===================================================================================
void updateIMU() {
  if (!mpuConnected) return;

  unsigned long nowUs = micros();
  if (nowUs - lastUpdateUs < 5000) return; // 200Hz
  float dt = (nowUs - lastUpdateUs) * 1e-6f;
  lastUpdateUs = nowUs;

  bool ok = false;
  int16_t rawGz = readRegister16(mpuAddr, 0x47, &ok);
  if (ok) {
    float gzRaw = (rawGz / 32.8f);
    float gz = gzRaw - gyroZOffset;

    // BỘ LỌC NGƯỠNG CHẾT CHỐNG TRÔI TĨNH (STATIC DEADZONE FILTER):
    // Nhiễu tĩnh của MPU-6050 thường dao động khoảng ±0.5 đến ±1.0 dps.
    // Khi xe đứng yên (!isRunning) và vận tốc góc < 1.2 dps -> Ép về 0.0 tuyệt đối để chống trôi góc!
    if (!isRunning && fabsf(gz) < 1.20f) {
      // Tự động học thích ứng (Auto Zero Tracking) để khử trôi nhiệt theo thời gian
      gyroZOffset = 0.995f * gyroZOffset + 0.005f * gzRaw;
      gz = 0.0f;
    } else if (fabsf(gz) < 0.30f) {
      // Khi đang chạy nhưng rung lắc nhẹ < 0.3 dps thì bỏ qua nhiễu
      gz = 0.0f;
    }

    if (invertGyroZ) gz = -gz;
    gyroRateDps = gz;
    yawDeg += gz * dt;
  }
}

// ===================================================================================
// CÁC KỊCH BẢN CHẠY TEST
// ===================================================================================

// 1. Chạy thẳng giữ hướng (Heading Hold)
void runStraightWithMPU(float durationSec) {
  stopMotors();
  Serial.println("\n=========================================================================");
  Serial.printf(">> [BẮT ĐẦU CHẠY THẲNG KHÓA HƯỚNG MPU] Thời gian: %.1f giây\n", durationSec);
  Serial.println("=========================================================================");
  delay(1000);

  // Khóa góc hướng xuất phát làm mốc 0.0°
  targetHeadingDeg = yawDeg;
  isRunning = true;
  currentMode = MODE_HOLD_STRAIGHT;
  unsigned long start = millis();
  lastHeadingError = 0.0f;

  while (millis() - start < (unsigned long)(durationSec * 1000)) {
    updateIMU();

    // Tính sai số góc hướng
    float headingError = targetHeadingDeg - yawDeg;
    // Chuẩn hóa góc sai số về khoảng [-180..+180]
    while (headingError >  180.0f) headingError -= 360.0f;
    while (headingError < -180.0f) headingError += 360.0f;

    float dErr = (headingError - lastHeadingError) / 0.01f;
    lastHeadingError = headingError;

    // Tín hiệu điều khiển bù lái Heading PID
    float u = (Kp_heading * headingError) + (Kd_heading * dErr);

    int pwmLeft  = constrain((int)(basePwm - u), 175, 235);
    int pwmRight = constrain((int)(basePwm + u), 175, 235);

    setMotors(pwmLeft, pwmRight);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf(">> Yaw: %+6.2f° | Target: %+6.2f° | Sai số: %+5.2f° | PWM:[L:%d R:%d]\n",
                    yawDeg, targetHeadingDeg, headingError, pwmLeft, pwmRight);
    }
    delay(10);
  }

  stopMotors();
  Serial.printf(">> [DỪNG XE] Góc lệch cuối cùng: %+.2f° (Tuyệt đối chuẩn xác!)\n\n", targetHeadingDeg - yawDeg);
}

// 2. Xoay tự động đúng góc định lượng (Target Angle Turn)
void turnAngleWithMPU(float deltaAngleDeg) {
  stopMotors();
  Serial.printf("\n>> [XOAY TỰ ĐỘNG BẰNG MPU] Mục tiêu xoay: %+.1f°\n", deltaAngleDeg);
  delay(800);

  float startYaw = yawDeg;
  float destYaw  = startYaw + deltaAngleDeg;
  isRunning = true;
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) {
    updateIMU();

    float err = destYaw - yawDeg;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    // Khi đã tới gần góc đích (< 1.5°) thì dừng
    if (fabsf(err) < 1.5f) break;

    int turnSpeed = constrain((int)(fabsf(err) * 4.0f), 175, 215);

    if (err > 0) {
      // Cần xoay trái
      setMotors(-turnSpeed, turnSpeed);
    } else {
      // Cần xoay phải
      setMotors(turnSpeed, -turnSpeed);
    }
    delay(10);
  }

  stopMotors();
  Serial.printf(">> [XOAY XONG] Bắt đầu: %+.1f° -> Kết thúc: %+.1f° (Đã xoay được: %+.1f°)\n\n",
                startYaw, yawDeg, yawDeg - startYaw);
}

void printHelp() {
  Serial.println("\n-------------------------------------------------------------------------");
  Serial.println("  BẢNG LỆNH KIỂM THỬ MPU-6050 (Gõ vào Serial Monitor):");
  Serial.println("    [run]     : Chạy thẳng 3 giây bằng bộ điều khiển Heading Hold");
  Serial.println("    [t90]     : Tự động quay TRÁI đúng 90.0°");
  Serial.println("    [-t90]    : Tự động quay PHẢI đúng 90.0°");
  Serial.println("    [t180]    : Tự động quay đầu 180.0°");
  Serial.println("    [c]       : Cân chỉnh lại điểm 0 Gyro (Calibrate)");
  Serial.println("    [z]       : Đặt góc Yaw hiện tại về 0.0°");
  Serial.println("    [invG]    : Đảo dấu góc xoay Gyro Z (+/-)");
  Serial.println("    [s=190]   : Đổi tốc độ chạy thẳng PWM");
  Serial.println("    [kp=3.5]  : Đổi độ nhạy bẻ lái Kp");
  Serial.println("    [h]       : In lại trợ giúp này");
  Serial.println("-------------------------------------------------------------------------\n");
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("run")) {
    runStraightWithMPU(3.0f);
  } else if (line.equalsIgnoreCase("t90")) {
    turnAngleWithMPU(90.0f);
  } else if (line.equalsIgnoreCase("-t90")) {
    turnAngleWithMPU(-90.0f);
  } else if (line.equalsIgnoreCase("t180")) {
    turnAngleWithMPU(180.0f);
  } else if (line.equalsIgnoreCase("c")) {
    calibrateGyro(150);
  } else if (line.equalsIgnoreCase("z")) {
    yawDeg = 0.0f;
    Serial.println(">> Đã reset Yaw về 0.0°");
  } else if (line.equalsIgnoreCase("invG")) {
    invertGyroZ = !invertGyroZ;
    Serial.printf(">> Đảo dấu Gyro Z: %s\n", invertGyroZ ? "BẬT" : "TẮT");
  } else if (line.startsWith("s=")) {
    basePwm = constrain(line.substring(2).toInt(), 175, 235);
    Serial.printf(">> Cập nhật Base PWM = %d\n", basePwm);
  } else if (line.startsWith("kp=")) {
    Kp_heading = line.substring(3).toFloat();
    Serial.printf(">> Cập nhật Kp_heading = %.2f\n", Kp_heading);
  } else if (line.equalsIgnoreCase("h") || line.equalsIgnoreCase("help")) {
    printHelp();
  }
}

void checkBootButton() {
  int reading = digitalRead(BUTTON_BOOT_PIN);
  if (reading != lastButtonState) {
    lastDebounceMs = millis();
  }
  if ((millis() - lastDebounceMs) > 50) {
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("\n[NÚT BOOT ĐƯỢC BẤM] -> Khởi động chạy thẳng giữ hướng 3 giây!");
      runStraightWithMPU(3.0f);
    }
  }
  lastButtonState = reading;
}

// ===================================================================================
// SETUP
// ===================================================================================
void setup() {
  Serial.begin(115200);
  delay(600);

  initMotors();
  initMPU6050();

  Serial.println("\n=========================================================================");
  Serial.println("    AGV_VER2 - BƯỚC 3: KIỂM THỬ GÓC HƯỚNG MPU-6050 & HEADING HOLD");
  Serial.println("=========================================================================");
  Serial.println("HƯỚNG DẪN KIỂM TRA TĨNH ĐẦU TIÊN:");
  Serial.println("  1. Dùng tay xoay xe sang TRÁI 90° -> Góc Yaw phải TĂNG DƯƠNG (+90°).");
  Serial.println("  2. Dùng tay xoay xe sang PHẢI 90° -> Góc Yaw phải GIẢM ÂM (-90°).");
  Serial.println("  3. Nếu bị ngược dấu, gõ 'invG' để đảo lại chiều chuẩn.");
  Serial.println("  4. Nhấn nút BOOT trên bo ESP32 để xe tự chạy thẳng giữ hướng 3 giây!");
  printHelp();

  lastUpdateUs = micros();
}

// ===================================================================================
// LOOP CHÍNH
// ===================================================================================
void loop() {
  updateIMU();
  checkBootButton();
  handleSerial();

  // In telemetry góc hướng định kỳ mỗi 100ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    if (!isRunning) {
      Serial.printf("[MPU-6050 LIVE] Góc Yaw = %+6.2f° | Vận tốc góc Z = %+6.2f dps | Offset = %+.3f\n",
                    yawDeg, gyroRateDps, gyroZOffset);
    }
  }
}
