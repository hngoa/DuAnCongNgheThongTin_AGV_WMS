#include <Arduino.h>
#include <esp_arduino_version.h>

/*
 * ===================================================================================
 * BƯỚC 0: KIỂM THỬ ĐIỀU KHIỂN ĐỘNG CƠ CĂN BẢN (AGV_Ver2)
 * -----------------------------------------------------------------------------------
 * HỖ TRỢ ĐỒNG THỜI 2 CHẾ ĐỘ:
 *   1. CHẾ ĐỘ PIN ĐỘC LẬP TRÊN SÀN (STANDALONE - KHÔNG CẦN CẮM DÂY MÁY TÍNH):
 *      - Sau khi bật công tắc pin, xe đếm lùi 4 giây (để bạn kịp đặt xe xuống sàn).
 *      - Xe tự động thực hiện kịch bản test chuyển động:
 *          [Tiến 2s] -> [Nghỉ 1s] -> [Lùi 2s] -> [Nghỉ 1s] -> [Xoay Trái 1s] -> [Xoay Phải 1s] -> [Dừng hẳn]
 *      - Dùng NÚT BOOT (GPIO 0) có sẵn trên bo ESP32: Bấm 1 cái để CHẠY LẠI / DỪNG XE bất kỳ lúc nào.
 *
 *   2. CHẾ ĐỘ CẮM CÁP MÁY TÍNH (SERIAL MONITOR 115200):
 *      - Điều khiển bằng phím [w, s, a, d, q, e, x, 1, 2, +, -].
 *
 * SƠ ĐỒ CHÂN DRIVER L298N:
 *   - ENA (PWM Trái) : GPIO 6  (Đã rút nắp Jumper ENA)
 *   - IN1 (Hướng T1) : GPIO 7
 *   - IN2 (Hướng T2) : GPIO 15
 *   - IN3 (Hướng P1) : GPIO 16
 *   - IN4 (Hướng P2) : GPIO 17
 *   - ENB (PWM Phải) : GPIO 18 (Đã rút nắp Jumper ENB)
 *   - Nút BOOT tích hợp ESP32-S3: GPIO 0
 * ===================================================================================
 */

#define L298N_ENA_PIN        6
#define L298N_IN1_PIN        7
#define L298N_IN2_PIN        15
#define L298N_IN3_PIN        16
#define L298N_IN4_PIN        17
#define L298N_ENB_PIN        18
#define BUTTON_BOOT_PIN      0      // Nút nhấn BOOT có sẵn trên bo ESP32-S3

#define PWM_FREQ_HZ          20000  // 20kHz êm ái
#define PWM_RES_BITS         8      // 8-bit (0..255)
#define PWM_CH_LEFT          0
#define PWM_CH_RIGHT         1

// Tốc độ chạy test cơ bản (Tối ưu theo thực nghiệm: min là 170 -> chọn 180 để xe chạy khỏe, êm)
int testSpeed                = 180; 
bool invertLeft              = true;  // Đã bật: Đảo chiều bánh Trái để quay Tiến chuẩn
bool invertRight             = true;  // Đã bật: Đảo chiều bánh Phải để quay Tiến chuẩn
bool swapMotors              = true;  // Đã bật: Hoán đổi bánh Trái <-> Phải vì đang bị cắm chéo dây

// Trạng thái nút bấm
int lastButtonState          = HIGH;
unsigned long lastDebounceTime = 0;

void writePwmLeft(int val) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(L298N_ENA_PIN, val);
#else
  ledcWrite(PWM_CH_LEFT, val);
#endif
}

void writePwmRight(int val) {
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

  writePwmLeft(0);
  writePwmRight(0);
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, LOW);
}

void setRawMotors(int leftVal, int rightVal) {
  // Hoán đổi nếu dây 2 bánh cắm chéo nhau
  int cmdLeft  = swapMotors ? rightVal : leftVal;
  int cmdRight = swapMotors ? leftVal  : rightVal;

  int l = invertLeft  ? -cmdLeft  : cmdLeft;
  int r = invertRight ? -cmdRight : cmdRight;

  // Bánh Trái
  if (l > 0) {
    digitalWrite(L298N_IN1_PIN, HIGH);
    digitalWrite(L298N_IN2_PIN, LOW);
    writePwmLeft(constrain(l, 0, 255));
  } else if (l < 0) {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, HIGH);
    writePwmLeft(constrain(-l, 0, 255));
  } else {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, LOW);
    writePwmLeft(0);
  }

  // Bánh Phải
  if (r > 0) {
    digitalWrite(L298N_IN3_PIN, HIGH);
    digitalWrite(L298N_IN4_PIN, LOW);
    writePwmRight(constrain(r, 0, 255));
  } else if (r < 0) {
    digitalWrite(L298N_IN3_PIN, LOW);
    digitalWrite(L298N_IN4_PIN, HIGH);
    writePwmRight(constrain(-r, 0, 255));
  } else {
    digitalWrite(L298N_IN3_PIN, LOW);
    digitalWrite(L298N_IN4_PIN, LOW);
    writePwmRight(0);
  }
}

void stopMotors() {
  setRawMotors(0, 0);
}

// ===================================================================================
// KỊCH BẢN CHẠY TEST TỰ ĐỘNG KHI CHẠY PIN ĐỘC LẬP
// ===================================================================================
void runAutoTestSequence() {
  Serial.println("\n>>> [BẮT ĐẦU KỊCH BẢN TEST XE ĐỘC LẬP TRÊN SÀN] <<<");

  // 1. TIẾN THẲNG 2 GIÂY
  Serial.println("1. Đang TIẾN THẲNG (2 giây)...");
  setRawMotors(testSpeed, testSpeed);
  delay(2000);

  // 2. NGHỈ 1 GIÂY
  Serial.println(">> Tạm dừng 1 giây...");
  stopMotors();
  delay(1000);

  // 3. LÙI THẲNG 2 GIÂY
  Serial.println("2. Đang LÙI THẲNG (2 giây)...");
  setRawMotors(-testSpeed, -testSpeed);
  delay(2000);

  // 4. NGHỈ 1 GIÂY
  Serial.println(">> Tạm dừng 1 giây...");
  stopMotors();
  delay(1000);

  // 5. XOAY TRÁI 1.2 GIÂY
  Serial.println("3. Đang XOAY TẠI CHỖ SANG TRÁI (1.2 giây)...");
  setRawMotors(-testSpeed, testSpeed);
  delay(1200);

  // 6. NGHỈ 1 GIÂY
  stopMotors();
  delay(1000);

  // 7. XOAY PHẢI 1.2 GIÂY
  Serial.println("4. Đang XOAY TẠI CHỖ SANG PHẢI (1.2 giây)...");
  setRawMotors(testSpeed, -testSpeed);
  delay(1200);

  // 8. HOÀN THÀNH - DỪNG HẲN
  stopMotors();
  Serial.println(">>> [HOÀN TẤT BÀI TEST CHUYỂN ĐỘNG] - Xe đã dừng an toàn!");
  Serial.println(">>> Muốn chạy lại bài test: Nhấn nút BOOT (GPIO 0) trên ESP32 một lần!");
}

void checkBootButton() {
  int reading = digitalRead(BUTTON_BOOT_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > 50) {
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("\n[NÚT BOOT ĐƯỢC BẤM] -> Khởi động lại kịch bản test xe!");
      runAutoTestSequence();
    }
  }
  lastButtonState = reading;
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'w': case 'W': setRawMotors(testSpeed, testSpeed); Serial.println("Tiến"); break;
    case 's': case 'S': setRawMotors(-testSpeed, -testSpeed); Serial.println("Lùi"); break;
    case 'a': case 'A': setRawMotors(-testSpeed, testSpeed); Serial.println("Xoay Trái"); break;
    case 'd': case 'D': setRawMotors(testSpeed, -testSpeed); Serial.println("Xoay Phải"); break;
    case 'x': case 'X': case ' ': stopMotors(); Serial.println("Dừng"); break;
    case 't': case 'T': runAutoTestSequence(); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  initMotors();
  stopMotors();

  Serial.println("\n=========================================================================");
  Serial.println("    AGV_VER2 - KIỂM THỬ ĐIỀU KHIỂN ĐỘNG CƠ CĂN BẢN (STANDALONE PIN)");
  Serial.println("=========================================================================");
  Serial.println(">> CHUẨN BỊ CHẠY ĐỘC LẬP TRÊN PIN!");
  Serial.println(">> Đếm ngược 4 giây để bạn kịp đặt xe xuống sàn...");

  // Đếm ngược 4 giây báo hiệu cho người dùng đặt xe xuống sàn
  for (int i = 4; i >= 1; i--) {
    Serial.printf(">> %d...\n", i);
    delay(1000);
  }

  // TỰ ĐỘNG CHẠY KỊCH BẢN TEST NGAY LẬP TỨC!
  runAutoTestSequence();
}

void loop() {
  // 1. Kiểm tra nút BOOT (GPIO 0) trên ESP32 để người dùng có thể kích chạy lại
  checkBootButton();

  // 2. Vẫn hỗ trợ nhận lệnh phím qua Serial nếu có cắm cáp
  handleSerial();

  delay(20);
}
