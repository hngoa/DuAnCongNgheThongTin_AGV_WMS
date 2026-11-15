#include <Arduino.h>
#include <esp_arduino_version.h>
#include "soc/usb_serial_jtag_reg.h"
#include "driver/gpio.h"

/*
 * ===================================================================================
 * BƯỚC 1.2: ĐIỀU KHIỂN AGV BÁM LINE BẰNG THUẬT TOÁN PD / PID CẢI TIẾN (AGV_Ver2)
 * -----------------------------------------------------------------------------------
 * SƠ ĐỒ ĐẤU DÂY:
 *   1. Mạch động lực L298N:
 *      - ENA (PWM Trái) : GPIO 6
 *      - IN1 (Hướng T1) : GPIO 7
 *      - IN2 (Hướng T2) : GPIO 15
 *      - IN3 (Hướng P1) : GPIO 16
 *      - IN4 (Hướng P2) : GPIO 17
 *      - ENB (PWM Phải) : GPIO 18
 *   2. Cảm biến 5 mắt TCRT5000:
 *      - L2 : GPIO 48  (w = -2.0)
 *      - L1 : GPIO 47  (w = -1.0)
 *      - C  : GPIO 21  (w =  0.0)
 *      - R1 : GPIO 20  (w = +1.0) [Alt: GPIO 45]
 *      - R2 : GPIO 19  (w = +2.0)
 *
 * CÁC TÍNH NĂNG VÀ CẢI TIẾN TRỌNG TÂM:
 *   - Điều khiển tần số lấy mẫu cố định 100Hz (dt = 10ms) chuẩn xác.
 *   - Thuật toán PD chống dao động lắc đuôi (Fishtailing Dampening).
 *   - Bộ nhớ hướng khi mất vạch (Lost-line memory): Nhớ hướng lệch trước đó để tự
 *     quay xe trở lại vạch thay vì văng ra ngoài sa bàn.
 *   - Tự động dừng khẩn cấp an toàn nếu mất vạch quá 1.5 giây.
 *   - Cân bằng động cơ và ôm cua góc vuông (Pivot turn khi |e| >= 1.8).
 *   - CÂN CHỈNH LIVE TRỰC TIẾP QUA SERIAL MONITOR (Không cần nạp lại code):
 *       'start' / 'stop' : Chạy / Dừng khẩn cấp
 *       'p=38.5'         : Đổi hệ số Kp
 *       'd=14.0'         : Đổi hệ số Kd
 *       's=140'          : Đổi tốc độ Base Speed (0..255)
 *       'invL' / 'invR'  : Đảo chiều động cơ Trái / Phải
 *       'swap'           : Đảo thứ tự mắt cảm biến Trái <-> Phải
 *       'info'           : Xem thông số hiện tại
 * ===================================================================================
 */

// --- CẤU HÌNH CHÂN ĐỘNG CƠ L298N ---
#define L298N_ENA_PIN        6
#define L298N_IN1_PIN        7
#define L298N_IN2_PIN        15
#define L298N_IN3_PIN        16
#define L298N_IN4_PIN        17
#define L298N_ENB_PIN        18
#define BUTTON_BOOT_PIN      0      // Nút nhấn BOOT tích hợp trên ESP32-S3

#define PWM_FREQ_HZ          20000  // 20kHz êm ái, triệt tiêu tiếng rít động cơ
#define PWM_RES_BITS         8      // 8-bit (0..255)
#define PWM_CH_LEFT          0
#define PWM_CH_RIGHT         1

// --- CẤU HÌNH CHÂN CẢM BIẾN TCRT5000 ---
int pinL2     = 48;
int pinL1     = 47;
int pinC      = 21;
int pinR1     = 20;
int pinR2     = 19;
const int PIN_R1_ALT = 45;

// --- BIẾN TRẠNG THÁI HỆ THỐNG ---
bool isRunning       = false; // Sẽ tự bật sau 4s đếm ngược khi cấp nguồn Pin
bool invertLeftMotor = true;  // Đã bật: Đảo chiều bánh Trái
bool invertRightMotor= true;  // Đã bật: Đảo chiều bánh Phải
bool swapMotors      = true;  // Đã bật: Hoán đổi bánh Trái <-> Phải vì bị cắm chéo dây
bool invertLogic     = false; // false: HIGH = vạch đen; true: LOW = vạch đen
bool swapSensors     = false; // true: Lật L2 <-> R2
bool useAltPinR1     = false;

int lastButtonState  = HIGH;
unsigned long lastDebounceTime = 0;

// --- THAM SỐ ĐIỀU KHIỂN PD & TỐC ĐỘ THÍCH ỨNG (ADAPTIVE VELOCITY) ---
float Kp             = 36.0f;
float Kd             = 15.0f;
int   straightPwm    = 220;   // Tốc độ phóng đường thẳng (tăng mạnh từ 180 lên 220)
int   curvePwm       = 178;   // Tốc độ khi ôm cua (giảm tốc an toàn trên ngưỡng min 170)
int   maxPwm         = 255;   // Giới hạn xung PWM kịch kim
int   turnPwm        = 175;   // Tốc độ xoay tại chỗ tìm vạch khi mất dấu

// --- BIẾN VÒNG LẶP ĐIỀU KHIỂN ---
float currentError   = 0.0f;
float lastError      = 0.0f;
float derivative     = 0.0f;
float lastValidError = 0.0f;  // Bộ nhớ hướng lệch cuối cùng trước khi mất vạch
unsigned long lastLoopTimeUs = 0;
unsigned long lineLostTimeMs = 0;
const unsigned long LOST_TIMEOUT_MS = 1500; // Dừng sau 1.5s mất vạch hoàn toàn

// ===================================================================================
// HÀM XUẤT PWM TƯƠNG THÍCH MỌI PHIÊN BẢN ESP32 ARDUINO CORE (2.x và 3.x)
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

  // Hoán đổi nếu dây 2 bánh cắm chéo nhau
  int cmdLeft  = swapMotors ? rightSpeed : leftSpeed;
  int cmdRight = swapMotors ? leftSpeed  : rightSpeed;

  // Đảo chiều phần mềm nếu cần
  int l = invertLeftMotor  ? -cmdLeft  : cmdLeft;
  int r = invertRightMotor ? -cmdRight : cmdRight;

  // Điều khiển Bánh Trái
  if (l > 0) {
    digitalWrite(L298N_IN1_PIN, HIGH);
    digitalWrite(L298N_IN2_PIN, LOW);
    writeLeftPwm(constrain(l, 0, maxPwm));
  } else if (l < 0) {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, HIGH);
    writeLeftPwm(constrain(-l, 0, maxPwm));
  } else {
    digitalWrite(L298N_IN1_PIN, LOW);
    digitalWrite(L298N_IN2_PIN, LOW);
    writeLeftPwm(0);
  }

  // Điều khiển Bánh Phải
  if (r > 0) {
    digitalWrite(L298N_IN3_PIN, HIGH);
    digitalWrite(L298N_IN4_PIN, LOW);
    writeRightPwm(constrain(r, 0, maxPwm));
  } else if (r < 0) {
    digitalWrite(L298N_IN3_PIN, LOW);
    digitalWrite(L298N_IN4_PIN, HIGH);
    writeRightPwm(constrain(-r, 0, maxPwm));
  } else {
    digitalWrite(L298N_IN3_PIN, LOW);
    digitalWrite(L298N_IN4_PIN, LOW);
    writeRightPwm(0);
  }
}

void stopMotors() {
  setMotors(0, 0);
}

// ===================================================================================
// CẤU HÌNH VÀ ĐỌC CẢM BIẾN TCRT5000
// ===================================================================================
void initSensors() {
  // Giải phóng GPIO 19 và GPIO 20 khỏi USB PHY
  CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG,
                      USB_SERIAL_JTAG_USB_PAD_ENABLE |
                      USB_SERIAL_JTAG_DP_PULLUP |
                      USB_SERIAL_JTAG_DM_PULLUP |
                      USB_SERIAL_JTAG_DP_PULLDOWN |
                      USB_SERIAL_JTAG_DM_PULLDOWN);
  SET_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_PAD_PULL_OVERRIDE);

  gpio_reset_pin((gpio_num_t)pinL2);
  gpio_reset_pin((gpio_num_t)pinL1);
  gpio_reset_pin((gpio_num_t)pinC);
  gpio_reset_pin((gpio_num_t)pinR1);
  gpio_reset_pin((gpio_num_t)pinR2);
  gpio_reset_pin((gpio_num_t)PIN_R1_ALT);

  pinMode(pinL2, INPUT_PULLUP);
  pinMode(pinL1, INPUT_PULLUP);
  pinMode(pinC,  INPUT_PULLUP);
  pinMode(pinR1, INPUT_PULLUP);
  pinMode(pinR2, INPUT_PULLUP);
  pinMode(PIN_R1_ALT, INPUT_PULLUP);
}

int readSensorBits(int &sL2, int &sL1, int &sC, int &sR1, int &sR2) {
  int curR1Pin = useAltPinR1 ? PIN_R1_ALT : pinR1;

  int rL2 = digitalRead(pinL2);
  int rL1 = digitalRead(pinL1);
  int rC  = digitalRead(pinC);
  int rR1 = digitalRead(curR1Pin);
  int rR2 = digitalRead(pinR2);

  sL2 = invertLogic ? !rL2 : rL2;
  sL1 = invertLogic ? !rL1 : rL1;
  sC  = invertLogic ? !rC  : rC;
  sR1 = invertLogic ? !rR1 : rR1;
  sR2 = invertLogic ? !rR2 : rR2;

  if (swapSensors) {
    int tL2 = sL2, tL1 = sL1;
    sL2 = sR2;
    sL1 = sR1;
    sR1 = tL1;
    sR2 = tL2;
  }

  return (sL2 << 4) | (sL1 << 3) | (sC << 2) | (sR1 << 1) | sR2;
}

// ===================================================================================
// XỬ LÝ LỆNH TƯƠNG TÁC SERIAL MONITOR (LIVE TUNING)
// ===================================================================================
void printCurrentConfig() {
  Serial.println("\n--- [AGV_VER2 CẤU HÌNH HIỆN TẠI] ---");
  Serial.printf("  Trạng thái chạy : %s\n", isRunning ? "RUNNING (ĐANG CHẠY)" : "STOPPED (ĐANG DỪNG)");
  Serial.printf("  Kp = %.2f | Kd = %.2f\n", Kp, Kd);
  Serial.printf("  Straight Speed = %d | Curve Speed = %d | Turn Speed = %d\n", straightPwm, curvePwm, turnPwm);
  Serial.printf("  Đảo động cơ     : Trái=%s | Phải=%s\n", invertLeftMotor ? "CÓ" : "KHÔNG", invertRightMotor ? "CÓ" : "KHÔNG");
  Serial.printf("  Đảo cảm biến    : MứcLogic=%s | ĐảoTráiPhải=%s\n", invertLogic ? "LOW" : "HIGH", swapSensors ? "CÓ" : "KHÔNG");
  Serial.println("------------------------------------\n");
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("start")) {
    isRunning = true;
    Serial.println(">> [LỆNH] AGV KHỞI ĐỘNG CHẠY BÁM LINE!");
  } else if (line.equalsIgnoreCase("stop")) {
    isRunning = false;
    stopMotors();
    Serial.println(">> [LỆNH] AGV ĐÃ DỪNG KHẨN CẤP!");
  } else if (line.startsWith("p=") || line.startsWith("P=")) {
    Kp = line.substring(2).toFloat();
    Serial.printf(">> [CẤU HÌNH] Cập nhật Kp = %.2f\n", Kp);
  } else if (line.startsWith("d=") || line.startsWith("D=")) {
    Kd = line.substring(2).toFloat();
    Serial.printf(">> [CẤU HÌNH] Cập nhật Kd = %.2f\n", Kd);
  } else if (line.startsWith("s=") || line.startsWith("S=")) {
    straightPwm = constrain(line.substring(2).toInt(), 175, 255);
    Serial.printf(">> [CẤU HÌNH] Cập nhật Straight PWM = %d\n", straightPwm);
  } else if (line.equalsIgnoreCase("invL")) {
    invertLeftMotor = !invertLeftMotor;
    Serial.printf(">> [CẤU HÌNH] Đảo chiều động cơ Trái: %s\n", invertLeftMotor ? "BẬT" : "TẮT");
  } else if (line.equalsIgnoreCase("invR")) {
    invertRightMotor = !invertRightMotor;
    Serial.printf(">> [CẤU HÌNH] Đảo chiều động cơ Phải: %s\n", invertRightMotor ? "BẬT" : "TẮT");
  } else if (line.equalsIgnoreCase("swap")) {
    swapSensors = !swapSensors;
    Serial.printf(">> [CẤU HÌNH] Đảo mắt Trái/Phải: %s\n", swapSensors ? "BẬT" : "TẮT");
  } else if (line.equalsIgnoreCase("info")) {
    printCurrentConfig();
  } else {
    Serial.println(">> Lệnh không nhận diện. Lệnh hợp lệ: start, stop, p=.., d=.., s=.., invL, invR, swap, info");
  }
}

// ===================================================================================
// SETUP CHƯƠNG TRÌNH
// ===================================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  initSensors();
  initMotors();
  pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

  Serial.println("\n=========================================================================");
  Serial.println("    AGV_VER2 - BƯỚC 1.2: KIỂM THỬ BÁM LINE VỚI THUẬT TOÁN PD CẢI TIẾN");
  Serial.println("=========================================================================");
  Serial.println(">> CHẾ ĐỘ NGUỒN PIN ĐỘC LẬP TRÊN SÀN (STANDALONE)!");
  Serial.println(">> Đang đếm ngược 4 giây để bạn kịp đặt xe vào giữa vạch line...");
  printCurrentConfig();

  // Đếm ngược 4 giây để người dùng kịp đặt xe lên vạch trên sàn
  for (int i = 4; i >= 1; i--) {
    Serial.printf(">> %d...\n", i);
    delay(1000);
  }

  // TỰ ĐỘNG BẬT CHẾ ĐỘ CHẠY BÁM LINE
  isRunning = true;
  Serial.println(">> [BẮT ĐẦU CHẠY BÁM LINE TỰ ĐỘNG]");
  Serial.println(">> Nút BOOT (GPIO 0): Nhấn để TẠM DỪNG / CHẠY LẠI bất cứ lúc nào!");

  lastLoopTimeUs = micros();
}

void checkBootButton() {
  int reading = digitalRead(BUTTON_BOOT_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > 50) {
    if (reading == LOW && lastButtonState == HIGH) {
      isRunning = !isRunning;
      if (!isRunning) stopMotors();
      Serial.printf("\n[NÚT BOOT ĐÃ BẤM] -> Trạng thái xe: %s\n", isRunning ? "ĐANG CHẠY" : "ĐÃ DỪNG XE");
    }
  }
  lastButtonState = reading;
}

// ===================================================================================
// LOOP ĐIỀU KHIỂN CHÍNH (Tần số 100Hz = 10ms)
// ===================================================================================
void loop() {
  // 1. Kiểm tra nút BOOT cứng trên bo mạch ESP32
  checkBootButton();

  // 2. Nhận lệnh Serial nếu có cắm cáp
  handleSerialCommands();

  unsigned long nowUs = micros();
  if (nowUs - lastLoopTimeUs < 10000) {
    return; // Đảm bảo chu kỳ lấy mẫu chính xác 10ms (100Hz)
  }
  float dt = (nowUs - lastLoopTimeUs) * 1e-6f;
  lastLoopTimeUs = nowUs;

  // 1. Đọc cảm biến 5 mắt
  int sL2, sL1, sC, sR1, sR2;
  int bitmask = readSensorBits(sL2, sL1, sC, sR1, sR2);
  int activeCount = sL2 + sL1 + sC + sR1 + sR2;

  int leftPwm  = 0;
  int rightPwm = 0;
  const char* actionState = "";

  // 2. Phân tích các trường hợp bám line
  if (activeCount == 0) {
    // --- TRƯỜNG HỢP 1: MẤT VẠCH HOÀN TOÀN (00000) ---
    if (lineLostTimeMs == 0) {
      lineLostTimeMs = millis();
    }

    if (millis() - lineLostTimeMs > LOST_TIMEOUT_MS) {
      // Quá 1.5s không thấy vạch -> Dừng an toàn
      actionState = "🛑 QUÁ HẠN MẤT VẠCH -> DỪNG XE";
      leftPwm  = 0;
      rightPwm = 0;
    } else {
      // Dùng BỘ NHỚ HƯỚNG LỆCH CUỐI CÙNG (Last Error Memory) để tìm lại vạch
      if (lastValidError < -0.3f) {
        // Vừa lệch sang trái -> Xe đã văng ra bên phải line -> Xoay trái gắt
        actionState = "🔄 MẤT VẠCH -> XOAY TRÁI TÌM VẠCH";
        leftPwm  = -turnPwm;
        rightPwm = +turnPwm;
      } else if (lastValidError > 0.3f) {
        // Vừa lệch sang phải -> Xe đã văng ra bên trái line -> Xoay phải gắt
        actionState = "🔄 MẤT VẠCH -> XOAY PHẢI TÌM VẠCH";
        leftPwm  = +turnPwm;
        rightPwm = -turnPwm;
      } else {
        // Không rõ hướng -> Rà chậm thẳng tới
        actionState = "⚠️ MẤT VẠCH -> TRƯỢT THẲNG DÒ TÌM";
        leftPwm  = turnPwm;
        rightPwm = turnPwm;
      }
    }
  } else if (activeCount == 5 || (activeCount >= 4 && sL1 && sC && sR1)) {
    // --- TRƯỜNG HỢP 2: VẠCH DỪNG HOẶC NGÃ TƯ (11111) ---
    lineLostTimeMs = 0;
    actionState = "🏁 VẠCH DỪNG / GIAO LỘ";
    currentError = 0.0f;
    leftPwm  = curvePwm;
    rightPwm = curvePwm;
    lastError = 0.0f;
  } else {
    // --- TRƯỜNG HỢP 3: BÌNH THƯỜNG TRÊN VẠCH -> THUẬT TOÁN PD ---
    lineLostTimeMs = 0;

    // Trọng số: L2=-2, L1=-1, C=0, R1=+1, R2=+2
    currentError = (float)(-2 * sL2 + -1 * sL1 + 0 * sC + 1 * sR1 + 2 * sR2) / (float)activeCount;
    lastValidError = currentError;

    // Tính đạo hàm D (tốc độ thay đổi sai số để chống văng / lắc)
    derivative = (currentError - lastError) / fmaxf(dt, 0.001f);
    lastError = currentError;

    // TỐC ĐỘ THÍCH ỨNG (ADAPTIVE SPEED CONTROL):
    // Đường thẳng (|e| < 0.35) -> TĂNG TỐC MẠNH lên straightPwm (220)
    // Vào cua gắt (|e| lớn)    -> TỰ ĐỘNG GIẢM TỐC về curvePwm (178) để không bị văng vạch
    float absErr = fabsf(currentError);
    int effectiveBase = straightPwm - (int)(absErr * (straightPwm - curvePwm));
    effectiveBase = constrain(effectiveBase, curvePwm, straightPwm);

    // Tín hiệu điều khiển PD
    float u = (Kp * currentError) + (Kd * derivative);

    // Xử lý góc vuông / ôm cua gắt (Pivot turn khi chỉ còn mắt ngoài L2 hoặc R2)
    if (currentError <= -1.8f) {
      actionState = "⮌ ÔM CUA TRÁI GẮT (PIVOT LEFT)";
      leftPwm  = -turnPwm;
      rightPwm = curvePwm + 25;
    } else if (currentError >= 1.8f) {
      actionState = "⮎ ÔM CUA PHẢI GẮT (PIVOT RIGHT)";
      leftPwm  = curvePwm + 25;
      rightPwm = -turnPwm;
    } else {
      actionState = (absErr < 0.35f) ? "🚀 PHÓNG NHANH ĐƯỜNG THẲNG" : "✔ BÁM LINE (PD)";
      leftPwm  = constrain((int)(effectiveBase + u), 0, maxPwm);
      rightPwm = constrain((int)(effectiveBase - u), 0, maxPwm);
    }
  }

  // 3. Truyền lệnh điều tốc xuống hai động cơ
  setMotors(leftPwm, rightPwm);

  // 4. In thông tin telemetry định kỳ mỗi 100ms ra Serial Monitor
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs > 100) {
    lastPrintMs = millis();
    Serial.printf("[%s%s%s%s%s] e=%+5.2f | PWM:[L:%4d R:%4d] | %s\n",
                  sL2 ? "█" : ".",
                  sL1 ? "█" : ".",
                  sC  ? "█" : ".",
                  sR1 ? "█" : ".",
                  sR2 ? "█" : ".",
                  currentError,
                  leftPwm, rightPwm,
                  actionState);
  }
}
