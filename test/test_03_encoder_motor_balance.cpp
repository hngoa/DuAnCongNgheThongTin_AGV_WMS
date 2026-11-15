#include <Arduino.h>
#include <esp_arduino_version.h>

/*
 * ===================================================================================
 * BƯỚC 2 (CẬP NHẬT): KIỂM THỬ ENCODER HALL & CÂN BẰNG TỐC ĐỘ 2 BÁNH (AGV_Ver2)
 * -----------------------------------------------------------------------------------
 * GIẢI MÃ NGUYÊN NHÂN XE BỊ NHÍCH RỒI LỆCH TRÁI:
 *   1. Hai động cơ lắp đối xứng nhau qua tâm xe (gương pha), nên khi xe cùng tiến,
 *      trục bánh Phải quay ngược chiều so với trục bánh Trái.
 *   2. Ngắt Encoder bánh Phải cần đảo dấu để khi TIẾN cả 2 bánh cùng tăng (+).
 *   3. Giới hạn xung PWM không bao giờ được tụt xuống dưới ngưỡng min 175 để tránh kẹt bánh.
 *
 * SƠ ĐỒ CHÂN:
 *   - L298N : ENA=6, IN1=7, IN2=15, IN3=16, IN4=17, ENB=18
 *   - Encoder Trái: Pha A = GPIO 10, Pha B = GPIO 11
 *   - Encoder Phải: Pha A = GPIO 12, Pha B = GPIO 13
 *   - Nút BOOT ESP32: GPIO 0
 * ===================================================================================
 */

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

#define ENC_L_A_PIN          10
#define ENC_L_B_PIN          11
#define ENC_R_A_PIN          12
#define ENC_R_B_PIN          13

// Cấu hình động cơ chuẩn đã kiểm nghiệm ở Bước 0
bool invertLeftMotor         = true;
bool invertRightMotor        = true;
bool swapMotors              = true;

// Cấu hình Encoder: Ngắt ISR đã tự đảo logic so sánh nên không cần invert
bool invertEncLeft           = false;
bool invertEncRight          = false;  // ĐÃ SỬA: Đặt false để cả 2 bánh cùng tăng (+) khi tiến
bool swapEncoders            = true;   // Hoán đổi đúng vị trí bánh Trái/Phải

// Hệ số chuẩn cho bánh 65mm (Quad-edge x4 trên động cơ 170RPM: 1.51m = 18611 ticks)
float ticksPerMeter          = 12325.0f; 

volatile long rawTicksL      = 0;
volatile long rawTicksR      = 0;

long prevTicksL              = 0;
long prevTicksR              = 0;
float currentSpeedL_cm_s     = 0.0f;
float currentSpeedR_cm_s     = 0.0f;
float currentSpeedCar_cm_s   = 0.0f;
float distanceCar_m          = 0.0f;

// Tốc độ chạy test (Đặt 195 để có biên dao động cân bằng [175..215] luôn trên min 170)
int  testPwm                 = 195;
bool isRunning               = false;
unsigned long lastCalcTimeMs = 0;
int lastButtonState          = HIGH;
unsigned long lastDebounceMs = 0;

// ===================================================================================
// TRÌNH PHỤC VỤ NGẮT ĐẾM XUNG
// ===================================================================================
void IRAM_ATTR isrEncLeftA() {
  if (digitalRead(ENC_L_A_PIN) == digitalRead(ENC_L_B_PIN)) rawTicksL++;
  else rawTicksL--;
}
void IRAM_ATTR isrEncLeftB() {
  if (digitalRead(ENC_L_A_PIN) != digitalRead(ENC_L_B_PIN)) rawTicksL++;
  else rawTicksL--;
}
void IRAM_ATTR isrEncRightA() {
  // Động cơ đối xứng: Đảo chiều điều kiện so sánh
  if (digitalRead(ENC_R_A_PIN) != digitalRead(ENC_R_B_PIN)) rawTicksR++;
  else rawTicksR--;
}
void IRAM_ATTR isrEncRightB() {
  if (digitalRead(ENC_R_A_PIN) == digitalRead(ENC_R_B_PIN)) rawTicksR++;
  else rawTicksR--;
}

void getCalibratedTicks(long &outL, long &outR) {
  noInterrupts();
  long tL = rawTicksL;
  long tR = rawTicksR;
  interrupts();

  if (invertEncLeft)  tL = -tL;
  if (invertEncRight) tR = -tR;

  if (swapEncoders) {
    outL = tR;
    outR = tL;
  } else {
    outL = tL;
    outR = tR;
  }
}

void resetEncoderTicks() {
  noInterrupts();
  rawTicksL = 0;
  rawTicksR = 0;
  interrupts();
  prevTicksL = 0;
  prevTicksR = 0;
  distanceCar_m = 0.0f;
}

// ===================================================================================
// ĐIỀU KHIỂN ĐỘNG CƠ
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

void initEncoders() {
  pinMode(ENC_L_A_PIN, INPUT_PULLUP);
  pinMode(ENC_L_B_PIN, INPUT_PULLUP);
  pinMode(ENC_R_A_PIN, INPUT_PULLUP);
  pinMode(ENC_R_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A_PIN), isrEncLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B_PIN), isrEncLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A_PIN), isrEncRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B_PIN), isrEncRightB, CHANGE);
}

// ===================================================================================
// KIỂM TRA ĐỘC LẬP TỪNG BÁNH (ĐẢM BẢO XUNG TĂNG DƯƠNG KHI TIẾN)
// ===================================================================================
void testSingleWheel(int wheel) {
  stopMotors();
  resetEncoderTicks();
  Serial.printf("\n>> [TEST RIÊNG BÁNH %s] Bánh sẽ quay TIẾN 1.5 giây...\n", wheel == 1 ? "TRÁI" : "PHẢI");
  delay(1000);

  isRunning = true;
  if (wheel == 1) setMotors(testPwm, 0);
  else            setMotors(0, testPwm);
  delay(1500);

  stopMotors();
  long curL, curR;
  getCalibratedTicks(curL, curR);
  Serial.printf(">> KẾT QUẢ ĐẾM XUNG: Bánh Trái = %+ld ticks | Bánh Phải = %+ld ticks\n", curL, curR);
  if (wheel == 1) {
    Serial.printf(">> ĐÁNH GIÁ BÁNH TRÁI: %s (Cần > 0 khi tiến)\n\n", curL > 50 ? "✔ ĐẠT CHUẨN (+)" : "❌ LỖI (Bị âm hoặc không đếm)");
  } else {
    Serial.printf(">> ĐÁNH GIÁ BÁNH PHẢI: %s (Cần > 0 khi tiến)\n\n", curR > 50 ? "✔ ĐẠT CHUẨN (+)" : "❌ LỖI (Bị âm hoặc không đếm)");
  }
}

// ===================================================================================
// CHẠY THẲNG ĐO QUÃNG ĐƯỜNG 1.5 MÉT
// ===================================================================================
void runDistanceTest(float targetMeters) {
  stopMotors();
  resetEncoderTicks();
  Serial.println("\n=========================================================================");
  Serial.printf(">> [BẮT ĐẦU CHẠY THẲNG ĐO QUÃNG ĐƯỜNG: %.2f MÉT] <<\n", targetMeters);
  Serial.println("=========================================================================");
  delay(1000);

  isRunning = true;
  unsigned long startTime = millis();
  float currentDist = 0.0f;

  while (currentDist < targetMeters && (millis() - startTime < 12000)) {
    long curL, curR;
    getCalibratedTicks(curL, curR);
    currentDist = (float)(labs(curL) + labs(curR)) / 2.0f / ticksPerMeter;

    // Bộ bù cân bằng tốc độ 2 bánh theo độ lệch xung (Closed-Loop Balance)
    // Giới hạn offset trong khoảng [-15, +15] để PWM luôn nằm trong vùng an toàn [178..215]
    long diffTicks = curL - curR;
    int balanceOffset = constrain((int)(diffTicks * 0.10f), -15, 15);

    int pwmLeft  = constrain(testPwm - balanceOffset, 178, 235);
    int pwmRight = constrain(testPwm + balanceOffset, 178, 235);

    setMotors(pwmLeft, pwmRight);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 150) {
      lastPrint = millis();
      Serial.printf(">> Đã đi: %5.2f m / %.2f m | Ticks:[L:%5ld R:%5ld] (Lệch:%+3ld) | PWM:[%d, %d]\n",
                    currentDist, targetMeters, curL, curR, diffTicks, pwmLeft, pwmRight);
    }
    delay(10);
  }

  stopMotors();
  long finalL, finalR;
  getCalibratedTicks(finalL, finalR);
  float finalDist = (float)(labs(finalL) + labs(finalR)) / 2.0f / ticksPerMeter;

  Serial.println("\n-------------------------------------------------------------------------");
  Serial.printf(">> [ĐÃ DỪNG XE TẠI ĐÍCH] Quãng đường thực đo: %.3f mét\n", finalDist);
  Serial.printf(">> Tổng số xung: Bánh Trái = %+ld | Bánh Phải = %+ld ticks\n", finalL, finalR);
  Serial.printf(">> Lệch giữa 2 bánh: %ld xung\n", labs(finalL - finalR));
  Serial.println("-------------------------------------------------------------------------\n");
}

void printHelp() {
  Serial.println("\n-------------------------------------------------------------------------");
  Serial.println("  BẢNG LỆNH KIỂM THỬ ENCODER (Gõ vào Serial Monitor):");
  Serial.println("    [t]       : Chạy thẳng tự động 1.5 mét rồi dừng");
  Serial.println("    [1]       : Chạy riêng bánh TRÁI 1.5s để xem xung");
  Serial.println("    [2]       : Chạy riêng bánh PHẢI 1.5s để xem xung");
  Serial.println("    [r]       : Reset toàn bộ xung Encoder về 0");
  Serial.println("    [invEL]   : Đảo dấu xung Bánh Trái");
  Serial.println("    [invER]   : Đảo dấu xung Bánh Phải");
  Serial.println("    [swapE]   : Đổi chỗ Encoder Trái <-> Phải");
  Serial.println("    [cal=1650]: Đặt hệ số Ticks/Mét");
  Serial.println("    [s=195]   : Cài đặt tốc độ PWM chạy test");
  Serial.println("    [h]       : In lại hướng dẫn này");
  Serial.println("-------------------------------------------------------------------------\n");
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("t")) {
    runDistanceTest(1.5f);
  } else if (line.equalsIgnoreCase("1")) {
    testSingleWheel(1);
  } else if (line.equalsIgnoreCase("2")) {
    testSingleWheel(2);
  } else if (line.equalsIgnoreCase("r")) {
    resetEncoderTicks();
    Serial.println(">> [ENCODER] Đã reset bộ đếm xung về 0.");
  } else if (line.equalsIgnoreCase("invEL")) {
    invertEncLeft = !invertEncLeft;
    Serial.printf(">> [CẤU HÌNH] Đảo dấu xung Trái: %s\n", invertEncLeft ? "BẬT" : "TẮT");
  } else if (line.equalsIgnoreCase("invER")) {
    invertEncRight = !invertEncRight;
    Serial.printf(">> [CẤU HÌNH] Đảo dấu xung Phải: %s\n", invertEncRight ? "BẬT" : "TẮT");
  } else if (line.equalsIgnoreCase("swapE")) {
    swapEncoders = !swapEncoders;
    Serial.printf(">> [CẤU HÌNH] Đổi vị trí Encoder Trái/Phải: %s\n", swapEncoders ? "BẬT" : "TẮT");
  } else if (line.startsWith("cal=")) {
    ticksPerMeter = line.substring(4).toFloat();
    Serial.printf(">> [CẤU HÌNH] Đã cập nhật TicksPerMeter = %.2f\n", ticksPerMeter);
  } else if (line.startsWith("s=")) {
    testPwm = constrain(line.substring(2).toInt(), 175, 240);
    Serial.printf(">> [CẤU HÌNH] Đã cập nhật Test PWM = %d\n", testPwm);
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
      Serial.println("\n[NÚT BOOT ĐƯỢC BẤM] -> Khởi động bài test chạy thẳng 1.5 mét!");
      runDistanceTest(1.5f);
    }
  }
  lastButtonState = reading;
}

void setup() {
  Serial.begin(115200);
  delay(600);

  initMotors();
  initEncoders();
  resetEncoderTicks();

  Serial.println("\n=========================================================================");
  Serial.println("    AGV_VER2 - BƯỚC 2: KIỂM THỬ ENCODER HALL & CÂN BẰNG TỐC ĐỘ 2 BÁNH");
  Serial.println("=========================================================================");
  Serial.printf("Cấu hình: Ticks/Mét = %.1f | PWM Test = %d\n", ticksPerMeter, testPwm);
  Serial.println(">> Đếm ngược 4 giây -> Xe tự động chạy thẳng đúng 1.5 mét rồi dừng!");
  Serial.println(">> Hoặc kê bánh lên gõ '1' để test bánh Trái, gõ '2' để test bánh Phải!");
  printHelp();

  for (int i = 4; i >= 1; i--) {
    Serial.printf(">> %d...\n", i);
    delay(1000);
  }

  runDistanceTest(1.5f);

  lastCalcTimeMs = millis();
}

void loop() {
  checkBootButton();
  handleSerialCommands();

  unsigned long now = millis();
  if (now - lastCalcTimeMs >= 100) {
    float dt = (now - lastCalcTimeMs) / 1000.0f;
    lastCalcTimeMs = now;

    long curL, curR;
    getCalibratedTicks(curL, curR);

    long dTicksL = curL - prevTicksL;
    long dTicksR = curR - prevTicksR;
    prevTicksL = curL;
    prevTicksR = curR;

    currentSpeedL_cm_s   = ((float)dTicksL / ticksPerMeter / dt) * 100.0f;
    currentSpeedR_cm_s   = ((float)dTicksR / ticksPerMeter / dt) * 100.0f;
    currentSpeedCar_cm_s = (currentSpeedL_cm_s + currentSpeedR_cm_s) / 2.0f;
    distanceCar_m        = (float)(labs(curL) + labs(curR)) / 2.0f / ticksPerMeter;

    Serial.printf("[ENCODER LIVE] Ticks:[L:%+6ld R:%+6ld] | S:%5.2fm | V:[L:%4.1f R:%4.1f cm/s]\n",
                  curL, curR, distanceCar_m, currentSpeedL_cm_s, currentSpeedR_cm_s);
  }
}
