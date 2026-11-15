#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_arduino_version.h>

/*
 * ===================================================================================
 * BƯỚC 5: CHƯƠNG TRÌNH KIỂM THỬ CHUYÊN DỤNG CALIB ENCODER 1 MÉT (ZERO-SLIP)
 * -----------------------------------------------------------------------------------
 * ĐẶC TẢ THỰC NGHIỆM:
 *   - Mục đích: Đo đạc số xung Encoder chính xác tuyệt đối khi đẩy xe 1 mét (hoặc 1.51m).
 *   - Vị trí tính toán: TÂM TRỤC 2 BÁNH XE CHỦ ĐỘNG.
 *   - Chế độ an toàn: Toàn bộ ngõ ra động cơ L298N bị khóa ở mức 0V (không có dòng điện,
 *     không bị phanh hãm điện từ), giúp bánh xe lăn tự do hoàn toàn khi đẩy tay.
 *   - Kết nối: Gửi dữ liệu số xung lên Server Calib chuyên dụng (Cổng 5001).
 *
 * PHƯƠNG THỨC RESET VỀ 0:
 *   1. Bấm nút "RESET VỀ 0" trên Web Dashboard (http://localhost:5001).
 *   2. Bấm nút cứng BOOT (GPIO 0) trên bo ESP32.
 *   3. Gõ phím 'r' hoặc '0' trên Serial Monitor (115200 baud).
 * ===================================================================================
 */

// --- 1. CẤU HÌNH KẾT NỐI WI-FI & SERVER CALIB ---
const char* wifi_ssid        = "LAU 3";
const char* wifi_pass        = "777777777";
// Địa chỉ Server Calib cổng riêng 5001
String      serverUrl        = "http://192.168.1.6:5001/api/encoder_data";

// --- 2. SƠ ĐỒ CHÂN PHẦN CỨNG ---
// Động cơ L298N (Khóa an toàn để thả trôi tự do)
#define L298N_ENA_PIN        6
#define L298N_IN1_PIN        7
#define L298N_IN2_PIN        15
#define L298N_IN3_PIN        16
#define L298N_IN4_PIN        17
#define L298N_ENB_PIN        18
#define BUTTON_BOOT_PIN      0

// Encoder Hall (4 ngắt kênh A & B)
#define ENC_L_A_PIN          10
#define ENC_L_B_PIN          11
#define ENC_R_A_PIN          12
#define ENC_R_B_PIN          13

// Cờ đảo chiều / hoán đổi Encoder
bool invertEncLeft           = false;
bool invertEncRight          = false; // Đã kiểm nghiệm: false để cả 2 bánh cùng tăng (+) khi tiến
bool swapEncoders            = true;  // Hoán đổi đúng vị trí bánh Trái / Phải

// Biến đếm xung ngắt
volatile long rawTicksL      = 0;
volatile long rawTicksR      = 0;

// Biến giao tiếp lệnh với Server
volatile int lastAckId       = 0;
volatile bool requestReset   = false;

// ===================================================================================
// TRÌNH PHỤC VỤ NGẮT ENCODER (4X QUAD-EDGE)
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
  // Bánh phải đối xứng cơ học: so sánh != để cùng tăng khi tiến
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

  if (swapEncoders) { outL = tR; outR = tL; }
  else              { outL = tL; outR = tR; }
}

void resetTicks() {
  noInterrupts();
  rawTicksL = 0;
  rawTicksR = 0;
  interrupts();
  Serial.println("\n>> [ĐÃ RESET SỐ XUNG VỀ 0] << Bắt đầu đẩy xe 1 mét!");
}

// ===================================================================================
// CORE 0: TÁC VỤ WI-FI & TRUYỀN DỮ LIỆU LÊN SERVER CỔNG 5001 (10Hz = 100ms)
// ===================================================================================
void telemetryTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  Serial.printf(">> [WI-FI]: Đang kết nối tới %s...\n", wifi_ssid);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 25) {
    delay(400);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ [WI-FI KẾT NỐI THÀNH CÔNG] IP ESP32 = %s\n", WiFi.localIP().toString().c_str());
    Serial.printf(">> Đang gửi số xung Encoder lên: %s\n\n", serverUrl.c_str());
  } else {
    Serial.println("\n⚠️ [WI-FI]: Không kết nối được Wi-Fi -> Vẫn có thể quan sát trực tiếp qua Serial Monitor!");
  }

  HTTPClient http;
  http.setTimeout(1200);

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      long curL, curR;
      getCalibratedTicks(curL, curR);

      String payload = "{";
      payload += "\"ticks_l\":" + String(curL) + ",";
      payload += "\"ticks_r\":" + String(curR) + ",";
      payload += "\"ack_id\":" + String(lastAckId);
      payload += "}";

      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");
      int httpCode = http.POST(payload);

      if (httpCode == 200) {
        String resp = http.getString();
        int idIdx = resp.indexOf("\"cmd_id\":");
        int incomingId = 0;
        if (idIdx >= 0) {
          incomingId = resp.substring(idIdx + 9).toInt();
        }

        if (incomingId > 0 && incomingId != lastAckId) {
          lastAckId = incomingId;
          if (resp.indexOf("RESET") >= 0) {
            resetTicks();
          }
        }
      }
      http.end();
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Gửi chu kỳ 100ms (10Hz) cực mượt
  }
}

// ===================================================================================
// SETUP
// ===================================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Khóa ngắt động cơ để thả trôi bánh xe hoàn toàn (Free-wheeling)
  pinMode(L298N_ENA_PIN, OUTPUT);
  pinMode(L298N_IN1_PIN, OUTPUT);
  pinMode(L298N_IN2_PIN, OUTPUT);
  pinMode(L298N_IN3_PIN, OUTPUT);
  pinMode(L298N_IN4_PIN, OUTPUT);
  pinMode(L298N_ENB_PIN, OUTPUT);
  digitalWrite(L298N_ENA_PIN, LOW);
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, LOW);
  digitalWrite(L298N_ENB_PIN, LOW);

  // Nút BOOT cứng
  pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

  // Khởi tạo ngắt Encoder 4 kênh
  pinMode(ENC_L_A_PIN, INPUT_PULLUP);
  pinMode(ENC_L_B_PIN, INPUT_PULLUP);
  pinMode(ENC_R_A_PIN, INPUT_PULLUP);
  pinMode(ENC_R_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A_PIN), isrEncLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B_PIN), isrEncLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A_PIN), isrEncRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B_PIN), isrEncRightB, CHANGE);

  Serial.println("\n=========================================================================");
  Serial.println("   AGV_VER2 - CHƯƠNG TRÌNH ĐO CALIB ENCODER 1 MÉT (TÂM TRỤC 2 BÁNH)");
  Serial.println("=========================================================================");
  Serial.println(">> Động cơ đang được THẢ TRÔI TỰ DO (Không bị phanh điện từ).");
  Serial.println(">> Hướng dẫn: Đặt TÂM 2 BÁNH tại vạch 0.00m -> Bấm 'r' -> Đẩy xe tới 1.00m.");
  Serial.println("=========================================================================\n");

  // Chạy tác vụ mạng trên Core 0
  xTaskCreatePinnedToCore(
    telemetryTask,
    "EncoderTelemetry",
    6144,
    NULL,
    1,
    NULL,
    0
  );
}

// ===================================================================================
// LOOP CHÍNH (CORE 1: GIÁM SÁT THỜI GIAN THỰC & NHẬN LỆNH SERIAL / NÚT BẤM)
// ===================================================================================
void loop() {
  // 1. Kiểm tra phím bấm Serial
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'r' || ch == '0' || ch == ' ') {
      resetTicks();
    }
  }

  // 2. Kiểm tra nút BOOT cứng trên bo ESP32
  static unsigned long lastBtnMs = 0;
  static int lastBtnState = HIGH;
  int btn = digitalRead(BUTTON_BOOT_PIN);
  if (btn == LOW && lastBtnState == HIGH && (millis() - lastBtnMs > 300)) {
    lastBtnMs = millis();
    resetTicks();
  }
  lastBtnState = btn;

  // 3. In thông số Serial Monitor định kỳ mỗi 150ms để quan sát trực tiếp
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs >= 150) {
    lastPrintMs = millis();
    long curL, curR;
    getCalibratedTicks(curL, curR);
    float avg = (curL + curR) / 2.0f;
    long diff = curL - curR;

    Serial.printf("[ENCODER LIVE] Trái: %+6ld | Phải: %+6ld | TB: %+7.1f | Lệch (L-R): %+4ld\n",
                  curL, curR, avg, diff);
  }

  delay(10);
}
