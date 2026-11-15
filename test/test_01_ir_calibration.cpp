#include <Arduino.h>
#include "soc/usb_serial_jtag_reg.h"
#include "driver/gpio.h"

/*
 * ===================================================================================
 * BƯỚC 1.1: HIỆU CHỈNH & CHẨN ĐOÁN THANH DÒ LINE 5 MẮT TCRT5000 (AGV_Ver2)
 * -----------------------------------------------------------------------------------
 * Sơ đồ đấu nối mặc định trên ESP32-S3 Extension Board:
 *   - L2 (Trái ngoài cùng) : GPIO 48  (Trọng số w = -2.0)
 *   - L1 (Trái giữa)       : GPIO 47  (Trọng số w = -1.0)
 *   - C  (Giữa tâm vạch)   : GPIO 21  (Trọng số w =  0.0)
 *   - R1 (Phải giữa)       : GPIO 20  (Trọng số w = +1.0) [Dự phòng: GPIO 45]
 *   - R2 (Phải ngoài cùng) : GPIO 19  (Trọng số w = +2.0)
 *
 * Tính năng độc quyền:
 *   1. Tự động vô hiệu hóa khối USB-Serial/JTAG PHY trên GPIO 19, 20 để đọc Digital chuẩn.
 *   2. Đồ họa Serial trực quan: Thanh đo cảm biến + Thanh trượt định vị tâm vạch.
 *   3. Phím tắt tương tác thời gian thực qua Serial Monitor:
 *        'i' : Đảo mức logic (Đen = 1 <-> Đen = 0)
 *        's' : Đảo ngược thứ tự mắt (Trái <-> Phải) nếu bẹ dây cắm ngược
 *        'a' : Hoán đổi chân R1 giữa GPIO 20 và GPIO 45
 *        'r' : Reset bộ đếm thống kê
 *        'h' : In lại bảng trợ giúp
 * ===================================================================================
 */

// Định nghĩa chân GPIO
int pinL2     = 48;
int pinL1     = 47;
int pinC      = 21;
int pinR1     = 20;
int pinR2     = 19;
const int PIN_R1_ALT = 45;

// Cấu hình linh hoạt qua Serial
bool invertLogic    = false; // false: HIGH (1) khi gặp vạch đen; true: LOW (0) khi gặp vạch đen
bool swapLeftRight  = false; // true: Lật ngược thứ tự L2 <-> R2, L1 <-> R1
bool useAltPinR1    = false; // true: Dùng GPIO 45 thay cho GPIO 20

// Bộ đếm thống kê kiểm tra sức khỏe cảm biến (Health Check)
unsigned long sampleCount = 0;
unsigned long triggerCount[5] = {0, 0, 0, 0, 0};

void printHelp() {
  Serial.println("\n-------------------------------------------------------------");
  Serial.println("  BẢNG PHÍM TẮT ĐIỀU CHỈNH TRỰC TIẾP (Gõ vào Serial Monitor):");
  Serial.println("    [i] : Đảo mức logic vạch đen (Hiện tại: " + String(invertLogic ? "LOW/0" : "HIGH/1") + ")");
  Serial.println("    [s] : Đảo chiều Trái <-> Phải (Hiện tại: " + String(swapLeftRight ? "ĐÃ ĐẢO" : "CHUẨN") + ")");
  Serial.println("    [a] : Đổi chân R1 (Hiện tại: GPIO " + String(useAltPinR1 ? PIN_R1_ALT : pinR1) + ")");
  Serial.println("    [r] : Đặt lại bộ đếm thống kê mẫu");
  Serial.println("    [h] : In lại trợ giúp này");
  Serial.println("-------------------------------------------------------------\n");
}

void configurePins() {
  // 1. Giải phóng GPIO 19 và GPIO 20 khỏi khối USB-Serial-JTAG PHY của ESP32-S3
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

void setup() {
  Serial.begin(115200);
  delay(800);

  configurePins();

  Serial.println("\n=========================================================================");
  Serial.println("    AGV_VER2 - BƯỚC 1.1: HIỆU CHỈNH & KIỂM THỬ CẢM BIẾN DÒ LINE TCRT5000");
  Serial.println("=========================================================================");
  Serial.printf("Cấu hình chân: L2=%d | L1=%d | C=%d | R1=%d | R2=%d\n",
                pinL2, pinL1, pinC, useAltPinR1 ? PIN_R1_ALT : pinR1, pinR2);
  printHelp();
  delay(1200);
}

void handleSerialCommand() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  if (cmd == 'i' || cmd == 'I') {
    invertLogic = !invertLogic;
    Serial.printf("\n[CẤU HÌNH] Đã chuyển mức logic vạch đen sang: %s\n",
                  invertLogic ? "LOW (0)" : "HIGH (1)");
  } else if (cmd == 's' || cmd == 'S') {
    swapLeftRight = !swapLeftRight;
    Serial.printf("\n[CẤU HÌNH] Thứ tự mắt cảm biến: %s\n",
                  swapLeftRight ? "ĐÃ ĐẢO CHIỀU (R2..L2)" : "CHUẨN (L2..R2)");
  } else if (cmd == 'a' || cmd == 'A') {
    useAltPinR1 = !useAltPinR1;
    Serial.printf("\n[CẤU HÌNH] Chân đọc mắt R1 đã đổi thành: GPIO %d\n",
                  useAltPinR1 ? PIN_R1_ALT : pinR1);
  } else if (cmd == 'r' || cmd == 'R') {
    sampleCount = 0;
    for (int i = 0; i < 5; i++) triggerCount[i] = 0;
    Serial.println("\n[THỐNG KÊ] Đã reset bộ đếm mẫu.");
  } else if (cmd == 'h' || cmd == 'H') {
    printHelp();
  }
}

void loop() {
  handleSerialCommand();

  int curR1Pin = useAltPinR1 ? PIN_R1_ALT : pinR1;

  // 1. Đọc tín hiệu từ phần cứng
  int rawL2 = digitalRead(pinL2);
  int rawL1 = digitalRead(pinL1);
  int rawC  = digitalRead(pinC);
  int rawR1 = digitalRead(curR1Pin);
  int rawR2 = digitalRead(pinR2);

  // 2. Xử lý logic theo cấu hình
  int sL2 = invertLogic ? !rawL2 : rawL2;
  int sL1 = invertLogic ? !rawL1 : rawL1;
  int sC  = invertLogic ? !rawC  : rawC;
  int sR1 = invertLogic ? !rawR1 : rawR1;
  int sR2 = invertLogic ? !rawR2 : rawR2;

  // Nếu bẹ dây cắm ngược đầu, đảo lại vị trí mắt
  if (swapLeftRight) {
    int tL2 = sL2, tL1 = sL1;
    sL2 = sR2;
    sL1 = sR1;
    sR1 = tL1;
    sR2 = tL2;
  }

  // 3. Thống kê mẫu
  sampleCount++;
  if (sL2) triggerCount[0]++;
  if (sL1) triggerCount[1]++;
  if (sC)  triggerCount[2]++;
  if (sR1) triggerCount[3]++;
  if (sR2) triggerCount[4]++;

  // 4. Tính toán sai số lệch tâm e [-2.0 .. +2.0]
  int activeCount = sL2 + sL1 + sC + sR1 + sR2;
  float lineError = 0.0f;
  const char* desc = "";

  if (activeCount == 0) {
    desc = "⚠️ MẤT VẠCH (00000)";
    lineError = 0.0f;
  } else if (activeCount == 5) {
    desc = "🛑 VẠCH DỪNG / NGÃ TƯ (11111)";
    lineError = 0.0f;
  } else if (activeCount >= 3 && sL1 && sC && sR1) {
    desc = "➕ VẠCH CẮT TRUNG TÂM";
    lineError = 0.0f;
  } else {
    // Trọng số: L2=-2, L1=-1, C=0, R1=+1, R2=+2
    lineError = (float)(-2 * sL2 + -1 * sL1 + 0 * sC + 1 * sR1 + 2 * sR2) / (float)activeCount;
    if (lineError <= -1.4f)      desc = "◄◄ LỆCH TRÁI NHIỀU";
    else if (lineError < -0.2f)  desc = " ◄ LỆCH TRÁI NHẸ";
    else if (lineError <= 0.2f)  desc = " ✔ CHÍNH GIỮA TÂM";
    else if (lineError < 1.4f)   desc = " ► LỆCH PHẢI NHẸ";
    else                         desc = "►► LỆCH PHẢI NHIỀU";
  }

  // 5. Thanh trượt định vị trực quan độ lệch
  // Biểu diễn dải từ -2.0 đến +2.0 thành 21 ký tự
  char slider[26];
  for (int i = 0; i < 21; i++) slider[i] = '-';
  slider[10] = '|'; // Điểm giữa tâm
  int markerPos = constrain((int)round((lineError + 2.0f) * 5.0f), 0, 20);
  if (activeCount > 0) slider[markerPos] = 'O';
  slider[21] = '\0';

  // 6. In kết quả dạng bảng và trực quan ra Serial Monitor
  Serial.printf("[%s|%s|%s|%s|%s] e=%+5.2f [%s] Raw:[%d%d%d%d%d] %s\n",
                sL2 ? "██" : "  ",
                sL1 ? "██" : "  ",
                sC  ? "██" : "  ",
                sR1 ? "██" : "  ",
                sR2 ? "██" : "  ",
                lineError,
                slider,
                rawL2, rawL1, rawC, rawR1, rawR2,
                desc);

  // Định kỳ mỗi 100 chu kỳ (khoảng 8 giây), in tóm tắt độ nhạy từng mắt
  if (sampleCount % 100 == 0) {
    Serial.println("  -------------------------------------------------------------");
    Serial.printf("  [SỨC KHỎE CẢM BIẾN] Nhận vạch: L2:%lu%% | L1:%lu%% | C:%lu%% | R1:%lu%% | R2:%lu%%\n",
                  triggerCount[0] * 100 / sampleCount,
                  triggerCount[1] * 100 / sampleCount,
                  triggerCount[2] * 100 / sampleCount,
                  triggerCount[3] * 100 / sampleCount,
                  triggerCount[4] * 100 / sampleCount);
    Serial.println("  -------------------------------------------------------------");
  }

  delay(80);
}
