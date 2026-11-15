#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>

/*
 * ===================================================================================
 * BƯỚC 7: KIỂM THỬ CẢM BIẾN DÒNG/ÁP INA226 (GPIO 8 & 9) & WEBSERVER QUẢN LÝ NĂNG LƯỢNG
 * -----------------------------------------------------------------------------------
 * ĐẶC TẢ KIỂM THỬ TOÀN DIỆN CHO MODULE INA226 MỚI:
 *   1. Chẩn đoán phần cứng điện áp tĩnh trên GPIO 8 (SDA) và GPIO 9 (SCL).
 *   2. Quét I2C Bus, nhận diện chính xác Chip ID của Texas Instruments:
 *      - Manufacturer ID (0xFE) == 0x5449 ('TI')
 *      - Die ID (0xFF) == 0x2260 (Chuẩn INA226)
 *   3. Kiểm tra độ toàn vẹn tín hiệu: Chạy Stress Test 100 gói đo tỷ lệ mất gói & độ trễ.
 *   4. Tích phân năng lượng thời gian thực: Tính Dung lượng (mAh) và Điện năng (Wh).
 *   5. KẾT NỐI WI-FI & TRUYỀN DỮ LIỆU ĐẾN SERVER NĂNG LƯỢNG RIÊNG (PORT 5002):
 *      - Web Dashboard hiển thị: % Pin, Điện áp (V), Dòng điện (A), Công suất (W), Đồ thị sóng.
 *      - Hỗ trợ UDP Discovery (Port 5006) để tự động nhận diện IP máy tính chạy server.
 *   6. Hoạt động độc lập kép: Có thể xem qua Serial Monitor (115200 baud) HOẶC Web Dashboard.
 *
 * SƠ ĐỒ ĐẤU DÂY (ESP32-S3 <-> INA226):
 *   - VCC  <-> Cọc 3.3V (hoặc 5V) của ESP32-S3
 *   - GND  <-> Cọc GND của ESP32-S3
 *   - SDA  <-> GPIO 8 (Có thể đảo sang GPIO 9 qua lệnh 'swap')
 *   - SCL  <-> GPIO 9 (Có thể đảo sang GPIO 8 qua lệnh 'swap')
 *
 * ĐẤU DÂY ĐO NĂNG LƯỢNG NGUỒN TẢI:
 *   - VIN+ (VBUS) : Cực Dương (+) Nguồn (Pin LiPo / Ắc quy / Nguồn tổ ong)
 *   - VIN-        : Cực Dương (+) cấp vào Tải (Mạch công suất L298N / Động cơ)
 *   - Cực Âm (-) nguồn nối chung Mass GND với ESP32 và Tải.
 * ===================================================================================
 */

// --- 1. CẤU HÌNH KẾT NỐI WI-FI & SERVER QUẢN LÝ NĂNG LƯỢNG ---
const char* wifi_ssid        = "LAU 3";
const char* wifi_pass        = "777777777";
String      powerServerUrl   = "http://192.168.1.31:5002/api/power_telemetry";
WiFiUDP     udpDiscovery;
#define     UDP_POWER_PORT   5006

// --- 2. CẤU HÌNH CHÂN I2C MẶC ĐỊNH ---
int pinSDA = 8;
int pinSCL = 9;
const int BUTTON_BOOT_PIN = 0;

// --- THANH GHI CHUẨN INA226 ---
#define INA226_REG_CONFIG       0x00
#define INA226_REG_SHUNTVOLTAGE 0x01
#define INA226_REG_BUSVOLTAGE   0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIBRATION  0x05
#define INA226_REG_MASKENABLE   0x06
#define INA226_REG_ALERTLIMIT   0x07
#define INA226_REG_MANUF_ID     0xFE  // Kỳ vọng: 0x5449 ('T','I')
#define INA226_REG_DIE_ID       0xFF  // Kỳ vọng: 0x2260 (INA226)

const uint8_t POSSIBLE_ADDRS[] = {0x40, 0x41, 0x44, 0x45, 0x42, 0x43, 0x46, 0x47};
const int NUM_POSSIBLE_ADDRS = sizeof(POSSIBLE_ADDRS) / sizeof(POSSIBLE_ADDRS[0]);

uint8_t inaAddress = 0x40;
bool inaDetected = false;
bool isPinSwapped = false;
uint32_t i2cClockSpeed = 100000;

// Cấu hình điện trở Shunt trên module (Mặc định R100 = 0.1 Ohm)
float rShunt = 0.100f; // Đơn vị: Ohm

// Số liệu đo đạc chia sẻ giữa 2 nhân
volatile float liveBusVoltageV   = 0.0f;
volatile float liveShuntVoltagemV = 0.0f;
volatile float liveCurrentA      = 0.0f;
volatile float liveCurrentmA     = 0.0f;
volatile float livePowerW        = 0.0f;
volatile float liveEnergyMah     = 0.0f;
volatile float liveEnergyWh      = 0.0f;

unsigned long lastCalcTimeUs     = 0;
unsigned long lastSerialPrintTime= 0;
unsigned long lastWaitPrintTime  = 0;
int lastButtonState = HIGH;
unsigned long lastDebounceTime   = 0;

// --- NGUYÊN MẪU HÀM ---
void recoverI2CBus(int sda, int scl);
void checkPinElectricalHealth();
void initI2C();
bool scanI2CBus(bool verbose = true);
bool checkINA226Identity(uint8_t addr);
bool writeRegister16(uint8_t addr, uint8_t reg, uint16_t value);
bool readRegister16(uint8_t addr, uint8_t reg, uint16_t &value);
bool configureINA226();
void runDataTransmissionStressTest(int cycles = 100);
void dumpAllRegisters();
void updateLivePowerReading();
void printSerialTelemetry();
void handleSerialCommands();
void checkBootButton();
void printHelpMenu();
void runFullDiagnosticSequence();
void powerTelemetryTask(void *pvParameters);

// ===================================================================================
// HÀM SETUP KHỞI ĐỘNG HỆ THỐNG
// ===================================================================================
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

  for (int i = 0; i < 3; i++) {
    delay(250);
    Serial.println(">> [ESP32-S3 KHỞI ĐỘNG] - Đang kết nối Serial Monitor...");
    Serial.flush();
  }

  // Chạy bài chẩn đoán phần cứng ban đầu
  runFullDiagnosticSequence();

  // Khởi động Task truyền Wi-Fi Telemetry lên Server năng lượng (Core 0)
  xTaskCreatePinnedToCore(powerTelemetryTask, "PowerTelemetry", 6144, NULL, 1, NULL, 0);
}

// ===================================================================================
// VÒNG LẶP CHÍNH (LOOP) - CHẠY TRÊN CORE 1
// ===================================================================================
void loop() {
  checkBootButton();
  handleSerialCommands();

  unsigned long now = millis();

  if (inaDetected) {
    // Đọc cảm biến và tích phân năng lượng liên tục (20Hz = 50ms)
    updateLivePowerReading();

    // In thông số lên Serial Monitor định kỳ mỗi 500ms
    if (now - lastSerialPrintTime >= 500) {
      lastSerialPrintTime = now;
      printSerialTelemetry();
    }
  } else {
    // Thông báo định kỳ khi chưa nhận được cảm biến
    if (now - lastWaitPrintTime >= 1500) {
      lastWaitPrintTime = now;
      int stateSDA = digitalRead(pinSDA);
      int stateSCL = digitalRead(pinSCL);
      Serial.printf("[ĐANG DÒ TÌM INA226] SDA(GPIO %d)=%s | SCL(GPIO %d)=%s --> Chưa nhận được phản hồi ACK.\n",
                    pinSDA, (stateSDA == HIGH) ? "HIGH(3.3V)" : "LOW(0V)",
                    pinSCL, (stateSCL == HIGH) ? "HIGH(3.3V)" : "LOW(0V)");
      Serial.println("  --> Gợi ý: Gõ 'swap' để đảo chân 8 và 9, gõ 's' để quét lại, hoặc nhấn nút BOOT trên mạch!");
      Serial.flush();
    }
  }

  delay(20);
}

// ===================================================================================
// CORE 0: KẾT NỐI WI-FI & TRUYỀN SỐ LIỆU ĐẾN SERVER NĂNG LƯỢNG (PORT 5002)
// ===================================================================================
void powerTelemetryTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  Serial.printf(">> [WI-FI]: Đang kết nối tới mạng '%s'...\n", wifi_ssid);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 25) {
    vTaskDelay(pdMS_TO_TICKS(400));
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ [WI-FI THÀNH CÔNG]: IP ESP32 = %s\n", WiFi.localIP().toString().c_str());

    // Quét UDP beacon để tự tìm Server Port 5002
    udpDiscovery.begin(UDP_POWER_PORT);
    Serial.printf(">> [DÒ TÌM POWER SERVER]: Đang lắng nghe beacon UDP tại cổng %d...\n", UDP_POWER_PORT);
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1500) {
      int packetSize = udpDiscovery.parsePacket();
      if (packetSize) {
        char packetBuffer[255];
        int len = udpDiscovery.read(packetBuffer, 254);
        if (len > 0) packetBuffer[len] = 0;
        String s = String(packetBuffer);
        if (s.indexOf("INA226_POWER_SERVER") >= 0) {
          IPAddress srvIp = udpDiscovery.remoteIP();
          powerServerUrl = "http://" + srvIp.toString() + ":5002/api/power_telemetry";
          Serial.printf("🎯 [TÌM THẤY POWER SERVER]: Server IP = %s -> URL: %s\n", srvIp.toString().c_str(), powerServerUrl.c_str());
          break;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(30));
    }
  } else {
    Serial.println("\n⚠️ [WI-FI]: Không kết nối được Wi-Fi -> Chạy ở chế độ Serial Monitor độc lập.");
  }

  HTTPClient http;
  http.setTimeout(800);
  unsigned long lastUdpCheckMs = 0;

  while (true) {
    if (WiFi.status() == WL_CONNECTED && inaDetected) {
      // Định kỳ kiểm tra beacon UDP cập nhật IP máy tính
      if (millis() - lastUdpCheckMs > 4000) {
        lastUdpCheckMs = millis();
        int packetSize = udpDiscovery.parsePacket();
        if (packetSize) {
          char packetBuffer[255];
          int len = udpDiscovery.read(packetBuffer, 254);
          if (len > 0) packetBuffer[len] = 0;
          String s = String(packetBuffer);
          if (s.indexOf("INA226_POWER_SERVER") >= 0) {
            IPAddress srvIp = udpDiscovery.remoteIP();
            String newUrl = "http://" + srvIp.toString() + ":5002/api/power_telemetry";
            if (newUrl != powerServerUrl) {
              powerServerUrl = newUrl;
              Serial.printf("🎯 [CẬP NHẬT IP SERVER MỚI]: %s\n", powerServerUrl.c_str());
            }
          }
        }
      }

      String payload = "{";
      payload += "\"voltage\":" + String(liveBusVoltageV, 3) + ",";
      payload += "\"shunt_mv\":" + String(liveShuntVoltagemV, 3) + ",";
      payload += "\"current\":" + String(liveCurrentA, 3) + ",";
      payload += "\"power\":" + String(livePowerW, 3) + ",";
      payload += "\"addr\":\"0x" + String(inaAddress, HEX) + "\",";
      payload += "\"r_shunt\":" + String(rShunt, 3);
      payload += "}";

      http.begin(powerServerUrl);
      http.addHeader("Content-Type", "application/json");
      int httpCode = http.POST(payload);
      http.end();
    }
    vTaskDelay(pdMS_TO_TICKS(150)); // Gửi telemetry mỗi 150ms (~6.6Hz)
  }
}

// ===================================================================================
// CHẨN ĐOÁN PHẦN CỨNG TOÀN DIỆN CHO CẢM BIẾN MỚI
// ===================================================================================
void runFullDiagnosticSequence() {
  Serial.println("\n");
  Serial.println("===============================================================================");
  Serial.println("   BƯỚC 7: KIỂM THỬ CẢM BIẾN DÒNG/ÁP INA226 MỚI (GPIO 8 - SDA | GPIO 9 - SCL)");
  Serial.println("===============================================================================");
  Serial.printf(" [CẤU HÌNH HIỆN TẠI]: SDA = GPIO %d | SCL = GPIO %d | Tốc độ = %u kHz\n", 
                pinSDA, pinSCL, (unsigned int)(i2cClockSpeed / 1000));
  Serial.printf(" [TRỞ SHUNT MẶC ĐỊNH]: R_shunt = %.3f Ohm (Module R100)\n", rShunt);
  Serial.println("-------------------------------------------------------------------------------");

  // 1. Kiểm tra điện áp tĩnh trên chân 8 và 9
  checkPinElectricalHealth();

  // 2. Khởi tạo I2C có cơ chế giải phóng bus
  initI2C();

  // 3. Quét tìm cảm biến
  inaDetected = scanI2CBus(true);

  // 4. Nếu chưa thấy ở cấu hình mặc định, thử đảo chân 8 <-> 9
  if (!inaDetected) {
    Serial.println("\n[TỰ ĐỘNG THỬ ĐẢO CHÂN]: Đang thử hoán đổi: SDA=GPIO 9, SCL=GPIO 8...");
    pinSDA = 9;
    pinSCL = 8;
    isPinSwapped = true;
    initI2C();
    inaDetected = scanI2CBus(false);

    if (inaDetected) {
      Serial.println(">> PHÁT HIỆN THÀNH CÔNG! Module đang cắm theo: SDA=GPIO 9, SCL=GPIO 8.");
    } else {
      pinSDA = 8;
      pinSCL = 9;
      isPinSwapped = false;
      initI2C();
      Serial.println("[KẾT QUẢ]: Chưa nhận được ACK từ cảm biến trên cả 2 chiều.");
    }
  }

  // 5. Nếu phát hiện cảm biến, cấu hình và chạy test truyền nhận 100 chu kỳ
  if (inaDetected) {
    Serial.println("\n-------------------------------------------------------------------------------");
    Serial.println(">>> TIẾN HÀNH KIỂM TRA ĐỘ TOÀN VẸN TRUYỀN DỮ LIỆU (100 CHU KỲ LIÊN TỤC) <<<");
    Serial.println("-------------------------------------------------------------------------------");
    configureINA226();
    runDataTransmissionStressTest(100);
    lastCalcTimeUs = micros();
  } else {
    Serial.println("\n[LƯU Ý KHI LẮP ĐẶT CẢM BIẾN MỚI]:");
    Serial.println(" 1. Nối chân VCC vào 3.3V, chân GND vào GND của ESP32-S3.");
    Serial.println(" 2. Nối chân SDA vào GPIO 8, chân SCL vào GPIO 9.");
    Serial.println(" 3. Nếu nghi ngờ cắm ngược, gõ 'swap' trên Serial Monitor để đảo chân phần mềm.");
    Serial.println(" 4. Gõ 's' để quét lại bus bất kỳ lúc nào.");
  }

  printHelpMenu();
  Serial.println("\n[BẮT ĐẦU ĐO & TÍCH PHÂN NĂNG LƯỢNG THỜI GIAN THỰC]:");
  Serial.println("-------------------------------------------------------------------------------");
  Serial.flush();
}

void recoverI2CBus(int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
    digitalWrite(scl, LOW);
    delayMicroseconds(5);
  }
  digitalWrite(scl, HIGH);
  pinMode(scl, INPUT_PULLUP);
  delayMicroseconds(20);
}

void checkPinElectricalHealth() {
  Serial.println("\n--- [1. CHẨN ĐOÁN PHẦN CỨNG TĨNH TRÊN GPIO 8 & GPIO 9] ---");

  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  delayMicroseconds(100);

  int statePin8 = digitalRead(8);
  int statePin9 = digitalRead(9);

  Serial.printf(" - GPIO 8 (SDA): Mức điện áp tĩnh = %s (%d)\n", 
                statePin8 == HIGH ? "HIGH (3.3V) [TỐT]" : "LOW (0V) [CẢNH BÁO]", statePin8);
  Serial.printf(" - GPIO 9 (SCL): Mức điện áp tĩnh = %s (%d)\n", 
                statePin9 == HIGH ? "HIGH (3.3V) [TỐT]" : "LOW (0V) [CẢNH BÁO]", statePin9);

  if (statePin8 == HIGH && statePin9 == HIGH) {
    Serial.println(" => ĐÁNH GIÁ ĐIỆN ÁP: Hai chân GPIO 8 & 9 sẵn sàng cho bus I2C Open-Drain!");
  } else {
    Serial.println(" [!] CẢNH BÁO: Có chân bị ghì xuống LOW (nguy cơ chập mass GND hoặc chạm dây)!");
  }
  Serial.flush();
}

void initI2C() {
  Wire.end();
  delay(10);
  recoverI2CBus(pinSDA, pinSCL);
  Wire.begin(pinSDA, pinSCL, i2cClockSpeed);
  Wire.setTimeOut(25);
}

bool scanI2CBus(bool verbose) {
  if (verbose) {
    Serial.println("\n--- [2. QUÉT I2C BUS TRÊN GPIO 8 VÀ GPIO 9] ---");
    Serial.printf(" Đang quét địa chỉ từ 0x01 đến 0x7F trên SDA=%d, SCL=%d...\n", pinSDA, pinSCL);
  }

  int devicesFound = 0;
  bool foundINA = false;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      devicesFound++;
      if (verbose) {
        Serial.printf(" -> Tìm thấy thiết bị phản hồi ACK tại địa chỉ: 0x%02X", address);
      }

      for (int i = 0; i < NUM_POSSIBLE_ADDRS; i++) {
        if (address == POSSIBLE_ADDRS[i]) {
          if (verbose) Serial.print(" [Khả nghi INA226]");
          break;
        }
      }
      if (verbose) Serial.println();

      if (checkINA226Identity(address)) {
        inaAddress = address;
        foundINA = true;
        if (verbose) {
          Serial.printf("    *** XÁC NHẬN CHÍNH XÁC: Chip tại 0x%02X là INA226 từ Texas Instruments! ***\n", address);
        }
      }
    }
  }

  if (devicesFound == 0) {
    if (verbose) Serial.println(" [KẾT QUẢ]: Không tìm thấy thiết bị nào phản hồi ACK.");
    return false;
  }
  return foundINA;
}

bool checkINA226Identity(uint8_t addr) {
  uint16_t manufID = 0;
  uint16_t dieID = 0;

  bool okManuf = readRegister16(addr, INA226_REG_MANUF_ID, manufID);
  bool okDie   = readRegister16(addr, INA226_REG_DIE_ID, dieID);

  if (okManuf && okDie) {
    // Manuf ID = 0x5449 ("TI"), Die ID = 0x2260
    if (manufID == 0x5449 && dieID == 0x2260) return true;
    if (manufID == 0x5449) return true;
  }
  return false;
}

bool writeRegister16(uint8_t addr, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return (Wire.endTransmission() == 0);
}

bool readRegister16(uint8_t addr, uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    err = Wire.endTransmission(true);
    if (err != 0) return false;
  }

  uint8_t bytesReceived = Wire.requestFrom(addr, (uint8_t)2);
  if (bytesReceived < 2) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  value = ((uint16_t)msb << 8) | (uint16_t)lsb;
  return true;
}

bool configureINA226() {
  uint16_t configVal = 0x4127; // Averaging 1, VBUS/VSHUNT 1.1ms, Continuous

  if (!writeRegister16(inaAddress, INA226_REG_CONFIG, configVal)) {
    Serial.println(" [LỖI] Không thể ghi thanh ghi cấu hình INA226!");
    return false;
  }

  float currentLSB = 0.0001f;
  uint16_t calVal = (uint16_t)(0.00512f / (currentLSB * rShunt));
  if (calVal == 0) calVal = 512;
  writeRegister16(inaAddress, INA226_REG_CALIBRATION, calVal);

  return true;
}

void runDataTransmissionStressTest(int cycles) {
  Serial.printf(">> BẮT ĐẦU BÀI TEST TRUYỀN NHẬN %d CHU KỲ VỚI INA226 (0x%02X)...\n", cycles, inaAddress);

  int successes = 0;
  int failures = 0;
  unsigned long totalTimeMicros = 0;

  for (int i = 1; i <= cycles; i++) {
    unsigned long t0 = micros();
    uint16_t manufID = 0;
    uint16_t dieID = 0;

    bool ok1 = readRegister16(inaAddress, INA226_REG_MANUF_ID, manufID);
    bool ok2 = readRegister16(inaAddress, INA226_REG_DIE_ID, dieID);
    unsigned long tElapsed = micros() - t0;
    totalTimeMicros += tElapsed;

    if (ok1 && ok2 && manufID == 0x5449 && dieID == 0x2260) {
      successes++;
    } else {
      failures++;
    }

    if (i % 25 == 0 || i == cycles) {
      Serial.printf("   - Đã kiểm tra: %3d/%d gói | Thành công: %3d | Lỗi: %d | Độ trễ 1 gói: %lu us\n",
                    i, cycles, successes, failures, tElapsed);
    }
    delay(5);
  }

  float successRate = ((float)successes / (float)cycles) * 100.0f;
  float avgLatency = (float)totalTimeMicros / (float)cycles;

  Serial.println("\n----------------- [KẾT QUẢ ĐÁNH GIÁ ĐƯỜNG TRUYỀN GPIO 8 & GPIO 9] -----------------");
  Serial.printf(" * Tổng số gói kiểm tra:       %d gói\n", cycles);
  Serial.printf(" * Số gói thành công (ACK):    %d gói\n", successes);
  Serial.printf(" * Số gói lỗi (Packet Loss):   %d gói (%.2f%%)\n", failures, (100.0f - successRate));
  Serial.printf(" * Tỷ lệ thành công:           %.2f %%\n", successRate);
  Serial.printf(" * Độ trễ phản hồi trung bình: %.1f micro-giây (us)\n", avgLatency);

  if (successRate >= 99.0f) {
    Serial.println(" => KẾT LUẬN: [XUẤT SẮC - HOÀN HẢO] 🌟 Module mới & GPIO 8, 9 giao tiếp hoàn hảo!");
  } else if (successRate >= 90.0f) {
    Serial.println(" => KẾT LUẬN: [ĐẠT YÊU CẦU - CÓ NHIỄU NHẸ] ⚠️ Nên gắn thêm trở kéo 4.7k.");
  } else {
    Serial.println(" => KẾT LUẬN: [KÉM - ĐƯỜNG TRUYỀN KHÔNG ỔN ĐỊNH] ❌ Kiểm tra lại tiếp xúc dây.");
  }
  Serial.println("-----------------------------------------------------------------------------------");
  Serial.flush();
}

void dumpAllRegisters() {
  Serial.println("\n================ [BẢNG THANH GHI NỘI TẠI CỦA INA226] ================");
  Serial.printf(" Địa chỉ cảm biến: 0x%02X | SDA: GPIO %d | SCL: GPIO %d\n", inaAddress, pinSDA, pinSCL);
  Serial.println("---------------------------------------------------------------------");

  struct RegInfo { uint8_t addr; const char *name; const char *desc; };
  RegInfo regs[] = {
    {INA226_REG_CONFIG,       "0x00: Config       ", "Cấu hình trung bình mẫu & chế độ"},
    {INA226_REG_SHUNTVOLTAGE, "0x01: Shunt Volt   ", "Sụt áp trên trở Shunt (LSB: 2.5 uV)"},
    {INA226_REG_BUSVOLTAGE,   "0x02: Bus Volt     ", "Điện áp nguồn tải (LSB: 1.25 mV)"},
    {INA226_REG_POWER,        "0x03: Power        ", "Công suất tính bởi chip"},
    {INA226_REG_CURRENT,      "0x04: Current      ", "Dòng điện tính bởi chip"},
    {INA226_REG_CALIBRATION,  "0x05: Calibration  ", "Hệ số hiệu chuẩn dòng điện"},
    {INA226_REG_MASKENABLE,   "0x06: Mask/Enable  ", "Cấu hình cảnh báo Alert"},
    {INA226_REG_ALERTLIMIT,   "0x07: Alert Limit  ", "Ngưỡng giới hạn kích hoạt Alert"},
    {INA226_REG_MANUF_ID,     "0xFE: Manuf ID     ", "Mã hãng sản xuất (0x5449 = 'TI')"},
    {INA226_REG_DIE_ID,       "0xFF: Die ID       ", "Mã chip INA226 (0x2260)"}
  };

  for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
    uint16_t val = 0;
    bool ok = readRegister16(inaAddress, regs[i].addr, val);
    if (ok) Serial.printf(" %-23s | 0x%04X  | %-7d | %s\n", regs[i].name, val, (int16_t)val, regs[i].desc);
    else Serial.printf(" %-23s | [LỖI ĐỌC] | ------- | Không nhận được ACK\n", regs[i].name);
  }
  Serial.println("=====================================================================\n");
  Serial.flush();
}

// ===================================================================================
// TÍNH TOÁN ĐO ĐẠC & TÍCH PHÂN NĂNG LƯỢNG (mAh & Wh)
// ===================================================================================
void updateLivePowerReading() {
  uint16_t rawBus = 0;
  uint16_t rawShunt = 0;

  bool okBus = readRegister16(inaAddress, INA226_REG_BUSVOLTAGE, rawBus);
  bool okShunt = readRegister16(inaAddress, INA226_REG_SHUNTVOLTAGE, rawShunt);

  if (!okBus || !okShunt) return;

  unsigned long nowUs = micros();
  float dtSec = (float)(nowUs - lastCalcTimeUs) / 1000000.0f;
  if (dtSec < 0.0f || dtSec > 1.0f) dtSec = 0.05f;
  lastCalcTimeUs = nowUs;

  // 1. Điện áp Bus (Reg 0x02) - LSB = 1.25 mV
  liveBusVoltageV = (float)rawBus * 0.00125f;

  // 2. Độ sụt áp Shunt (Reg 0x01) - LSB = 2.5 uV (0.0025 mV)
  int16_t signedShunt = (int16_t)rawShunt;
  liveShuntVoltagemV = (float)signedShunt * 0.0025f;
  float shuntVoltageV = liveShuntVoltagemV / 1000.0f;

  // 3. Dòng điện I = Vshunt / Rshunt (A)
  liveCurrentA = shuntVoltageV / rShunt;
  liveCurrentmA = liveCurrentA * 1000.0f;

  // 4. Công suất P = Vbus * I (W)
  livePowerW = liveBusVoltageV * liveCurrentA;
  if (livePowerW < 0) livePowerW = -livePowerW;

  // 5. Tích phân dung lượng tiêu thụ (mAh và Wh)
  if (liveCurrentA > 0.005f) {
    liveEnergyMah += (liveCurrentmA * (dtSec / 3600.0f));
    liveEnergyWh  += (livePowerW * (dtSec / 3600.0f));
  }
}

// In dòng thông tin đẹp lên Serial Monitor
void printSerialTelemetry() {
  int barLength = (int)((liveBusVoltageV / 15.0f) * 16.0f);
  if (barLength > 16) barLength = 16;
  if (barLength < 0) barLength = 0;
  char barStr[20];
  for (int i = 0; i < 16; i++) barStr[i] = (i < barLength) ? '=' : ' ';
  barStr[16] = '\0';

  Serial.printf("[INA226 @ 0x%02X] V: %6.3fV [%-16s] | Shunt: %+6.2fmV | Dòng: %+6.3fA (%+7.1fmA) | P: %6.3fW | Đã dùng: %7.1fmAh (%5.3fWh)\n",
                inaAddress, liveBusVoltageV, barStr, liveShuntVoltagemV, liveCurrentA, liveCurrentmA, livePowerW, liveEnergyMah, liveEnergyWh);
  Serial.flush();
}

void checkBootButton() {
  int reading = digitalRead(BUTTON_BOOT_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > 50) {
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("\n>>> [NÚT BOOT ĐƯỢC BẤM] -> Chạy lại chẩn đoán phần cứng toàn diện!");
      runFullDiagnosticSequence();
    }
  }
  lastButtonState = reading;
}

void handleSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.println();
  Serial.printf(">> LỆNH SERIAL: '%s'\n", cmd.c_str());

  if (cmd.equalsIgnoreCase("s") || cmd.equalsIgnoreCase("scan")) {
    inaDetected = scanI2CBus(true);
    if (inaDetected) configureINA226();
  }
  else if (cmd.equalsIgnoreCase("swap")) {
    int temp = pinSDA;
    pinSDA = pinSCL;
    pinSCL = temp;
    isPinSwapped = !isPinSwapped;

    Serial.printf("--> ĐÃ ĐẢO CHÂN: SDA = GPIO %d | SCL = GPIO %d\n", pinSDA, pinSCL);
    checkPinElectricalHealth();
    initI2C();
    inaDetected = scanI2CBus(true);
    if (inaDetected) configureINA226();
  }
  else if (cmd.equalsIgnoreCase("t") || cmd.equalsIgnoreCase("test")) {
    if (inaDetected) {
      runDataTransmissionStressTest(100);
    } else {
      Serial.println("[THÔNG BÁO] Chưa kết nối INA226. Đang thử quét lại...");
      inaDetected = scanI2CBus(true);
      if (inaDetected) runDataTransmissionStressTest(100);
    }
  }
  else if (cmd.equalsIgnoreCase("r") || cmd.equalsIgnoreCase("dump")) {
    if (inaDetected) dumpAllRegisters();
    else Serial.println("[LỖI] Chưa phát hiện cảm biến INA226!");
  }
  else if (cmd.equalsIgnoreCase("zero") || cmd.equalsIgnoreCase("reset")) {
    liveEnergyMah = 0.0f;
    liveEnergyWh = 0.0f;
    Serial.println("--> ĐÃ RESET NĂNG LƯỢNG TÍCH LŨY VỀ 0.0 mAh / 0.0 Wh!");
  }
  else if (cmd.startsWith("cal ") || cmd.startsWith("CAL ")) {
    float val = cmd.substring(4).toFloat();
    if (val > 0.0001f && val < 10.0f) {
      rShunt = val;
      Serial.printf("--> ĐÃ CẬP NHẬT R_SHUNT = %.4f Ohm!\n", rShunt);
      if (inaDetected) configureINA226();
    } else {
      Serial.println("[LỖI] Giá trị R_shunt không hợp lệ! Ví dụ đúng: cal 0.1 hoặc cal 0.01");
    }
  }
  else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
    printHelpMenu();
  }
  Serial.flush();
}

void printHelpMenu() {
  Serial.println("\n-------------------------------------------------------------------------------");
  Serial.println("  DANH SÁCH LỆNH SERIAL MONITOR (Nhập lệnh rồi nhấn Enter):");
  Serial.println("-------------------------------------------------------------------------------");
  Serial.printf("  [swap]     : Đảo chân SDA <-> SCL (Hiện tại: SDA=%d, SCL=%d)\n", pinSDA, pinSCL);
  Serial.println("  [s / scan] : Quét lại bus I2C tìm cảm biến");
  Serial.println("  [t / test] : Chạy bài test truyền nhận 100 gói dữ liệu đo tỷ lệ mất gói");
  Serial.println("  [r / dump] : In toàn bộ thanh ghi phần cứng (Register Dump)");
  Serial.println("  [zero]     : Đặt lại bộ đếm dung lượng tiêu thụ về 0.0 mAh");
  Serial.printf("  [cal <val>]: Thay đổi giá trị trở Shunt (Hiện tại: %.3f Ohm). Ví dụ: 'cal 0.01'\n", rShunt);
  Serial.println("  [Nút BOOT] : Bấm nút BOOT trên mạch để chạy lại chẩn đoán toàn diện");
  Serial.println("  [h / help] : In bảng trợ giúp này");
  Serial.println("-------------------------------------------------------------------------------");
  Serial.flush();
}
