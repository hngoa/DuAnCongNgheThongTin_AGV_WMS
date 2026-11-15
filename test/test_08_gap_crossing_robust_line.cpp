#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <math.h>
#include <esp_arduino_version.h>
#include "soc/usb_serial_jtag_reg.h"
#include "driver/gpio.h"

/*
 * ===================================================================================
 * AGV_Ver2 - TEST 08: ĐIỀU KHIỂN BÁM LINE ROBUST & VƯỢT VẠCH ĐỨT ĐOẠN / MỜ / NHIỄU SÁNG
 * -----------------------------------------------------------------------------------
 * ĐẶC TẢ KỸ THUẬT NÂNG CẤP BỔ TRỢ TOÀN DIỆN CHO TEST 05:
 *   1. CẦU NỐI QUÁN TÍNH VƯỢT ĐỨT ĐOẠN (DEAD-RECKONING VIRTUAL LINE BRIDGE):
 *      - Khi vạch line bị đứt đoạn (khe hở từ 5cm đến 50cm), hệ thống KHÔNG xoay tại chỗ
 *        như các thuật toán thông thường (tránh văng khỏi đường băng).
 *      - Xe tự động chuyển sang chế độ Cầu Nối Quán Tính: Dùng MPU-6050 Gyro Z kết hợp
 *        Odometry vi sai để khóa góc hướng thẳng tắp theo la bàn quán tính, tiếp tục
 *        lăn bánh băng qua khoảng đứt như đang có "vạch kẻ ảo vô hình".
 *      - Ngay khi mắt cảm biến bắt lại được vạch đen phía bên kia khe hở -> Chuyển êm
 *        (Bumpless Transfer) về lại bám line PD mà không hề giật cục.
 *
 *   2. BỘ LỌC ĐỘ TIN CẬY KHÁNG NHIỄU VẠCH MỜ & CHẬP CHỜN (TEMPORAL DEBOUNCE FILTER):
 *      - Nếu vạch mờ làm tín hiệu mắt cảm biến mất tạm thời trong 1-3 chu kỳ (10-30ms),
 *        thuật toán duy trì ước lượng nội suy sai số lệch tâm trước đó (Persistent Error),
 *        triệt tiêu hiện tượng giật lắc bánh lái do tín hiệu chập chờn.
 *
 *   3. LỌC KHÔNG GIAN BÀO TRÒN NHIỄU QUANG HỌC (SPATIAL PLAUSIBILITY CHECK):
 *      - Loại bỏ các mẫu tín hiệu phi vật lý do phản xạ lóa đèn hoặc nhiễu bề mặt sàn
 *        (ví dụ mẫu 10001: 2 mắt rìa sáng nhưng 3 mắt giữa tối) -> Tự động nhận diện nhiễu.
 *
 *   4. TÍNH NĂNG MÔ PHỎNG ĐỨT ĐOẠN THỬ NGHIỆM TRỰC TIẾP (SIMULATED GAP MODE):
 *      - Nhấn phím 'g' trên Serial Monitor để kích hoạt chế độ "Giả lập đứt vạch 25cm":
 *        Vi điều khiển cố tình ngắt tín hiệu mắt line trong 25cm trên đường thẳng
 *        để bạn kiểm chứng xe tự động kích hoạt Cầu nối Quán tính mà không cần bóc băng keo!
 *
 *   5. KẾ THỪA TOÀN VẸN CÁC THÀNH CÔNG TỪ TEST 05:
 *      - Tọa độ sa bàn chuẩn 154cm x 151cm, tâm 2 bánh xe tại gốc O (Trạm A).
 *      - Tiếp cận góc trạm chuẩn tâm 2 bánh (APPROACH_CORNER, lăn thêm L = 23cm).
 *      - Xoay chuyển hướng thích ứng khóa vạch tức thì (SUB_TURNING_FOR_MISSION).
 *      - Khử trôi đa tầng: ZUPT tĩnh + Complementary Line-Heading Fusion động.
 *      - Giám sát Web Telemetry thời gian thực trên Port 5000 (Core 0 mạng, Core 1 điều khiển 100Hz).
 * ===================================================================================
 */

// --- 1. CẤU HÌNH KẾT NỐI WI-FI & SERVER ---
const char* wifi_ssid        = "LAU 3";
const char* wifi_pass        = "777777777";
String      serverUrl        = "http://192.168.1.31:5000/api/telemetry"; // IP máy tính chạy server.py
WiFiUDP     udpDiscovery;
#define     UDP_DISCOVERY_PORT 5005

// --- 2. SƠ ĐỒ CHÂN PHẦN CỨNG ---
// I2C MPU-6050
#define I2C_SDA_PIN          4
#define I2C_SCL_PIN          5
#define MPU6050_ADDR         0x68

// Driver L298N
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

// Encoder Hall
#define ENC_L_A_PIN          10
#define ENC_L_B_PIN          11
#define ENC_R_A_PIN          12
#define ENC_R_B_PIN          13

// Cảm biến dò line TCRT5000 5 mắt
#define LINE_L2_PIN          48
#define LINE_L1_PIN          47
#define LINE_C_PIN           21
#define LINE_R1_PIN          20
#define LINE_R2_PIN          19

// --- 3. THÔNG SỐ VẬT LÝ & HIỆU CHUẨN THỰC ĐO ---
bool invertLeftMotor         = true;
bool invertRightMotor        = true;
bool swapMotors              = true;

bool invertEncLeft           = false;
bool invertEncRight          = false;
bool swapEncoders            = true;

const float TICKS_PER_METER          = 6970.0f;
const float DIST_AXLE_TO_SENSOR_M    = 0.23f; // L = 23cm từ tâm trục bánh đến hàng mắt
const float WHEEL_BASE_W_M           = 0.29f; // W = 29cm khoảng cách 2 bánh
const float WHEEL_DIAM_M             = 0.065f;// d = 6.5cm

// Tốc độ điều khiển thích ứng
int   straightPwm            = 215;
int   curvePwm               = 178;
int   brakePwm               = 182;
int   turnPwm                = 175;
int   bridgePwm              = 192; // Tốc độ vượt khoảng đứt đoạn
int   maxPwm                 = 240;

// Hệ số PD bám line
float Kp_line                = 36.0f;
float Kd_line                = 15.0f;

// --- 4. CÁC THAM SỐ CHUYÊN BIỆT VƯỢT ĐỨT ĐOẠN / VẠCH MỜ ---
const float MAX_BRIDGE_DISTANCE_M    = 0.60f; // Quãng đường tối đa cho phép vượt vạch đứt (60cm)
const unsigned long MAX_BRIDGE_TIME_MS = 2500; // Thời gian tối đa cho phép vượt đứt đoạn (2.5 giây)
float gapStartDistM                  = 0.0f;
float gapHeadingHoldDeg              = 0.0f;
bool  inGapBridgeMode                = false;
unsigned long gapStartTimeMs         = 0;
int   consecutiveZeroLineFrames      = 0; // Bộ đếm frame mất vạch để lọc vạch mờ

// Cờ mô phỏng đứt vạch nhân tạo bằng phần mềm để thử nghiệm
bool  simulateGapActive              = false;
float simGapStartLegDistM            = 0.0f;
const float SIM_GAP_LENGTH_M         = 0.25f; // Giả lập đứt 25cm

// --- 5. CÁC BIẾN TOÀN CỤC CHIA SẺ GIỮA 2 CORE ---
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool isRunning      = false;
volatile float odomX         = 0.0f;
volatile float odomY         = 0.0f;
volatile float yawDeg        = 90.0f;
volatile float totalDistM    = 0.0f;
volatile float legProgressM  = 0.0f;
volatile float currentSpeed  = 0.0f;
volatile int   lineMask      = 0;
volatile float currentLineErr= 0.0f;
volatile int   cmdPwmL       = 0;
volatile int   cmdPwmR       = 0;
volatile long  globalTicksL  = 0;
volatile long  globalTicksR  = 0;
String         robotStateStr = "STOPPED";

// Encoder ISR raw
volatile long rawTicksL      = 0;
volatile long rawTicksR      = 0;
long prevTicksL              = 0;
long prevTicksR              = 0;

// MPU-6050
bool  mpuConnected           = false;
float gyroZOffset            = -1.31f;

// Trạng thái vận hành & Nhiệm vụ
enum LegId { LEG_AB = 0, LEG_BC = 1, LEG_CD = 2, LEG_DA = 3, LEG_NONE = 4 };
enum RobotSubState {
  SUB_STOPPED,
  SUB_TURNING_FOR_MISSION,
  SUB_TURNING_REVERSE_SEARCH,
  SUB_TURNING_MANUAL,
  SUB_LINE_FOLLOWING,
  SUB_DEAD_RECKONING_BRIDGE, // Chế độ Cầu nối Quán tính vượt đứt đoạn
  SUB_MOTOR_TEST,
  SUB_PAUSE_AT_STATION
};

volatile LegId         currentLeg          = LEG_AB;
volatile RobotSubState robotSubState       = SUB_STOPPED;
volatile float         legTargetDistM      = 1.51f;
volatile float         legTargetHeadingDeg = 90.0f;
volatile float         startTurnYaw        = 0.0f;
volatile unsigned long turnSearchStartMs   = 0;

volatile bool          autoLoopEnabled     = false;
volatile unsigned long stationPauseUntilMs = 0;

float lastValidError         = 0.0f;
float lastLineError          = 0.0f;
unsigned long lineLostTimeMs = 0;

float smoothedBaseSpeed      = 0.0f;
const float MAX_RAMP_STEP    = 4.0f;

volatile int lastAckId                  = 0;
volatile unsigned long testMotorUntilMs = 0;

inline float normalizeAngleDeg(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ===================================================================================
// TRÌNH PHỤC VỤ NGẮT ENCODER
// ===================================================================================
void IRAM_ATTR isrEncLeftA() {
  int a = gpio_get_level((gpio_num_t)ENC_L_A_PIN);
  int b = gpio_get_level((gpio_num_t)ENC_L_B_PIN);
  if (a == b) rawTicksL++; else rawTicksL--;
}

void IRAM_ATTR isrEncLeftB() {
  int a = gpio_get_level((gpio_num_t)ENC_L_A_PIN);
  int b = gpio_get_level((gpio_num_t)ENC_L_B_PIN);
  if (a != b) rawTicksL++; else rawTicksL--;
}

void IRAM_ATTR isrEncRightA() {
  int a = gpio_get_level((gpio_num_t)ENC_R_A_PIN);
  int b = gpio_get_level((gpio_num_t)ENC_R_B_PIN);
  if (a != b) rawTicksR++; else rawTicksR--;
}

void IRAM_ATTR isrEncRightB() {
  int a = gpio_get_level((gpio_num_t)ENC_R_A_PIN);
  int b = gpio_get_level((gpio_num_t)ENC_R_B_PIN);
  if (a == b) rawTicksR++; else rawTicksR--;
}

void getCalibratedTicks(long &outL, long &outR) {
  noInterrupts();
  long rL = rawTicksL;
  long rR = rawTicksR;
  interrupts();

  long tL = swapEncoders ? rR : rL;
  long tR = swapEncoders ? rL : rR;

  outL = invertEncLeft  ? -tL : tL;
  outR = invertEncRight ? -tR : tR;
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

void setMotors(int leftVal, int rightVal) {
  cmdPwmL = leftVal;
  cmdPwmR = rightVal;

  int cmdL = swapMotors ? rightVal : leftVal;
  int cmdR = swapMotors ? leftVal  : rightVal;

  int l = invertLeftMotor  ? -cmdL : cmdL;
  int r = invertRightMotor ? -cmdR : cmdR;

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
  setMotors(0, 0);
  smoothedBaseSpeed = 0.0f;
}

// ===================================================================================
// BẮT ĐẦU NHIỆM VỤ CHẶNG MỚI
// ===================================================================================
void startMission(LegId leg) {
  currentLeg = leg;

  if (currentLeg == LEG_AB) {
    legTargetDistM = 1.51f;
    legTargetHeadingDeg = 90.0f;
  } else if (currentLeg == LEG_BC) {
    legTargetDistM = 1.54f;
    legTargetHeadingDeg = 0.0f;
  } else if (currentLeg == LEG_CD) {
    legTargetDistM = 1.51f;
    legTargetHeadingDeg = -90.0f;
  } else if (currentLeg == LEG_DA) {
    legTargetDistM = 1.54f;
    legTargetHeadingDeg = 180.0f;
  }

  prevTicksL = globalTicksL;
  prevTicksR = globalTicksR;
  legProgressM = 0.0f;

  float headingErr = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
  smoothedBaseSpeed = 0.0f;
  lastLineError     = 0.0f;
  lastValidError    = 0.0f;
  lineLostTimeMs    = 0;
  inGapBridgeMode   = false;
  consecutiveZeroLineFrames = 0;

  if (fabsf(headingErr) > 6.0f) {
    startTurnYaw = yawDeg;
    robotSubState = SUB_TURNING_FOR_MISSION;
    robotStateStr = "TURNING_STATION";
    isRunning = true;
    Serial.printf("\n>> [KHỞI ĐỘNG CHẶNG %d] Cần xoay %+.1f° về hướng đích %.1f°...\n",
                  (int)leg + 1, headingErr, legTargetHeadingDeg);
  } else {
    prevTicksL = globalTicksL;
    prevTicksR = globalTicksR;
    robotSubState = SUB_LINE_FOLLOWING;
    robotStateStr = "RUNNING_FAST";
    isRunning = true;
    Serial.printf("\n>> [KHỞI ĐỘNG CHẶNG %d] Đúng hướng %.1f° -> Khởi hành bám line!\n",
                  (int)leg + 1, legTargetHeadingDeg);
  }
}

void advanceToNextLeg() {
  if (currentLeg == LEG_AB) startMission(LEG_BC);
  else if (currentLeg == LEG_BC) startMission(LEG_CD);
  else if (currentLeg == LEG_CD) startMission(LEG_DA);
  else startMission(LEG_AB);
}

void startManualTurn(float deltaAngleDeg) {
  legTargetHeadingDeg = normalizeAngleDeg(yawDeg + deltaAngleDeg);
  robotSubState = SUB_TURNING_MANUAL;
  robotStateStr = "TURNING_MANUAL";
  isRunning = true;
  smoothedBaseSpeed = 0.0f;
  Serial.printf("\n>> [LỆNH XOAY THỦ CÔNG] Xoay %+.1f° tới hướng đích %.1f°...\n", deltaAngleDeg, legTargetHeadingDeg);
}

// ===================================================================================
// KHỞI TẠO CẢM BIẾN DÒ LINE TCRT5000
// ===================================================================================
void initLineSensors() {
  CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG,
                      USB_SERIAL_JTAG_USB_PAD_ENABLE |
                      USB_SERIAL_JTAG_DP_PULLUP |
                      USB_SERIAL_JTAG_DM_PULLUP |
                      USB_SERIAL_JTAG_DP_PULLDOWN |
                      USB_SERIAL_JTAG_DM_PULLDOWN);
  SET_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_PAD_PULL_OVERRIDE);

  gpio_reset_pin((gpio_num_t)LINE_L2_PIN);
  gpio_reset_pin((gpio_num_t)LINE_L1_PIN);
  gpio_reset_pin((gpio_num_t)LINE_C_PIN);
  gpio_reset_pin((gpio_num_t)LINE_R1_PIN);
  gpio_reset_pin((gpio_num_t)LINE_R2_PIN);

  pinMode(LINE_L2_PIN, INPUT_PULLUP);
  pinMode(LINE_L1_PIN, INPUT_PULLUP);
  pinMode(LINE_C_PIN,  INPUT_PULLUP);
  pinMode(LINE_R1_PIN, INPUT_PULLUP);
  pinMode(LINE_R2_PIN, INPUT_PULLUP);
}

int readLineSensors(int &sL2, int &sL1, int &sC, int &sR1, int &sR2) {
  sL2 = digitalRead(LINE_L2_PIN);
  sL1 = digitalRead(LINE_L1_PIN);
  sC  = digitalRead(LINE_C_PIN);
  sR1 = digitalRead(LINE_R1_PIN);
  sR2 = digitalRead(LINE_R2_PIN);

  int mask = (sL2 << 4) | (sL1 << 3) | (sC << 2) | (sR1 << 1) | sR2;

  // LỌC KHÔNG GIAN BÀO TRÒN NHIỄU QUANG HỌC:
  // Nếu xuất hiện mẫu rời rạc 10001 (2 mắt rìa sáng mà 3 mắt giữa tối) -> Nhiễu ánh sáng!
  if (mask == 0b10001) {
    sL2 = 0; sL1 = 0; sC = 0; sR1 = 0; sR2 = 0;
    return 0;
  }

  // NẾU ĐANG BẬT CHẾ ĐỘ GIẢ LẬP ĐỨT VẠCH ĐỂ TEST (SIMULATED GAP)
  if (simulateGapActive) {
    float simTraveled = legProgressM - simGapStartLegDistM;
    if (simTraveled >= 0.0f && simTraveled < SIM_GAP_LENGTH_M) {
      sL2 = 0; sL1 = 0; sC = 0; sR1 = 0; sR2 = 0;
      return 0; // Giả lập mất vạch hoàn toàn
    } else if (simTraveled >= SIM_GAP_LENGTH_M) {
      simulateGapActive = false; // Hết khoảng đứt giả lập -> Trả lại vạch thật
      Serial.println("\n>> [GIẢ LẬP ĐỨT VẠCH XONG] Đã vượt qua 25cm giả lập -> Trở lại cảm biến thật!");
    }
  }

  return mask;
}

// ===================================================================================
// KHỞI TẠO & CẬP NHẬT MPU-6050
// ===================================================================================
bool writeReg8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

int16_t readReg16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, 2);
  if (Wire.available() == 2) {
    return (int16_t)((Wire.read() << 8) | Wire.read());
  }
  return 0;
}

void calibrateGyroZ(int samples = 500) {
  double sum = 0;
  for (int i = 0; i < samples; i++) {
    int16_t r = readReg16(MPU6050_ADDR, 0x47);
    sum += (r / 32.8f);
    delay(2);
  }
  gyroZOffset = (float)(sum / samples);
}

void initMPU() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  delay(50);
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    writeReg8(MPU6050_ADDR, 0x6B, 0x00);
    delay(10);
    writeReg8(MPU6050_ADDR, 0x1A, 0x03); // DLPF 44Hz
    writeReg8(MPU6050_ADDR, 0x1B, 0x10); // ±1000 dps
    mpuConnected = true;
    calibrateGyroZ(400);
  }
}

// ===================================================================================
// CORE 1: ĐIỀU KHIỂN BÁM LINE ROBUST & VƯỢT ĐỨT ĐOẠN (100Hz CHUẨN)
// ===================================================================================
void updateControlAndOdometry(float dt = 0.010f) {
  // 1. Cập nhật góc Yaw từ MPU-6050
  if (mpuConnected) {
    int16_t rGz = readReg16(MPU6050_ADDR, 0x47);
    float gzRaw = (rGz / 32.8f);

    if (!isRunning && (testMotorUntilMs < millis()) && (robotSubState == SUB_STOPPED || robotSubState == SUB_PAUSE_AT_STATION)) {
      gyroZOffset = 0.995f * gyroZOffset + 0.005f * gzRaw; // ZUPT khử trôi tĩnh
    } else {
      float gz = -(gzRaw - gyroZOffset);
      if (fabsf(gz) < 0.25f) gz = 0.0f;
      yawDeg = normalizeAngleDeg(yawDeg + gz * dt);
    }
  }

  // 2. Cập nhật Odometry
  long curL, curR;
  getCalibratedTicks(curL, curR);
  globalTicksL = curL;
  globalTicksR = curR;

  long dL = curL - prevTicksL;
  long dR = curR - prevTicksR;
  prevTicksL = curL;
  prevTicksR = curR;

  float deltaSL = (float)dL / TICKS_PER_METER;
  float deltaSR = (float)dR / TICKS_PER_METER;
  float deltaS  = (deltaSL + deltaSR) / 2.0f;

  currentSpeed = (deltaS / dt) * 100.0f;
  totalDistM += fabsf(deltaS);

  float rad = (yawDeg * PI) / 180.0f;
  odomX += deltaS * cosf(rad);
  odomY += deltaS * sinf(rad);

  // 3. Đọc 5 mắt cảm biến TCRT5000 có lọc quang học
  int sL2, sL1, sC, sR1, sR2;
  lineMask = readLineSensors(sL2, sL1, sC, sR1, sR2);
  int activeCount = sL2 + sL1 + sC + sR1 + sR2;

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 0: TẠM NGHỈ TẠI TRẠM (PAUSE 2S)
  // ---------------------------------------------------------------------------------
  if (robotSubState == SUB_PAUSE_AT_STATION) {
    setMotors(0, 0);
    if (millis() >= stationPauseUntilMs) {
      advanceToNextLeg();
    }
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 1A: XOAY TẠI TRẠM (KHÓA VẠCH THÍCH ỨNG TỨC THÌ)
  // ---------------------------------------------------------------------------------
  if (robotSubState == SUB_TURNING_FOR_MISSION) {
    float err = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
    float absErr = fabsf(err);
    float rotatedSoFar = fabsf(normalizeAngleDeg(yawDeg - startTurnYaw));

    bool lineCaught = (rotatedSoFar >= 20.0f) && ((lineMask & 0b01110) != 0);

    if (lineCaught) {
      setMotors(0, 0);
      smoothedBaseSpeed = 0.0f;
      yawDeg = legTargetHeadingDeg;
      prevTicksL = globalTicksL;
      prevTicksR = globalTicksR;
      legProgressM = 0.0f;
      lastLineError = 0.0f;
      lastValidError = 0.0f;
      inGapBridgeMode = false;
      consecutiveZeroLineFrames = 0;
      robotSubState = SUB_LINE_FOLLOWING;
      robotStateStr = "RUNNING_FAST";
      isRunning = true;
      Serial.printf("\n>> [BẮT ĐÚNG VẠCH MỚI] Đã khóa vạch chặng %d tại Yaw = %.1f° -> Khởi hành!\n",
                    (int)currentLeg + 1, yawDeg);
      return;
    }

    if (absErr <= 2.5f) {
      setMotors(0, 0);
      smoothedBaseSpeed = 0.0f;
      yawDeg = legTargetHeadingDeg;

      if ((lineMask & 0b01110) != 0) {
        prevTicksL = globalTicksL;
        prevTicksR = globalTicksR;
        legProgressM = 0.0f;
        lastLineError = 0.0f;
        lastValidError = 0.0f;
        inGapBridgeMode = false;
        consecutiveZeroLineFrames = 0;
        robotSubState = SUB_LINE_FOLLOWING;
        robotStateStr = "RUNNING_FAST";
        isRunning = true;
        return;
      } else {
        robotSubState = SUB_TURNING_REVERSE_SEARCH;
        turnSearchStartMs = millis();
        return;
      }
    }

    int turnSpeed = (rotatedSoFar < 20.0f) ? 175 : 168;
    if (err < 0.0f) setMotors(-turnSpeed, turnSpeed);
    else setMotors(turnSpeed, -turnSpeed);
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 1B: QUÉT NGƯỢC NẾU XOAY LỐ ĐÀ
  // ---------------------------------------------------------------------------------
  if (robotSubState == SUB_TURNING_REVERSE_SEARCH) {
    bool found = ((lineMask & 0b01110) != 0);
    bool timeout = (millis() - turnSearchStartMs > 800);

    if (found || timeout) {
      setMotors(0, 0);
      smoothedBaseSpeed = 0.0f;
      prevTicksL = globalTicksL;
      prevTicksR = globalTicksR;
      legProgressM = 0.0f;
      lastLineError = 0.0f;
      lastValidError = 0.0f;
      inGapBridgeMode = false;
      consecutiveZeroLineFrames = 0;
      robotSubState = SUB_LINE_FOLLOWING;
      robotStateStr = "RUNNING_FAST";
      isRunning = true;
      return;
    }

    float err = normalizeAngleDeg(legTargetHeadingDeg - startTurnYaw);
    int sweepSpeed = 168;
    if (err < 0.0f) setMotors(sweepSpeed, -sweepSpeed);
    else setMotors(-sweepSpeed, sweepSpeed);
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 1C: XOAY THỦ CÔNG
  // ---------------------------------------------------------------------------------
  if (robotSubState == SUB_TURNING_MANUAL) {
    float err = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
    float absErr = fabsf(err);

    if (absErr <= 2.5f) {
      stopMotors();
      yawDeg = legTargetHeadingDeg;
      robotSubState = SUB_STOPPED;
      robotStateStr = "STOPPED";
      isRunning = false;
      return;
    }

    int turnSpeed = (absErr > 40.0f) ? 178 : ((absErr > 15.0f) ? 172 : 168);
    if (err < 0.0f) setMotors(-turnSpeed, turnSpeed);
    else setMotors(turnSpeed, -turnSpeed);
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 2: TEST ĐỘNG CƠ 1 GIÂY
  // ---------------------------------------------------------------------------------
  if (testMotorUntilMs > millis()) {
    setMotors(205, 205);
    robotStateStr = "MOTOR_TEST";
    return;
  }

  if (!isRunning || robotSubState == SUB_STOPPED) {
    stopMotors();
    robotStateStr = "STOPPED";
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 3: THEO DÕI TIẾN ĐỘ CHẶNG & TIẾP CẬN GÓC TRẠM
  // ---------------------------------------------------------------------------------
  legProgressM += deltaS;
  float remainingLegM = legTargetDistM - legProgressM;
  float sensorReachCornerDistM = legTargetDistM - DIST_AXLE_TO_SENSOR_M;

  int leftPwm  = 0;
  int rightPwm = 0;

  // GIAI ĐOẠN 3.1: ĐÃ VÀO VÙNG GÓC TRẠM (BÒ THÊM L = 23cm ĐỂ TÂM TRỤC TỚI ĐÚNG GÓC)
  if (legProgressM >= sensorReachCornerDistM) {
    if (remainingLegM <= 0.005f) {
      stopMotors();
      robotStateStr = "AT_STATION";

      Serial.printf("\n>> [ĐÃ TỚI TRẠM DỪNG] Đích chặng %d! Tâm trục 2 bánh tại X=%.3fm, Y=%.3fm, Yaw=%.1f°\n",
                    (int)currentLeg + 1, odomX, odomY, yawDeg);

      if (autoLoopEnabled) {
        stationPauseUntilMs = millis() + 2000;
        robotSubState = SUB_PAUSE_AT_STATION;
        robotStateStr = "PAUSED_AT_STATION";
        isRunning = true;
      } else {
        robotSubState = SUB_STOPPED;
        isRunning = false;
      }
      return;
    }

    float hErr = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
    float u_h = 3.0f * hErr;
    leftPwm  = constrain((int)(brakePwm + u_h), 168, 195);
    rightPwm = constrain((int)(brakePwm - u_h), 168, 195);
    robotStateStr = "APPROACH_CORNER";
    setMotors(leftPwm, rightPwm);
    return;
  }

  // =================================================================================
  // GIAI ĐOẠN 3.2: XỬ LÝ VƯỢT ĐỨT ĐOẠN / VẠCH MỜ / BÁM LINE ROBUST
  // =================================================================================
  if (activeCount == 0) {
    consecutiveZeroLineFrames++;

    // BỘ LỌC ĐỘ TIN CẬY KHÁNG NHIỄU VẠCH MỜ (DEBOUNCE 30ms = 3 frames):
    // Nếu vạch chỉ mờ chập chờn mất trong 1-3 chu kỳ, ta giữ nguyên hướng lái cũ
    if (consecutiveZeroLineFrames <= 3 && !inGapBridgeMode) {
      // Giữ tín hiệu lái trước đó, chưa vội kích hoạt cầu nối
      float u = (Kp_line * lastValidError);
      leftPwm  = constrain((int)(smoothedBaseSpeed + u), 168, maxPwm);
      rightPwm = constrain((int)(smoothedBaseSpeed - u), 168, maxPwm);
      robotStateStr = "LINE_FLICKER_FILTER";
      setMotors(leftPwm, rightPwm);
      return;
    }

    // NẾU MẤT VẠCH THỰC SỰ (> 30ms): KÍCH HOẠT CẦU NỐI QUÁN TÍNH VƯỢT ĐỨT ĐOẠN
    if (!inGapBridgeMode) {
      inGapBridgeMode = true;
      gapStartTimeMs = millis();
      gapStartDistM = totalDistM;
      gapHeadingHoldDeg = legTargetHeadingDeg; // Khóa hướng danh định của chặng
      Serial.printf("\n⚡ [PHÁT HIỆN VẠCH ĐỨT] Kích hoạt Cầu Nối Quán Tính! Khóa hướng Yaw = %.1f°...\n", gapHeadingHoldDeg);
    }

    float gapTraveled = totalDistM - gapStartDistM;

    // KIỂM TRA BẢO VỆ AN TOÀN:
    // Nếu vượt quá 60cm hoặc quá 2.5s mà vẫn không thấy vạch -> Dừng an toàn khẩn cấp
    if (gapTraveled > MAX_BRIDGE_DISTANCE_M || (millis() - gapStartTimeMs > MAX_BRIDGE_TIME_MS)) {
      stopMotors();
      robotStateStr = "GAP_TIMEOUT_STOP";
      Serial.printf("\n❌ [DỪNG AN TOÀN] Vượt đứt đoạn quá %.2fm không thấy vạch lại!\n", gapTraveled);
      return;
    }

    // NẾU TRƯỚC ĐÓ ĐANG ÔM CUA GẮT (|lastValidError| >= 1.2):
    // Có thể xe bị trượt ra ngoài khúc cua, dùng bộ nhớ hướng để quay lại vạch
    if (fabsf(lastValidError) >= 1.2f) {
      if (lastValidError < 0.0f) {
        robotStateStr = "LOST_CURVE_LEFT";
        leftPwm  = -turnPwm;
        rightPwm = curvePwm + 20;
      } else {
        robotStateStr = "LOST_CURVE_RIGHT";
        leftPwm  = curvePwm + 20;
        rightPwm = -turnPwm;
      }
      setMotors(leftPwm, rightPwm);
      return;
    }

    // TRƯỜNG HỢP CỐT LÕI: VƯỢT ĐỨT ĐOẠN ĐƯỜNG THẲNG BẰNG DEAD-RECKONING VIRTUAL BRIDGE
    // Duy trì vận tốc tịnh tiến và khóa góc hướng Yaw chuẩn xác bằng MPU-6050
    float hErr = normalizeAngleDeg(gapHeadingHoldDeg - yawDeg);
    float u_h = 3.5f * hErr;

    int targetBase = (remainingLegM < 0.40f) ? brakePwm : bridgePwm;
    if (targetBase > smoothedBaseSpeed) smoothedBaseSpeed = min((float)targetBase, smoothedBaseSpeed + MAX_RAMP_STEP);
    else if (targetBase < smoothedBaseSpeed) smoothedBaseSpeed = max((float)targetBase, smoothedBaseSpeed - MAX_RAMP_STEP);

    leftPwm  = constrain((int)(smoothedBaseSpeed + u_h), 160, 225);
    rightPwm = constrain((int)(smoothedBaseSpeed - u_h), 160, 225);
    robotStateStr = "DEAD_RECKONING_BRIDGE";
    setMotors(leftPwm, rightPwm);
    return;

  } else {
    // ===============================================================================
    // CÓ TÍN HIỆU VẠCH (HOẶC VỪA BẮT LẠI ĐƯỢC VẠCH SAU KHOẢNG ĐỨT)
    // ===============================================================================
    consecutiveZeroLineFrames = 0;

    // Nếu vừa thoát khỏi chế độ cầu nối đứt đoạn:
    if (inGapBridgeMode) {
      float gapCrossedM = totalDistM - gapStartDistM;
      inGapBridgeMode = false;
      Serial.printf("\n🎯 [VƯỢT ĐỨT ĐOẠN XUẤT SẮC] Đã băng qua khe hở %.2fm an toàn -> Chuyển êm về bám line!\n", gapCrossedM);
    }

    lineLostTimeMs = 0;

    // Trọng số: L2=-2, L1=-1, C=0, R1=+1, R2=+2
    currentLineErr = (float)(-2*sL2 + -1*sL1 + 0*sC + 1*sR1 + 2*sR2) / (float)activeCount;
    lastValidError = currentLineErr;

    // Khử trôi Gyro động bằng vạch Line (Complementary Line-Heading Fusion)
    if (fabsf(currentLineErr) < 0.6f) {
      float expectedHeading = legTargetHeadingDeg - (currentLineErr * 4.0f);
      float headingDiff = normalizeAngleDeg(expectedHeading - yawDeg);
      yawDeg = normalizeAngleDeg(yawDeg + 0.025f * headingDiff);
    }

    float dErr = (currentLineErr - lastLineError) / fmaxf(dt, 0.001f);
    lastLineError = currentLineErr;

    float absErr = fabsf(currentLineErr);
    int targetBase = straightPwm - (int)(absErr * (straightPwm - curvePwm));
    targetBase = constrain(targetBase, curvePwm, straightPwm);

    if (remainingLegM < 0.40f) {
      targetBase = min(targetBase, brakePwm);
      robotStateStr = "BRAKING_NEAR_STATION";
    } else {
      robotStateStr = (absErr < 0.35f) ? "RUNNING_FAST" : "TURNING_LINE";
    }

    if (targetBase > smoothedBaseSpeed) smoothedBaseSpeed = min((float)targetBase, smoothedBaseSpeed + MAX_RAMP_STEP);
    else if (targetBase < smoothedBaseSpeed) smoothedBaseSpeed = max((float)targetBase, smoothedBaseSpeed - MAX_RAMP_STEP);

    float u = (Kp_line * currentLineErr) + (Kd_line * dErr);

    if (currentLineErr <= -1.8f) {
      leftPwm  = -turnPwm;
      rightPwm = curvePwm + 25;
      robotStateStr = "PIVOT_LEFT";
    } else if (currentLineErr >= 1.8f) {
      leftPwm  = curvePwm + 25;
      rightPwm = -turnPwm;
      robotStateStr = "PIVOT_RIGHT";
    } else {
      leftPwm  = constrain((int)(smoothedBaseSpeed + u), 0, maxPwm);
      rightPwm = constrain((int)(smoothedBaseSpeed - u), 0, maxPwm);
    }
  }

  setMotors(leftPwm, rightPwm);
}

// ===================================================================================
// TÁC VỤ FREERTOS ĐỊNH THỜI 100Hz TRÊN CORE 1
// ===================================================================================
void controlLoopTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    updateControlAndOdometry(0.010f);
  }
}

// ===================================================================================
// CORE 0: KẾT NỐI WI-FI & TELEMETRY WEBSERVER (150ms)
// ===================================================================================
void telemetryTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  Serial.print(">> [WI-FI]: Đang kết nối tới ");
  Serial.println(wifi_ssid);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 25) {
    delay(400);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ [WI-FI]: Đã kết nối! IP = %s\n", WiFi.localIP().toString().c_str());

    // Tự động lắng nghe gói phát sóng UDP (Port 5005) từ Server để tìm IP máy tính
    udpDiscovery.begin(UDP_DISCOVERY_PORT);
    Serial.printf(">> [DÒ TÌM SERVER]: Đang quét beacon UDP tại cổng %d...\n", UDP_DISCOVERY_PORT);
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1500) {
      int packetSize = udpDiscovery.parsePacket();
      if (packetSize) {
        char packetBuffer[255];
        int len = udpDiscovery.read(packetBuffer, 254);
        if (len > 0) packetBuffer[len] = 0;
        String s = String(packetBuffer);
        if (s.indexOf("AGV_SERVER") >= 0) {
          IPAddress srvIp = udpDiscovery.remoteIP();
          serverUrl = "http://" + srvIp.toString() + ":5000/api/telemetry";
          Serial.printf("🎯 [TỰ ĐỘNG TÌM THẤY SERVER]: Server IP = %s -> URL: %s\n", srvIp.toString().c_str(), serverUrl.c_str());
          break;
        }
      }
      delay(30);
    }
  } else {
    Serial.println("\n⚠️ [WI-FI]: Không kết nối được Wi-Fi -> Chạy nội bộ.");
  }

  HTTPClient http;
  http.setTimeout(1200);
  unsigned long lastUdpCheckMs = 0;
  unsigned long lastHttpErrMs = 0;

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      // Định kỳ kiểm tra gói UDP beacon nếu cần cập nhật IP Server động
      if (millis() - lastUdpCheckMs > 4000) {
        lastUdpCheckMs = millis();
        int packetSize = udpDiscovery.parsePacket();
        if (packetSize) {
          char packetBuffer[255];
          int len = udpDiscovery.read(packetBuffer, 254);
          if (len > 0) packetBuffer[len] = 0;
          String s = String(packetBuffer);
          if (s.indexOf("AGV_SERVER") >= 0) {
            IPAddress srvIp = udpDiscovery.remoteIP();
            String newUrl = "http://" + srvIp.toString() + ":5000/api/telemetry";
            if (newUrl != serverUrl) {
              serverUrl = newUrl;
              Serial.printf("🎯 [CẬP NHẬT IP SERVER MỚI]: %s\n", serverUrl.c_str());
            }
          }
        }
      }

      String payload = "{";
      payload += "\"x\":" + String(odomX, 3) + ",";
      payload += "\"y\":" + String(odomY, 3) + ",";
      payload += "\"yaw\":" + String(yawDeg, 1) + ",";
      payload += "\"dist\":" + String(totalDistM, 3) + ",";
      payload += "\"speed\":" + String(currentSpeed, 1) + ",";
      payload += "\"line_mask\":" + String(lineMask) + ",";
      payload += "\"line_err\":" + String(currentLineErr, 2) + ",";
      payload += "\"ticks_l\":" + String(globalTicksL) + ",";
      payload += "\"ticks_r\":" + String(globalTicksR) + ",";
      payload += "\"pwm_l\":" + String(cmdPwmL) + ",";
      payload += "\"pwm_r\":" + String(cmdPwmR) + ",";
      payload += "\"ack_id\":" + String(lastAckId) + ",";
      payload += "\"auto_loop\":" + String(autoLoopEnabled ? "true" : "false") + ",";
      payload += "\"state\":\"" + robotStateStr + "\"";
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
          Serial.printf("\n>> [LỆNH MẠNG #%d]: %s\n", incomingId, resp.c_str());

          if (resp.indexOf("START") >= 0) {
            startMission(currentLeg);
          } else if (resp.indexOf("STOP") >= 0) {
            autoLoopEnabled = false;
            stopMotors();
            robotStateStr = "STOPPED";
            Serial.println(">> [LỆNH]: DỪNG KHẨN CẤP!");
          } else if (resp.indexOf("TEST_FORWARD") >= 0) {
            testMotorUntilMs = millis() + 1000;
            isRunning = true;
            robotStateStr = "MOTOR_TEST";
            Serial.println(">> [LỆNH]: TEST ĐỘNG CƠ TIẾN 1 GIÂY!");
          } else if (resp.indexOf("RESET_ODOM") >= 0) {
            odomX = 0.0f; odomY = 0.0f; yawDeg = 90.0f; totalDistM = 0.0f; legProgressM = 0.0f;
            currentLeg = LEG_AB;
            autoLoopEnabled = false;
            noInterrupts(); rawTicksL = 0; rawTicksR = 0; interrupts();
            prevTicksL = 0; prevTicksR = 0;
            Serial.println(">> [LỆNH]: RESET ODOMETRY VỀ GỐC A (0,0) 90°!");
          } else if (resp.indexOf("CALIBRATE_GYRO") >= 0) {
            calibrateGyroZ(400);
            yawDeg = 90.0f;
            Serial.printf(">> [LỆNH]: ĐÃ CALIB GYRO! Offset = %+.3f | Yaw = 90.0°\n", gyroZOffset);
          } else if (resp.indexOf("MISSION:M1") >= 0) {
            autoLoopEnabled = false;
            startMission(LEG_AB);
          } else if (resp.indexOf("MISSION:M2") >= 0) {
            autoLoopEnabled = false;
            startMission(LEG_BC);
          } else if (resp.indexOf("MISSION:M3") >= 0) {
            autoLoopEnabled = false;
            startMission(LEG_CD);
          } else if (resp.indexOf("MISSION:M4") >= 0) {
            autoLoopEnabled = false;
            startMission(LEG_DA);
          } else if (resp.indexOf("MISSION:NEXT") >= 0) {
            advanceToNextLeg();
          } else if (resp.indexOf("AUTO_LOOP:ON") >= 0) {
            autoLoopEnabled = true;
            startMission(currentLeg);
            Serial.println(">> [LỆNH]: BẬT CHẾ ĐỘ VÒNG LẶP TỰ ĐỘNG (A-B-C-D-A)!");
          } else if (resp.indexOf("AUTO_LOOP:OFF") >= 0) {
            autoLoopEnabled = false;
            Serial.println(">> [LỆNH]: TẮT VÒNG LẶP TỰ ĐỘNG!");
          } else if (resp.indexOf("TURN_LEFT_90") >= 0) {
            startManualTurn(-90.0f); // Xoay TRÁI 90°
          } else if (resp.indexOf("TURN_RIGHT_90") >= 0) {
            startManualTurn(90.0f);  // Xoay PHẢI 90°
          } else if (resp.indexOf("TURN_180") >= 0) {
            startManualTurn(180.0f);
          } else if (resp.indexOf("SIMULATE_GAP") >= 0) {
            simulateGapActive = true;
            simGapStartLegDistM = legProgressM;
            Serial.printf("\n⚡ [LỆNH MẠNG]: BẬT GIẢ LẬP ĐỨT VẠCH 25cm tại %.2fm!\n", legProgressM);
          }
        }
      } else {
        if (millis() - lastHttpErrMs > 3000) {
          lastHttpErrMs = millis();
          Serial.printf("⚠️ [LỖI KẾT NỐI SERVER]: Không gửi được tới %s (Mã HTTP: %d). Đảm bảo server.py đang chạy!\n",
                        serverUrl.c_str(), httpCode);
        }
      }
      http.end();
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// ===================================================================================
// SETUP
// ===================================================================================
void setup() {
  Serial.begin(115200);
  delay(600);

  initMotors();
  initLineSensors();
  initMPU();

  pinMode(ENC_L_A_PIN, INPUT_PULLUP);
  pinMode(ENC_L_B_PIN, INPUT_PULLUP);
  pinMode(ENC_R_A_PIN, INPUT_PULLUP);
  pinMode(ENC_R_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A_PIN), isrEncLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B_PIN), isrEncLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A_PIN), isrEncRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B_PIN), isrEncRightB, CHANGE);

  odomX = 0.00f;
  odomY = 0.00f;
  yawDeg = 90.0f;
  legProgressM = 0.0f;

  Serial.println("\n=========================================================================");
  Serial.println("   AGV_VER2 - TEST 08: BÁM LINE ROBUST & VƯỢT VẠCH ĐỨT ĐOẠN / VẠCH MỜ");
  Serial.println("=========================================================================");
  Serial.printf("Tọa độ khởi đầu: X=%.2fm | Y=%.2fm | Yaw=%.1f°\n", odomX, odomY, yawDeg);
  Serial.println(">> CÁC PHÍM TẮT ĐIỀU CHUYỂN TRỰC TIẾP TRÊN SERIAL MONITOR:");
  Serial.println("   [Space / s] : Bật / Tắt chạy (START / STOP)");
  Serial.println("   [g]         : BẬT GIẢ LẬP ĐỨT VẠCH 25CM (Kiểm chứng cầu nối quán tính)");
  Serial.println("   [a]         : Bật / Tắt Vòng Lặp Tự Động (A-B-C-D-A)");
  Serial.println("   [n]         : Chuyển sang chặng kế tiếp (Leg AB -> BC -> CD -> DA)");
  Serial.println("   [1,2,3,4]   : Chạy chặng 1 (AB), 2 (BC), 3 (CD), 4 (DA)");
  Serial.println("   [c]         : Calib lại Gyro điểm 0 tĩnh");
  Serial.println("-------------------------------------------------------------------------");

  xTaskCreatePinnedToCore(telemetryTask, "TelemetryTask", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlLoopTask, "Control100Hz", 8192, NULL, 2, NULL, 1);
}

// ===================================================================================
// LOOP CHÍNH (XỬ LÝ PHÍM SERIAL & NÚT BOOT)
// ===================================================================================
void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == ' ' || ch == 's') {
      if (isRunning) {
        stopMotors();
        robotStateStr = "STOPPED";
        isRunning = false;
      } else {
        startMission(currentLeg);
      }
      Serial.printf("\n[SERIAL KEY]: %s\n", isRunning ? "START (CHẠY)" : "STOP (DỪNG)");
    } 
    else if (ch == 'g' || ch == 'G') {
      if (isRunning && robotSubState == SUB_LINE_FOLLOWING) {
        simulateGapActive = true;
        simGapStartLegDistM = legProgressM;
        Serial.printf("\n⚡ [KÍCH HOẠT TEST]: Đã kích hoạt GIẢ LẬP ĐỨT VẠCH 25cm tại quãng đường %.2fm!\n", legProgressM);
      } else {
        Serial.println("\n⚠️ [CHÚ Ý]: Hãy bấm 's' cho xe chạy trước, sau đó gõ 'g' để thử vượt vạch đứt!");
      }
    }
    else if (ch == '1') {
      autoLoopEnabled = false;
      startMission(LEG_AB);
    } else if (ch == '2') {
      autoLoopEnabled = false;
      startMission(LEG_BC);
    } else if (ch == '3') {
      autoLoopEnabled = false;
      startMission(LEG_CD);
    } else if (ch == '4') {
      autoLoopEnabled = false;
      startMission(LEG_DA);
    } else if (ch == 'n') {
      advanceToNextLeg();
    } else if (ch == 'a') {
      autoLoopEnabled = !autoLoopEnabled;
      Serial.printf("\n[SERIAL KEY]: Chế độ Vòng Lặp Tự Động: %s\n", autoLoopEnabled ? "BẬT" : "TẮT");
      if (autoLoopEnabled && !isRunning) startMission(currentLeg);
    } else if (ch == 'c') {
      Serial.println("\n[SERIAL KEY]: CALIB LẠI GYRO...");
      calibrateGyroZ(400);
      yawDeg = legTargetHeadingDeg;
    }
  }

  // Nút BOOT (GPIO 0)
  int reading = digitalRead(BUTTON_BOOT_PIN);
  static int lastBtn = HIGH;
  static unsigned long lastDeb = 0;
  if (reading != lastBtn) lastDeb = millis();
  if ((millis() - lastDeb) > 50) {
    if (reading == LOW && lastBtn == HIGH) {
      if (isRunning) {
        stopMotors();
        robotStateStr = "STOPPED";
        isRunning = false;
      } else {
        startMission(currentLeg);
      }
    }
  }
  lastBtn = reading;

  vTaskDelay(pdMS_TO_TICKS(50));
}
