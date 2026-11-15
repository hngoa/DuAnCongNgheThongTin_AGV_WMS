#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <math.h>
#include <esp_arduino_version.h>
#include "soc/usb_serial_jtag_reg.h"
#include "driver/gpio.h"

/*
 * ===================================================================================
 * AGV_Ver2 - TEST 09: ĐIỀU HƯỚNG ĐỒ THỊ 2 CHIỀU & BẢO MẬT WMS RBAC (MANG TẢI NẶNG)
 * -----------------------------------------------------------------------------------
 * ĐẶC TẢ TÍCH HỢP TOÀN DIỆN:
 *   1. KẾ THỪA 100% LOGIC DI CHUYỂN BẢN CHUẨN THỰC NGHIỆM:
 *      - Xoay tại trạm (SUB_TURNING_FOR_MISSION): Tiệm cận |err| <= 35°, xoay >= 20°,
 *        bắt đúng vạch 0b01110, triệt tiêu 100% hiện tượng bắt nhầm nhánh khi quay đầu 180°.
 *      - Chiều quay động cơ chuẩn xác: err < 0 -> Quay Trái (-turnSpeed, turnSpeed),
 *        err > 0 -> Quay Phải (turnSpeed, -turnSpeed).
 *      - Quét tìm vạch đảo chiều (SUB_TURNING_REVERSE_SEARCH) khi quay lố.
 *      - Tiếp cận góc trạm chuẩn hình học L = 23cm (bò thêm đúng khoảng cách trục bánh tới mắt).
 *      - Cầu nối quán tính vượt đứt đoạn (Dead-Reckoning Virtual Line Bridge) khóa Yaw con quay.
 *
 *   2. TỐC ĐỘ & XUNG PWM NÂNG CẤP CHO XE MANG TẢI (LOAD PROFILE V2.1):
 *      - straightPwm = 240, curvePwm = 205, brakePwm = 212, turnPwm = 200, bridgePwm = 222.
 *      - minMovePwm = 190: Giữ mô-men tối thiểu thắng hoàn toàn ma sát tải, chống ì chết máy.
 *      - MAX_BRIDGE_DISTANCE_M = 0.90m, MAX_BRIDGE_TIME_MS = 5000ms.
 *
 *   3. TÍNH NĂNG KẾT NỐI & BẢO MẬT MỚI:
 *      - Tự động phân giải Server qua mDNS (agv-wms.local) và phát hiện beacon UDP Port 5005.
 *      - Xác thực bảo mật thiết bị cấp quyền RBAC qua Header X-Device-Token.
 *      - Đồng bộ động kích thước Bản đồ (map_w, map_h) thời gian thực từ Server WMS.
 *      - Trích xuất lệnh cmd chính xác tránh nhầm chuỗi STOP trong trạng thái hệ thống.
 *      - Quản lý năng lượng INA226 trên I2C1 (GPIO 8, 9) gửi kèm telemetry.
 * ===================================================================================
 */

// --- 1. CẤU HÌNH KẾT NỐI WI-FI, TÊN MIỀN mDNS & BẢO MẬT TOKEN ---
const char* wifi_ssid        = "LAU 3";
const char* wifi_pass        = "777777777";
const char* mdns_hostname    = "agv-wms";                         // Tên miền mDNS duy nhất: agv-wms.local
const char* agv_device_token = "agv_sec_robot_v2_7f8a9b";         // Mã khóa bảo mật RBAC của Robot AGV
String      serverUrl        = "http://192.168.1.8:5000/api/telemetry"; // URL khởi đầu, tự cập nhật qua mDNS/UDP
String      lastKnownServerUrl = "http://192.168.1.8:5000/api/telemetry";
WiFiUDP     udpDiscovery;
#define     UDP_DISCOVERY_PORT 5005

// --- 2. SƠ ĐỒ CHÂN PHẦN CỨNG ---
#define I2C_SDA_PIN          4
#define I2C_SCL_PIN          5
#define MPU6050_ADDR         0x68

// --- 2.1. CẢM BIẾN DÒNG/ÁP & QUẢN LÝ NĂNG LƯỢNG INA226 (I2C1 - GPIO 8 & 9) ---
#define INA_SDA_PIN          8
#define INA_SCL_PIN          9
#define INA226_ADDR          0x40
TwoWire I2C_INA = TwoWire(1);

#define INA226_REG_CONFIG       0x00
#define INA226_REG_SHUNTVOLTAGE 0x01
#define INA226_REG_BUSVOLTAGE   0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIBRATION  0x05

bool  inaDetected            = false;
float rShunt                 = 0.100f; // 0.1 Ohm (Module R100)
volatile float liveBusV      = 0.0f;
volatile float liveCurrentA  = 0.0f;
volatile float livePowerW    = 0.0f;
volatile float liveEnergyMah = 0.0f;
volatile float liveEnergyWh  = 0.0f;
unsigned long  lastPowerCalcUs = 0;

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

#define LINE_L2_PIN          48
#define LINE_L1_PIN          47
#define LINE_C_PIN           21
#define LINE_R1_PIN          20
#define LINE_R2_PIN          19

// --- 3. THÔNG SỐ VẬT LÝ & TỐC ĐỘ CHO XE MANG TẢI ---
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

// Tốc độ điều khiển thích ứng (Đã nâng công suất chịu tải cho xe mang tải)
int   straightPwm            = 240; // Tốc độ chạy thẳng cao khi mang tải (trước: 215)
int   curvePwm               = 205; // Tốc độ ôm chỉnh lệch line khi mang tải (trước: 178)
int   brakePwm               = 212; // Tốc độ tiếp cận trạm có tải, chống ì chết máy (trước: 182)
int   turnPwm                = 200; // Tốc độ xoay tại chỗ khỏe khi mang tải (trước: 175)
int   bridgePwm              = 222; // Tốc độ vượt vạch đứt theo Map khi mang tải (trước: 192)
int   minMovePwm             = 190; // Ngưỡng PWM tối thiểu giữ mô-men thắng ma sát tải
int   maxPwm                 = 255; // Công suất tối đa 100% duty cycle (trước: 240)

// Hệ số PD bám line
float Kp_line                = 36.0f;
float Kd_line                = 15.0f;

// --- 4. THAM SỐ VƯỢT ĐỨT ĐOẠN / VẠCH MỜ CHO XE CÓ TẢI ---
const float MAX_BRIDGE_DISTANCE_M    = 0.90f; // Nâng lên 0.90m để vượt trọn vẹn khoảng đứt vạch thực tế
const unsigned long MAX_BRIDGE_TIME_MS = 5000; // 5000ms an toàn khi xe mang tải chạy chậm
float gapStartDistM                  = 0.0f;
float gapHeadingHoldDeg              = 0.0f;
float lastGapHeadingErr              = 0.0f;
bool  inGapBridgeMode                = false;
unsigned long gapStartTimeMs         = 0;
int   consecutiveZeroLineFrames      = 0;

bool  simulateGapActive              = false;
float simGapStartLegDistM            = 0.0f;
const float SIM_GAP_LENGTH_M         = 0.25f;

// --- 5. MÔ HÌNH ĐỒ THỊ 8 CHẶNG 2 CHIỀU LINH HOẠT & TỌA ĐỘ TRẠM ---
enum StationId {
  STATION_A = 0,
  STATION_B = 1,
  STATION_C = 2,
  STATION_D = 3,
  STATION_UNKNOWN = 4
};

enum LegId {
  LEG_AB = 0, // Xuôi A -> B (1.51m, +90°)
  LEG_BC = 1, // Xuôi B -> C (1.54m, 0°)
  LEG_CD = 2, // Xuôi C -> D (1.51m, -90°)
  LEG_DA = 3, // Xuôi D -> A (1.54m, 180°)
  LEG_BA = 4, // Ngược B -> A (1.51m, -90°)
  LEG_CB = 5, // Ngược C -> B (1.54m, 180°)
  LEG_DC = 6, // Ngược D -> C (1.51m, +90°)
  LEG_AD = 7, // Ngược A -> D (1.54m, 0°)
  LEG_NONE = 8
};

struct LegDef {
  StationId fromStation;
  StationId toStation;
  float targetDistM;
  float targetHeadingDeg;
  const char* name;
};

struct StationCoord {
  float x;
  float y;
};

StationCoord STATION_COORDS[4] = {
  { 0.00f, 0.00f }, // STATION_A (0.00m, 0.00m)
  { 0.00f, 1.51f }, // STATION_B (0.00m, 1.51m)
  { 1.54f, 1.51f }, // STATION_C (1.54m, 1.51m)
  { 1.54f, 0.00f }  // STATION_D (1.54m, 0.00m)
};

LegDef LEG_TABLE[8] = {
  { STATION_A, STATION_B, 1.51f,  90.0f, "A ➔ B" },
  { STATION_B, STATION_C, 1.54f,   0.0f, "B ➔ C" },
  { STATION_C, STATION_D, 1.51f, -90.0f, "C ➔ D" },
  { STATION_D, STATION_A, 1.54f, 180.0f, "D ➔ A" },
  { STATION_B, STATION_A, 1.51f, -90.0f, "B ➔ A" },
  { STATION_C, STATION_B, 1.54f, 180.0f, "C ➔ B" },
  { STATION_D, STATION_C, 1.51f,  90.0f, "D ➔ C" },
  { STATION_A, STATION_D, 1.54f,   0.0f, "A ➔ D" }
};

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

float activeMapWidthM  = 1.54f;
float activeMapLengthM = 1.51f;

void syncMapDimensionsFromServer(float w, float h) {
  if (w < 0.30f || h < 0.30f || w > 20.0f || h > 20.0f) return;
  if (fabsf(w - activeMapWidthM) < 0.005f && fabsf(h - activeMapLengthM) < 0.005f) return;

  portENTER_CRITICAL(&stateMux);
  activeMapWidthM  = w;
  activeMapLengthM = h;

  STATION_COORDS[STATION_A] = { 0.00f, 0.00f };
  STATION_COORDS[STATION_B] = { 0.00f, h     };
  STATION_COORDS[STATION_C] = { w,     h     };
  STATION_COORDS[STATION_D] = { w,     0.00f };

  LEG_TABLE[LEG_AB].targetDistM = h;
  LEG_TABLE[LEG_BC].targetDistM = w;
  LEG_TABLE[LEG_CD].targetDistM = h;
  LEG_TABLE[LEG_DA].targetDistM = w;
  LEG_TABLE[LEG_BA].targetDistM = h;
  LEG_TABLE[LEG_CB].targetDistM = w;
  LEG_TABLE[LEG_DC].targetDistM = h;
  LEG_TABLE[LEG_AD].targetDistM = w;
  portEXIT_CRITICAL(&stateMux);

  Serial.printf("\n🗺️ [ĐỒNG BỘ MAP SERVER]: Đã cập nhật bản đồ dẫn đường %.2fm x %.2fm!\n", w, h);
}

// Hàng đợi lộ trình đa chặng (Multi-leg Queue)
LegId legQueue[4];
int   queueLength = 0;
int   queueIndex  = 0;

volatile StationId     currentStation      = STATION_A;
volatile LegId         currentLeg          = LEG_AB;
volatile float         legTargetDistM      = 1.51f;
volatile float         legTargetHeadingDeg = 90.0f;
volatile float         startTurnYaw        = 0.0f;
volatile unsigned long turnSearchStartMs   = 0;

// Trạng thái FSM
enum RobotSubState {
  SUB_STOPPED,
  SUB_TURNING_FOR_MISSION,
  SUB_TURNING_REVERSE_SEARCH,
  SUB_TURNING_MANUAL,
  SUB_LINE_FOLLOWING,
  SUB_DEAD_RECKONING_BRIDGE,
  SUB_MOTOR_TEST,
  SUB_PAUSE_AT_STATION
};

volatile RobotSubState robotSubState       = SUB_STOPPED;
volatile bool          autoLoopEnabled     = false;
volatile bool          autoLoopReverse     = false;
volatile unsigned long stationPauseUntilMs = 0;

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

volatile long rawTicksL      = 0;
volatile long rawTicksR      = 0;
long prevTicksL              = 0;
long prevTicksR              = 0;

bool  mpuConnected           = false;
float gyroZOffset            = -1.31f;

float lastValidError         = 0.0f;
float lastLineError          = 0.0f;
unsigned long lineLostTimeMs = 0;

float smoothedBaseSpeed      = 0.0f;
const float MAX_RAMP_STEP    = 6.0f; // Tăng tốc êm nhưng dứt khoát khi mang tải (600 PWM/s)

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
// HÀM KHỞI CHẠY CHẶNG BẤT KỲ TRONG 8 CHẶNG (GIỮ NGUYÊN 100% LOGIC BẢN CŨ)
// ===================================================================================
void startMission(LegId leg) {
  currentLeg = leg;
  currentStation = LEG_TABLE[leg].fromStation;
  legTargetDistM = LEG_TABLE[leg].targetDistM;
  legTargetHeadingDeg = LEG_TABLE[leg].targetHeadingDeg;

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
    Serial.printf("\n>> [BẮT ĐẦU CHẶNG: %s] Yaw hiện tại = %.1f° | Cần xoay %+.1f° tới %.1f°...\n",
                  LEG_TABLE[leg].name, yawDeg, headingErr, legTargetHeadingDeg);
  } else {
    prevTicksL = globalTicksL;
    prevTicksR = globalTicksR;
    robotSubState = SUB_LINE_FOLLOWING;
    robotStateStr = "RUNNING_FAST";
    isRunning = true;
    Serial.printf("\n>> [BẮT ĐẦU CHẶNG: %s] Đúng hướng %.1f° -> Khởi hành bám line!\n",
                  LEG_TABLE[leg].name, legTargetHeadingDeg);
  }
}

// Lập kế hoạch lộ trình Điểm - Điểm tự động (GIỮ NGUYÊN BẢN CŨ)
void planAndExecuteRoute(StationId fromSt, StationId toSt) {
  if (fromSt == toSt) {
    Serial.printf("\n>> [THÔNG BÁO] Xe đang ở sẵn trạm đích: %d!\n", (int)toSt);
    return;
  }

  queueLength = 0;
  queueIndex = 0;

  // 1. Kiểm tra chặng trực tiếp kề bên
  for (int i = 0; i < 8; i++) {
    if (LEG_TABLE[i].fromStation == fromSt && LEG_TABLE[i].toStation == toSt) {
      legQueue[0] = (LegId)i;
      queueLength = 1;
      break;
    }
  }

  // 2. Nếu là trạm đối diện (cách 2 chặng): Lập đường đi 2 chặng tối ưu
  if (queueLength == 0) {
    if (fromSt == STATION_A && toSt == STATION_C) {
      legQueue[0] = LEG_AB; legQueue[1] = LEG_BC; queueLength = 2;
    } else if (fromSt == STATION_C && toSt == STATION_A) {
      legQueue[0] = LEG_CD; legQueue[1] = LEG_DA; queueLength = 2;
    } else if (fromSt == STATION_B && toSt == STATION_D) {
      legQueue[0] = LEG_BC; legQueue[1] = LEG_CD; queueLength = 2;
    } else if (fromSt == STATION_D && toSt == STATION_B) {
      legQueue[0] = LEG_DA; legQueue[1] = LEG_AB; queueLength = 2;
    }
  }

  if (queueLength > 0) {
    Serial.printf("\n🚀 [ĐIỀU PHỐI ĐIỂM - ĐIỂM] Lộ trình %d chặng: Từ Trạm %c ➔ Trạm %c\n",
                  queueLength, "ABCD"[fromSt], "ABCD"[toSt]);
    startMission(legQueue[0]);
  }
}

// Chuyển sang chặng kế tiếp
void advanceToNextLeg() {
  if (queueLength > 0 && queueIndex + 1 < queueLength) {
    queueIndex++;
    startMission(legQueue[queueIndex]);
    return;
  }

  LegId nextLeg = LEG_AB;
  if (currentStation == STATION_A) nextLeg = LEG_AB;
  else if (currentStation == STATION_B) nextLeg = LEG_BC;
  else if (currentStation == STATION_C) nextLeg = LEG_CD;
  else if (currentStation == STATION_D) nextLeg = LEG_DA;

  startMission(nextLeg);
}

// Chuyển sang chặng lùi về (Ngược chiều)
void advanceToPrevLeg() {
  LegId prevLeg = LEG_BA;
  if (currentStation == STATION_B) prevLeg = LEG_BA;
  else if (currentStation == STATION_C) prevLeg = LEG_CB;
  else if (currentStation == STATION_D) prevLeg = LEG_DC;
  else if (currentStation == STATION_A) prevLeg = LEG_AD;

  startMission(prevLeg);
}

// Khởi chạy xoay thủ công góc deltaDeg
void startManualTurn(float deltaDeg) {
  legTargetHeadingDeg = normalizeAngleDeg(yawDeg + deltaDeg);
  startTurnYaw = yawDeg;
  robotSubState = SUB_TURNING_MANUAL;
  robotStateStr = "TURNING_MANUAL";
  isRunning = true;
  Serial.printf("\n>> [XOAY THỦ CÔNG]: Cần xoay %+.1f° -> Hướng mới: %.1f°\n", deltaDeg, legTargetHeadingDeg);
}

// ===================================================================================
// CẢM BIẾN DÒ LINE TCRT5000 (5 MẮT)
// ===================================================================================
void initLineSensors() {
  pinMode(LINE_L2_PIN, INPUT);
  pinMode(LINE_L1_PIN, INPUT);
  pinMode(LINE_C_PIN,  INPUT);
  pinMode(LINE_R1_PIN, INPUT);
  pinMode(LINE_R2_PIN, INPUT);
}

int readLineSensors(int &sL2, int &sL1, int &sC, int &sR1, int &sR2) {
  if (simulateGapActive) {
    if ((legProgressM - simGapStartLegDistM) <= SIM_GAP_LENGTH_M) {
      sL2 = sL1 = sC = sR1 = sR2 = 0;
      return 0;
    } else {
      simulateGapActive = false;
      Serial.println("\n⚡ [TEST ĐỨT VẠCH]: Kết thúc đoạn giả lập 25cm -> Trở lại cảm biến thật!");
    }
  }

  sL2 = digitalRead(LINE_L2_PIN) == HIGH ? 1 : 0;
  sL1 = digitalRead(LINE_L1_PIN) == HIGH ? 1 : 0;
  sC  = digitalRead(LINE_C_PIN)  == HIGH ? 1 : 0;
  sR1 = digitalRead(LINE_R1_PIN) == HIGH ? 1 : 0;
  sR2 = digitalRead(LINE_R2_PIN) == HIGH ? 1 : 0;

  return (sL2 << 4) | (sL1 << 3) | (sC << 2) | (sR1 << 1) | sR2;
}

// ===================================================================================
// IMU MPU-6050
// ===================================================================================
int16_t readReg16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom((int)addr, 2) != 2) return 0;
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

void writeReg8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void calibrateGyroZ(int samples = 400) {
  if (!mpuConnected) return;
  stopMotors();
  delay(100);

  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readReg16(MPU6050_ADDR, 0x47);
    delay(2);
  }
  gyroZOffset = ((float)sum / samples) / 32.8f;
  Serial.printf("\n>> [GYRO CALIB]: Offset Z = %.3f deg/s\n", gyroZOffset);
}

void initMPU() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  Wire.setTimeOut(25);

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    mpuConnected = true;
    writeReg8(MPU6050_ADDR, 0x6B, 0x00);
    delay(50);
    writeReg8(MPU6050_ADDR, 0x1A, 0x03);
    writeReg8(MPU6050_ADDR, 0x1B, 0x10);
    calibrateGyroZ(400);
    Serial.println("✅ [MPU-6050]: Khởi tạo thành công!");
  } else {
    mpuConnected = false;
    Serial.println("⚠️ [MPU-6050]: Không tìm thấy MPU-6050! Chạy thuần Encoder.");
  }
}

// ===================================================================================
// CẢM BIẾN DÒNG/ÁP INA226 (I2C1: GPIO 8 & 9)
// ===================================================================================
bool writeInaReg16(uint8_t reg, uint16_t value) {
  I2C_INA.beginTransmission(INA226_ADDR);
  I2C_INA.write(reg);
  I2C_INA.write((uint8_t)(value >> 8));
  I2C_INA.write((uint8_t)(value & 0xFF));
  return (I2C_INA.endTransmission() == 0);
}

bool readInaReg16(uint8_t reg, uint16_t &value) {
  I2C_INA.beginTransmission(INA226_ADDR);
  I2C_INA.write(reg);
  if (I2C_INA.endTransmission(false) != 0) return false;
  if (I2C_INA.requestFrom((int)INA226_ADDR, 2) != 2) return false;
  uint8_t msb = I2C_INA.read();
  uint8_t lsb = I2C_INA.read();
  value = ((uint16_t)msb << 8) | (uint16_t)lsb;
  return true;
}

void initINA226() {
  pinMode(INA_SDA_PIN, INPUT_PULLUP);
  pinMode(INA_SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(INA_SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(INA_SCL_PIN, LOW);  delayMicroseconds(5);
  }
  digitalWrite(INA_SCL_PIN, HIGH);
  pinMode(INA_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(20);

  I2C_INA.begin(INA_SDA_PIN, INA_SCL_PIN, 100000);
  I2C_INA.setTimeOut(25);

  I2C_INA.beginTransmission(INA226_ADDR);
  if (I2C_INA.endTransmission() == 0) {
    inaDetected = true;
    writeInaReg16(INA226_REG_CONFIG, 0x4127);
    uint16_t calVal = (uint16_t)(0.00512f / (0.0001f * rShunt));
    writeInaReg16(INA226_REG_CALIBRATION, calVal);
    Serial.println("✅ [INA226]: Khởi tạo cảm biến năng lượng trên GPIO 8, 9 THÀNH CÔNG!");
  } else {
    inaDetected = false;
    Serial.println("⚠️ [INA226]: Không thấy cảm biến INA226 trên GPIO 8, 9 (Chạy chế độ điều hướng thuần túy).");
  }
}

void updatePowerSensors() {
  if (!inaDetected) return;
  uint16_t rawBus = 0;
  uint16_t rawShunt = 0;
  if (!readInaReg16(INA226_REG_BUSVOLTAGE, rawBus) || !readInaReg16(INA226_REG_SHUNTVOLTAGE, rawShunt)) return;

  unsigned long nowUs = micros();
  float dtSec = (float)(nowUs - lastPowerCalcUs) / 1000000.0f;
  if (dtSec < 0.0f || dtSec > 1.0f) dtSec = 0.05f;
  lastPowerCalcUs = nowUs;

  liveBusV = (float)rawBus * 0.00125f;
  int16_t signedShunt = (int16_t)rawShunt;
  float shunt_mV = (float)signedShunt * 0.0025f;
  liveCurrentA = (shunt_mV / 1000.0f) / rShunt;
  livePowerW = liveBusV * liveCurrentA;
  if (livePowerW < 0) livePowerW = -livePowerW;

  if (liveCurrentA > 0.005f) {
    liveEnergyMah += (liveCurrentA * 1000.0f) * (dtSec / 3600.0f);
    liveEnergyWh  += livePowerW * (dtSec / 3600.0f);
  }
}

// ===================================================================================
// CORE 1: ĐIỀU KHIỂN THỜI GIAN THỰC 100Hz (GIỮ 100% LOGIC DI CHUYỂN BẢN CHUẨN)
// ===================================================================================
void updateControlAndOdometry(float dt = 0.010f) {
  // 1. Cập nhật góc Yaw từ MPU-6050
  if (mpuConnected) {
    int16_t rGz = readReg16(MPU6050_ADDR, 0x47);
    float gzRaw = (rGz / 32.8f);

    if (!isRunning && (testMotorUntilMs < millis()) && (robotSubState == SUB_STOPPED || robotSubState == SUB_PAUSE_AT_STATION)) {
      gyroZOffset = 0.995f * gyroZOffset + 0.005f * gzRaw;
    } else {
      float gz = (gzRaw - gyroZOffset);
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

  bool isTurningInPlace = (robotSubState == SUB_TURNING_FOR_MISSION ||
                           robotSubState == SUB_TURNING_REVERSE_SEARCH ||
                           robotSubState == SUB_TURNING_MANUAL);
  if (isTurningInPlace) {
    deltaS = 0.0f;
  }

  currentSpeed = (deltaS / dt) * 100.0f;
  totalDistM += fabsf(deltaS);

  float rad = (yawDeg * PI) / 180.0f;
  odomX += deltaS * cosf(rad);
  odomY += deltaS * sinf(rad);

  // Cập nhật cảm biến năng lượng INA226 định kỳ (20Hz = mỗi 5 chu kỳ 100Hz)
  static int powerDiv = 0;
  if (++powerDiv >= 5) {
    powerDiv = 0;
    updatePowerSensors();
  }

  // 3. Đọc mắt cảm biến
  int sL2, sL1, sC, sR1, sR2;
  lineMask = readLineSensors(sL2, sL1, sC, sR1, sR2);
  int activeCount = sL2 + sL1 + sC + sR1 + sR2;

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 0: NGHỈ TẠI TRẠM (2 GIÂY TRONG CHẾ ĐỘ AUTO LOOP)
  // ---------------------------------------------------------------------------------
  if (robotSubState == SUB_PAUSE_AT_STATION) {
    setMotors(0, 0);
    if (millis() >= stationPauseUntilMs) {
      advanceToNextLeg();
    }
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 1A: XOAY TẠI TRẠM (GIỮ CHÍNH XÁC LOGIC KHÓA VẠCH BẢN CŨ)
  // ---------------------------------------------------------------------------------
  if (robotSubState == SUB_TURNING_FOR_MISSION) {
    float err = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
    float absErr = fabsf(err);
    float rotatedSoFar = fabsf(normalizeAngleDeg(yawDeg - startTurnYaw));

    // ĐIỀU KIỆN KHÓA VẠCH ĐƯỢC KIỂM CHỨNG BẢN CŨ:
    // 1. Đã xoay thoát khỏi vạch ban đầu (rotatedSoFar >= 20°)
    // 2. Góc hướng ĐÃ TIỆM CẬN GÓC ĐÍCH (|err| <= 35°) -> Triệt tiêu 100% bắt nhầm nhánh vuông khi quay 180°!
    // 3. Mắt giữa chạm vạch đen của chặng mới (0b01110)
    bool lineCaught = (absErr <= 35.0f) && (rotatedSoFar >= 20.0f) && ((lineMask & 0b01110) != 0);

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
      Serial.printf("\n🎯 [KHÓA VẠCH THÀNH CÔNG] Đã bắt đúng vạch chặng %s tại Yaw = %.1f° -> Khởi hành!\n",
                    LEG_TABLE[currentLeg].name, yawDeg);
      return;
    }

    // Nếu góc đã đến đích (|err| <= 2.5°):
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

    // Tốc độ xoay thích ứng theo bản mới (PWM cao để thắng ma sát tải)
    int turnSpeed = (absErr > 45.0f) ? turnPwm : ((absErr > 15.0f) ? (turnPwm - 10) : minMovePwm);
    turnSpeed = constrain(turnSpeed, minMovePwm, maxPwm);

    // Chiều quay CHUẨN: err > 0 (Yaw cần tăng) -> Quay Trái (CCW), err < 0 -> Quay Phải (CW)
    if (err > 0.0f) setMotors(-turnSpeed, turnSpeed); // Quay Trái (CCW)
    else setMotors(turnSpeed, -turnSpeed);            // Quay Phải (CW)
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 1B: QUÉT TÌM VẠCH NẾU QUAY LỐ (GIỮ NGUYÊN BẢN CŨ)
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
    int sweepSpeed = minMovePwm;
    if (err > 0.0f) setMotors(sweepSpeed, -sweepSpeed); // Quét ngược chiều quay (CW)
    else setMotors(-sweepSpeed, sweepSpeed);            // Quét ngược chiều quay (CCW)
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 1C: XOAY THỦ CÔNG (GIỮ NGUYÊN BẢN CŨ)
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

    int turnSpeed = (absErr > 40.0f) ? turnPwm : ((absErr > 15.0f) ? (turnPwm - 8) : minMovePwm);
    turnSpeed = constrain(turnSpeed, minMovePwm, maxPwm);
    if (err > 0.0f) setMotors(-turnSpeed, turnSpeed); // Quay Trái (CCW)
    else setMotors(turnSpeed, -turnSpeed);            // Quay Phải (CW)
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 2: TEST ĐỘNG CƠ 1 GIÂY
  // ---------------------------------------------------------------------------------
  if (testMotorUntilMs > millis()) {
    setMotors(230, 230);
    robotStateStr = "MOTOR_TEST";
    return;
  }

  if (!isRunning || robotSubState == SUB_STOPPED) {
    stopMotors();
    robotStateStr = "STOPPED";
    return;
  }

  // ---------------------------------------------------------------------------------
  // TRẠNG THÁI 3: THEO DÕI TIẾN ĐỘ CHẶNG & TIẾP CẬN GÓC TRẠM (LOGIC CHUẨN BẢN CŨ)
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

      // Cập nhật trạm hiện tại chính xác & neo tọa độ chuẩn
      currentStation = LEG_TABLE[currentLeg].toStation;
      odomX = STATION_COORDS[currentStation].x;
      odomY = STATION_COORDS[currentStation].y;
      yawDeg = legTargetHeadingDeg;

      Serial.printf("\n🏁 [ĐÃ TỚI ĐÍCH CHẶNG: %s] Xe đang ở TRẠM: %c | Tọa độ: X=%.3fm, Y=%.3fm, Yaw=%.1f°\n",
                    LEG_TABLE[currentLeg].name, "ABCD"[currentStation], odomX, odomY, yawDeg);

      // Nếu có hàng đợi đa chặng (Point-to-Point) hoặc Auto-Loop:
      if ((queueLength > 0 && queueIndex + 1 < queueLength) || autoLoopEnabled) {
        stationPauseUntilMs = millis() + 2000;
        robotSubState = SUB_PAUSE_AT_STATION;
        robotStateStr = "PAUSED_AT_STATION";
        isRunning = true;
      } else {
        queueLength = 0;
        robotSubState = SUB_STOPPED;
        isRunning = false;
      }
      return;
    }

    // Bò 23cm cuối: Duy trì xung cao (brakePwm 212 >= minMovePwm 190) để xe mang tải không bị khựng lại
    // Bù lái MPU chuẩn: Lệch trái (hErr < 0) -> bánh trái tăng tốc (base - u_h), bánh phải giảm tốc (base + u_h) -> Xe nắn về phải!
    float hErr = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
    float u_h = constrain(3.0f * hErr, -40.0f, 40.0f);
    leftPwm  = constrain((int)(brakePwm - u_h), minMovePwm, maxPwm);
    rightPwm = constrain((int)(brakePwm + u_h), minMovePwm, maxPwm);
    robotStateStr = "APPROACH_CORNER";
    setMotors(leftPwm, rightPwm);
    return;
  }

  // =================================================================================
  // GIAI ĐOẠN 3.2: XỬ LÝ VƯỢT ĐỨT ĐOẠN / VẠCH MỜ / BÁM LINE ROBUST (LOGIC BẢN CŨ)
  // =================================================================================
  if (activeCount == 0) {
    consecutiveZeroLineFrames++;

    if (consecutiveZeroLineFrames <= 3 && !inGapBridgeMode) {
      int u = constrain((int)(Kp_line * lastValidError * 0.35f), -35, 35);
      leftPwm  = constrain((int)(smoothedBaseSpeed + u), minMovePwm, maxPwm);
      rightPwm = constrain((int)(smoothedBaseSpeed - u), minMovePwm, maxPwm);
      robotStateStr = "LINE_FLICKER_FILTER";
      setMotors(leftPwm, rightPwm);
      return;
    }

    if (!inGapBridgeMode) {
      inGapBridgeMode = true;
      gapStartTimeMs = millis();
      gapStartDistM = totalDistM;
      gapHeadingHoldDeg = legTargetHeadingDeg;
      lastValidError = 0.0f; // Triệt tiêu sai số mép cũ, tránh bẻ lái giật khi bắt đầu vượt đứt đoạn
      lastGapHeadingErr = normalizeAngleDeg(gapHeadingHoldDeg - yawDeg);
      Serial.printf("\n⚡ [VƯỢT ĐỨT ĐOẠN] Kích hoạt Cầu Nối Quán Tính trên chặng %s! Khóa Yaw = %.1f°\n",
                    LEG_TABLE[currentLeg].name, gapHeadingHoldDeg);
    }

    float gapTraveled = totalDistM - gapStartDistM;

    if (gapTraveled > MAX_BRIDGE_DISTANCE_M || (millis() - gapStartTimeMs > MAX_BRIDGE_TIME_MS)) {
      stopMotors();
      robotSubState = SUB_STOPPED;
      robotStateStr = "GAP_TIMEOUT_STOP";
      isRunning = false;
      Serial.printf("\n❌ [DỪNG AN TOÀN] Vượt quá %.2fm không thấy vạch!\n", gapTraveled);
      return;
    }

    // KHÓA CỨNG HƯỚNG YAW CỦA CHẶNG: Dùng Gyro MPU-6050 giữ thẳng tuyệt đối qua khoảng đứt line
    // Đã sửa triệt để lỗi đảo dấu: hErr < 0 (Yaw > target, lệch trái) -> bánh trái tăng tốc (base - u_h), bánh phải giảm tốc (base + u_h) -> Xe giữ thẳng tắp!
    float hErr = normalizeAngleDeg(gapHeadingHoldDeg - yawDeg);
    float dHErr = (hErr - lastGapHeadingErr) / fmaxf(dt, 0.005f);
    lastGapHeadingErr = hErr;
    float u_h = constrain((3.2f * hErr) + (0.5f * dHErr), -45.0f, 45.0f);

    int targetBase = (remainingLegM < 0.40f) ? brakePwm : bridgePwm;
    if (targetBase > smoothedBaseSpeed) smoothedBaseSpeed = min((float)targetBase, smoothedBaseSpeed + MAX_RAMP_STEP);
    else if (targetBase < smoothedBaseSpeed) smoothedBaseSpeed = max((float)targetBase, smoothedBaseSpeed - MAX_RAMP_STEP);

    leftPwm  = constrain((int)(smoothedBaseSpeed - u_h), minMovePwm - 20, maxPwm);
    rightPwm = constrain((int)(smoothedBaseSpeed + u_h), minMovePwm - 20, maxPwm);
    robotStateStr = "DEAD_RECKONING_BRIDGE";
    setMotors(leftPwm, rightPwm);
    return;

  } else {
    consecutiveZeroLineFrames = 0;

    if (inGapBridgeMode) {
      float gapCrossedM = totalDistM - gapStartDistM;
      inGapBridgeMode = false;
      Serial.printf("\n🎯 [BẮT LẠI VẠCH] Đã vượt qua khoảng đứt %.2fm -> Tiếp tục bám line %s!\n",
                    gapCrossedM, LEG_TABLE[currentLeg].name);
    }

    lineLostTimeMs = 0;

    currentLineErr = (float)(-2*sL2 + -1*sL1 + 0*sC + 1*sR1 + 2*sR2) / (float)activeCount;
    lastValidError = currentLineErr;

    // Khử trôi Gyro động bằng vạch Line
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

void controlLoopTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    updateControlAndOdometry(0.010f);
  }
}

// ===================================================================================
// HÀM PHÂN GIẢI mDNS TÌM SERVER TỰ ĐỘNG (agv-wms.local:5000)
// ===================================================================================
bool resolveServerMdns(int maxAttempts = 2, int timeoutMs = 900) {
  Serial.printf(">> [mDNS Client]: Đang phân giải tên miền http://%s.local:5000...\n", mdns_hostname);
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    IPAddress mdnsIp = MDNS.queryHost(mdns_hostname, timeoutMs);
    if (mdnsIp != INADDR_NONE && mdnsIp.toString() != "0.0.0.0") {
      serverUrl = "http://" + mdnsIp.toString() + ":5000/api/telemetry";
      lastKnownServerUrl = serverUrl;
      Serial.printf("🎯 [mDNS THÀNH CÔNG - Lần %d]: Server IP = %s -> URL: %s\n", attempt, mdnsIp.toString().c_str(), serverUrl.c_str());
      return true;
    }
    delay(150);
  }
  if (lastKnownServerUrl.length() > 0 && serverUrl.length() == 0) {
    serverUrl = lastKnownServerUrl;
  }
  Serial.printf("⚠️ [mDNS]: Không nhận được gói Multicast '%s.local' -> Duy trì kết nối qua URL: %s\n",
                mdns_hostname, serverUrl.c_str());
  return false;
}

// ===================================================================================
// CORE 0: TELEMETRY & BẢO MẬT RBAC & ĐIỀU PHỐI ĐA TRẠM 2 CHIỀU
// ===================================================================================
void telemetryTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);         // Tắt chế độ ngủ Modem để chống rớt gói khi động cơ ăn dòng lớn
  WiFi.setAutoReconnect(true);  // Tự động kết nối lại ở tầng phần cứng
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
    WiFi.setSleep(false);
    Serial.printf("\n✅ [WI-FI]: Đã kết nối! IP = %s\n", WiFi.localIP().toString().c_str());

    // 1. Phân giải mDNS agv-wms.local
    if (MDNS.begin("agv-esp32")) {
      delay(200);
      resolveServerMdns(3, 1000);
    }

    // 2. Dự phòng UDP Discovery (Port 5005)
    udpDiscovery.begin(UDP_DISCOVERY_PORT);
    Serial.printf(">> [DÒ TÌM UDP]: Quét beacon UDP tại cổng %d...\n", UDP_DISCOVERY_PORT);
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1200) {
      int packetSize = udpDiscovery.parsePacket();
      if (packetSize) {
        char packetBuffer[255];
        int len = udpDiscovery.read(packetBuffer, 254);
        if (len > 0) packetBuffer[len] = 0;
        String s = String(packetBuffer);
        if (s.indexOf("AGV_SERVER") >= 0) {
          IPAddress srvIp = udpDiscovery.remoteIP();
          serverUrl = "http://" + srvIp.toString() + ":5000/api/telemetry";
          lastKnownServerUrl = serverUrl;
          Serial.printf("🎯 [TỰ ĐỘNG TÌM THẤY SERVER QUA UDP]: %s\n", serverUrl.c_str());
          break;
        }
      }
      delay(30);
    }
  } else {
    Serial.println("\n⚠️ [WI-FI]: Không kết nối được Wi-Fi -> Chạy nội bộ.");
  }

  HTTPClient http;
  http.setTimeout(1500);
  http.setReuse(true); // Giữ kết nối TCP Keep-Alive tránh cạn kiệt socket lwIP
  unsigned long lastMdnsRetryMs = 0;
  unsigned long lastUdpCheckMs = 0;
  unsigned long lastHttpErrMs = 0;
  unsigned long lastWifiRetryMs = 0;
  int consecutiveHttpErrors = 0;

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      // Quét kiểm tra beacon UDP định kỳ
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
              lastKnownServerUrl = serverUrl;
              Serial.printf("🎯 [CẬP NHẬT IP SERVER MỚI TỪ UDP]: %s\n", serverUrl.c_str());
            }
          }
        }
      }

      // Snapshot dữ liệu an toàn đa nhân (Thread-safe Snapshot)
      portENTER_CRITICAL(&stateMux);
      float snapX = odomX;
      float snapY = odomY;
      float snapYaw = yawDeg;
      float snapDist = totalDistM;
      float snapSpeed = currentSpeed;
      int   snapMask = lineMask;
      float snapLineErr = currentLineErr;
      long  snapTicksL = globalTicksL;
      long  snapTicksR = globalTicksR;
      int   snapPwmL = cmdPwmL;
      int   snapPwmR = cmdPwmR;
      StationId snapStation = currentStation;
      LegId snapLeg = currentLeg;
      bool  snapAutoLoop = autoLoopEnabled;
      String snapState = robotStateStr;
      portEXIT_CRITICAL(&stateMux);

      String payload = "{";
      payload += "\"x\":" + String(snapX, 3) + ",";
      payload += "\"y\":" + String(snapY, 3) + ",";
      payload += "\"yaw\":" + String(snapYaw, 1) + ",";
      payload += "\"dist\":" + String(snapDist, 3) + ",";
      payload += "\"speed\":" + String(snapSpeed, 1) + ",";
      payload += "\"line_mask\":" + String(snapMask) + ",";
      payload += "\"line_err\":" + String(snapLineErr, 2) + ",";
      payload += "\"ticks_l\":" + String(snapTicksL) + ",";
      payload += "\"ticks_r\":" + String(snapTicksR) + ",";
      payload += "\"pwm_l\":" + String(snapPwmL) + ",";
      payload += "\"pwm_r\":" + String(snapPwmR) + ",";
      payload += "\"station\":\"" + String("ABCD"[snapStation]) + "\",";
      payload += "\"leg\":\"" + String(LEG_TABLE[snapLeg].name) + "\",";
      payload += "\"ack_id\":" + String(lastAckId) + ",";
      payload += "\"auto_loop\":" + String(snapAutoLoop ? "true" : "false") + ",";
      payload += "\"voltage\":" + String(liveBusV, 3) + ",";
      payload += "\"current\":" + String(liveCurrentA, 3) + ",";
      payload += "\"power\":" + String(livePowerW, 3) + ",";
      payload += "\"energy_mah\":" + String(liveEnergyMah, 1) + ",";
      payload += "\"ina_ok\":" + String(inaDetected ? "true" : "false") + ",";
      payload += "\"state\":\"" + snapState + "\"";
      payload += "}";

      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");
      http.addHeader("X-Device-Token", agv_device_token); // Xác thực bảo mật RBAC với Server WMS
      int httpCode = http.POST(payload);

      if (httpCode == 200) {
        consecutiveHttpErrors = 0;
        lastKnownServerUrl = serverUrl;
        String resp = http.getString();

        // Đồng bộ kích thước Bản đồ động (map_w, map_h) thời gian thực từ Server
        int mwIdx = resp.indexOf("\"map_w\":");
        int mhIdx = resp.indexOf("\"map_h\":");
        if (mwIdx >= 0 && mhIdx >= 0) {
          int wStart = mwIdx + 8;
          while (wStart < (int)resp.length() && resp.charAt(wStart) == ' ') wStart++;
          int hStart = mhIdx + 8;
          while (hStart < (int)resp.length() && resp.charAt(hStart) == ' ') hStart++;
          float srvW = resp.substring(wStart).toFloat();
          float srvH = resp.substring(hStart).toFloat();
          syncMapDimensionsFromServer(srvW, srvH);
        }

        int idIdx = resp.indexOf("\"cmd_id\":");
        int incomingId = 0;
        if (idIdx >= 0) {
          int numStart = idIdx + 9;
          while (numStart < (int)resp.length() && resp.charAt(numStart) == ' ') numStart++;
          incomingId = resp.substring(numStart).toInt();
        }

        // Trích xuất chính xác chuỗi cmd từ JSON (Tránh quét nhầm substring)
        String cmdStr = "";
        int cmdIdx = resp.indexOf("\"cmd\":");
        if (cmdIdx >= 0) {
          int valStart = cmdIdx + 6;
          while (valStart < (int)resp.length() && resp.charAt(valStart) == ' ') valStart++;
          if (valStart < (int)resp.length() && resp.charAt(valStart) == '\"') {
            int valEnd = resp.indexOf('\"', valStart + 1);
            if (valEnd > valStart) {
              cmdStr = resp.substring(valStart + 1, valEnd);
            }
          }
        }

        if (incomingId > 0 && incomingId != lastAckId && cmdStr.length() > 0) {
          lastAckId = incomingId;
          Serial.printf("\n>> [LỆNH MẠNG #%d]: cmd='%s'\n", incomingId, cmdStr.c_str());

          // 1. Lệnh cơ bản
          if (cmdStr == "START") {
            startMission(currentLeg);
          } else if (cmdStr == "STOP") {
            portENTER_CRITICAL(&stateMux);
            autoLoopEnabled = false;
            queueLength = 0;
            robotSubState = SUB_STOPPED;
            isRunning = false;
            robotStateStr = "STOPPED";
            portEXIT_CRITICAL(&stateMux);
            stopMotors();
          } else if (cmdStr == "RESET_ODOM") {
            stopMotors();
            noInterrupts(); rawTicksL = 0; rawTicksR = 0; interrupts();
            portENTER_CRITICAL(&stateMux);
            odomX = 0.0f; odomY = 0.0f; yawDeg = 90.0f; totalDistM = 0.0f; legProgressM = 0.0f;
            currentLeg = LEG_AB;
            currentStation = STATION_A;
            autoLoopEnabled = false;
            queueLength = 0;
            prevTicksL = 0; prevTicksR = 0;
            robotSubState = SUB_STOPPED;
            isRunning = false;
            robotStateStr = "STOPPED";
            portEXIT_CRITICAL(&stateMux);
            Serial.println(">> [LỆNH]: RESET ODOMETRY VỀ GỐC A (0,0) 90°!");
          } else if (cmdStr == "CALIBRATE_GYRO") {
            calibrateGyroZ(400);
            yawDeg = 90.0f;
          } 
          // 2. Chạy từng chặng cụ thể trong 8 chặng 2 chiều
          else if (cmdStr == "MISSION:AB" || cmdStr == "MISSION:M1") {
            autoLoopEnabled = false; startMission(LEG_AB);
          } else if (cmdStr == "MISSION:BC" || cmdStr == "MISSION:M2") {
            autoLoopEnabled = false; startMission(LEG_BC);
          } else if (cmdStr == "MISSION:CD" || cmdStr == "MISSION:M3") {
            autoLoopEnabled = false; startMission(LEG_CD);
          } else if (cmdStr == "MISSION:DA" || cmdStr == "MISSION:M4") {
            autoLoopEnabled = false; startMission(LEG_DA);
          } else if (cmdStr == "MISSION:BA") {
            autoLoopEnabled = false; startMission(LEG_BA);
          } else if (cmdStr == "MISSION:CB") {
            autoLoopEnabled = false; startMission(LEG_CB);
          } else if (cmdStr == "MISSION:DC") {
            autoLoopEnabled = false; startMission(LEG_DC);
          } else if (cmdStr == "MISSION:AD") {
            autoLoopEnabled = false; startMission(LEG_AD);
          }
          // 3. Điều hướng tiến / lùi tuần tự
          else if (cmdStr == "MISSION:NEXT") {
            advanceToNextLeg();
          } else if (cmdStr == "MISSION:PREV") {
            advanceToPrevLeg();
          }
          // 4. Lập lộ trình Điểm - Điểm tự động (GO_TO)
          else if (cmdStr == "GO_TO:A") {
            planAndExecuteRoute(currentStation, STATION_A);
          } else if (cmdStr == "GO_TO:B") {
            planAndExecuteRoute(currentStation, STATION_B);
          } else if (cmdStr == "GO_TO:C") {
            planAndExecuteRoute(currentStation, STATION_C);
          } else if (cmdStr == "GO_TO:D") {
            planAndExecuteRoute(currentStation, STATION_D);
          }
          // 5. Vòng lặp tuần hoàn xuôi / ngược
          else if (cmdStr == "AUTO_LOOP:ON") {
            autoLoopEnabled = true;
            autoLoopReverse = false;
            startMission(currentLeg);
          } else if (cmdStr == "AUTO_LOOP:REVERSE") {
            autoLoopEnabled = true;
            autoLoopReverse = true;
            advanceToPrevLeg();
          } else if (cmdStr == "AUTO_LOOP:OFF") {
            autoLoopEnabled = false;
          }
          // 6. Xoay thủ công & Giả lập đứt vạch (Trái CCW = -90°, Phải CW = +90°)
          else if (cmdStr == "TURN_LEFT_90") {
            startManualTurn(-90.0f);
          } else if (cmdStr == "TURN_RIGHT_90") {
            startManualTurn(90.0f);
          } else if (cmdStr == "TURN_180") {
            startManualTurn(180.0f);
          } else if (cmdStr == "SIMULATE_GAP") {
            simulateGapActive = true;
            simGapStartLegDistM = legProgressM;
          } else if (cmdStr == "RESET_ENERGY") {
            liveEnergyMah = 0.0f;
            liveEnergyWh = 0.0f;
            Serial.println(">> [LỆNH]: ĐÃ ĐẶT LẠI NĂNG LƯỢNG TIÊU THỤ VỀ 0.0 mAh!");
          }
        }
      } else {
        consecutiveHttpErrors++;
        if (millis() - lastHttpErrMs > 3000) {
          lastHttpErrMs = millis();
          if (httpCode == 401) {
            Serial.printf("❌ [LỖI BẢO MẬT 401]: Mã khóa X-Device-Token bị từ chối bởi Server!\n");
          } else {
            Serial.printf("⚠️ [CẢNH BÁO MẠNG]: HTTP %d tại %s (Lỗi liên tiếp: %d)\n",
                          httpCode, serverUrl.c_str(), consecutiveHttpErrors);
          }
        }
        if (consecutiveHttpErrors >= 15 && (millis() - lastMdnsRetryMs > 5000)) {
          lastMdnsRetryMs = millis();
          consecutiveHttpErrors = 0;
          resolveServerMdns(1, 600);
        }
      }
      http.end();
    } else {
      if (millis() - lastWifiRetryMs > 5000) {
        lastWifiRetryMs = millis();
        Serial.println("⚠️ [WI-FI]: Mất kết nối Wi-Fi, đang thử kết nối lại...");
        WiFi.reconnect();
      }
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
  initINA226();

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
  currentStation = STATION_A;
  currentLeg = LEG_AB;

  Serial.println("\n=========================================================================");
  Serial.println("   AGV_VER2 - TEST 09: ĐIỀU HƯỚNG 2 CHIỀU & BẢO MẬT WMS (MANG TẢI)");
  Serial.println("=========================================================================");
  Serial.printf("Tọa độ khởi đầu: X=%.2fm | Y=%.2fm | Yaw=%.1f° | Trạm hiện tại: Trạm A\n", odomX, odomY, yawDeg);
  Serial.printf("Cấu hình tốc độ tải: Straight=%d, Curve=%d, Brake=%d, Turn=%d, MinMove=%d\n",
                straightPwm, curvePwm, brakePwm, turnPwm, minMovePwm);
  Serial.printf("Xác thực bảo mật: Token='%s' | mDNS='%s.local'\n", agv_device_token, mdns_hostname);
  Serial.println(">> PHÍM TẮT ĐIỀU CHUYỂN:");
  Serial.println("   [Space / s] : START / STOP");
  Serial.println("   [n]         : Đến Trạm Kế Tiếp (Next Leg)");
  Serial.println("   [p]         : Lùi Về Trạm Trước Đó (Prev Leg - Ngược Chiều)");
  Serial.println("   [u]         : Quay Đầu 180° Tại Chỗ");
  Serial.println("   [g]         : Giả Lập Đứt Vạch 25cm");
  Serial.println("   [z]         : Đặt lại năng lượng tiêu thụ (Reset 0.0 mAh)");
  Serial.println("   [A, B, C, D]: Ra lệnh đến thẳng Trạm tương ứng (Point-to-Point)");
  Serial.println("-------------------------------------------------------------------------");

  xTaskCreatePinnedToCore(telemetryTask, "TelemetryTask", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlLoopTask, "Control100Hz", 8192, NULL, 2, NULL, 1);
}

// ===================================================================================
// LOOP CHÍNH (XỬ LÝ PHÍM SERIAL & BOOT)
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
      Serial.printf("\n[SERIAL KEY]: %s\n", isRunning ? "START" : "STOP");
    } 
    else if (ch == 'n' || ch == 'N') {
      advanceToNextLeg();
    }
    else if (ch == 'p' || ch == 'P') {
      advanceToPrevLeg();
    }
    else if (ch == 'u' || ch == 'U') {
      startManualTurn(180.0f);
    }
    else if (ch == 'g' || ch == 'G') {
      if (isRunning && robotSubState == SUB_LINE_FOLLOWING) {
        simulateGapActive = true;
        simGapStartLegDistM = legProgressM;
        Serial.printf("\n⚡ [TEST ĐỨT VẠCH]: Bật giả lập đứt 25cm tại %.2fm!\n", legProgressM);
      }
    }
    else if (ch == 'z' || ch == 'Z') {
      liveEnergyMah = 0.0f;
      liveEnergyWh = 0.0f;
      Serial.println("\n⚡ [SERIAL KEY]: Đã reset năng lượng tiêu thụ về 0.0 mAh!");
    }
    else if (ch == 'a' || ch == 'A') {
      planAndExecuteRoute(currentStation, STATION_A);
    }
    else if (ch == 'b' || ch == 'B') {
      planAndExecuteRoute(currentStation, STATION_B);
    }
    else if (ch == 'c' || ch == 'C') {
      planAndExecuteRoute(currentStation, STATION_C);
    }
    else if (ch == 'd' || ch == 'D') {
      planAndExecuteRoute(currentStation, STATION_D);
    }
  }

  // Nút BOOT
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
        advanceToNextLeg();
      }
    }
  }
  lastBtn = reading;

  vTaskDelay(pdMS_TO_TICKS(50));
}
