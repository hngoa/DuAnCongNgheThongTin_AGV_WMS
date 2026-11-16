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

// --- 4. THAM SỐ VƯỢT ĐỨT ĐOẠN & BỘ ĐIỀU KHIỂN VÒNG KÍN EKF + CASCADE (GIỮ TẢI NẶNG) ---
const float MAX_BRIDGE_DISTANCE_M    = 0.96f; // Đủ biên độ cho quét hình sin đến sát góc rẽ mà không cắt oan
const unsigned long MAX_BRIDGE_TIME_MS = 5500; // 5500ms an toàn khi xe mang tải chạy góc quét rộng
float gapStartDistM                  = 0.0f;
float gapStartAlongM                 = 0.0f;
float gapHeadingHoldDeg              = 0.0f;
float lastGapHeadingErr              = 0.0f;
float legStartX                      = 0.0f;
float legStartY                      = 0.0f;
float legLineOriginX                 = 0.0f;
float legLineOriginY                 = 0.0f;
bool  useGlobalLineProgress          = false;
float probeStageStartDistM           = 0.0f;
uint8_t probeStage                   = 0;
uint8_t turnSearchStage              = 0;
int8_t probeSweepSign                = 1;
int8_t reverseSearchPhase1Dir        = 1;
bool  inGapBridgeMode                = false;
bool  cornerApproachLatched          = false;
bool  hasSeenLineOnLeg               = false;
bool  lineTrackedOnLeg               = false;
float lastConfirmedLineProgM         = -1.0f;
float bridgeExitLineErr              = 0.0f;
bool  bridgeTurnAligning             = false;
unsigned long gapStartTimeMs         = 0;
unsigned long turnBrakeUntilMs       = 0;
unsigned long stationBrakeUntilMs    = 0;
unsigned long turnCatchGraceUntilMs  = 0;
int8_t lastTurnSpinDir               = 0;
bool  pendingStationSnap             = false;
float pendingSnapX                   = 0.0f;
float pendingSnapY                   = 0.0f;
float pendingSnapYaw                 = 90.0f;
unsigned long lastControlLoopUs      = 0;
int   consecutiveZeroLineFrames      = 0;
int   consecutiveGoodLineFrames      = 0;

// Trạng thái EKF 4 chiều [X, Y, yawRad, gyroBiasRadS]
float ekfX                           = 0.0f;
float ekfY                           = 0.0f;
float ekfYawRad                      = 1.5707963f;
float ekfGyroBiasRadS                = 0.0f;
float ekfP[4][4] = {
  { 0.0001f, 0.0f,    0.0f,      0.0f      },
  { 0.0f,    0.0001f, 0.0f,      0.0f      },
  { 0.0f,    0.0f,    0.00122f,  0.0f      },
  { 0.0f,    0.0f,    0.0f,      0.00030f  }
};
bool  wheelSlipDetected              = false;

// Vòng trong PID tốc độ 2 bánh + Feedforward tải nặng
float vLMeasFiltered                 = 0.0f;
float vRMeasFiltered                 = 0.0f;
float wheelIntL                      = 0.0f;
float wheelIntR                      = 0.0f;

bool  simulateGapActive              = false;
float simGapStartLegDistM            = 0.0f;
const float SIM_GAP_LENGTH_M         = 0.25f;

// --- 5. MÔ HÌNH ĐỒ THỊ ĐỘNG ĐA TRẠM (HỖ TRỢ route_legs TỪ SERVER + DỰ PHÒNG OFFLINE 4 TRẠM) ---
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

struct DynamicLeg {
  char  fromName[12];
  char  toName[12];
  float distM;
  float headingDeg;
  float targetX;
  float targetY;
  bool  hasTargetCoord;
};

StationCoord STATION_COORDS[4] = {
  { 0.00f, 0.00f }, // STATION_A (0.00m, 0.00m)
  { 0.00f, 1.51f }, // STATION_B (0.00m, 1.51m)
  { 1.54f, 1.51f }, // STATION_C (1.54m, 1.51m)
  { 1.54f, 0.00f }  // STATION_D (1.54m, 0.00m)
};

LegDef LEG_TABLE[8] = {
  { STATION_A, STATION_B, 1.51f,  90.0f, "A -> B" },
  { STATION_B, STATION_C, 1.54f,   0.0f, "B -> C" },
  { STATION_C, STATION_D, 1.51f, -90.0f, "C -> D" },
  { STATION_D, STATION_A, 1.54f, 180.0f, "D -> A" },
  { STATION_B, STATION_A, 1.51f, -90.0f, "B -> A" },
  { STATION_C, STATION_B, 1.54f, 180.0f, "C -> B" },
  { STATION_D, STATION_C, 1.51f,  90.0f, "D -> C" },
  { STATION_A, STATION_D, 1.54f,   0.0f, "A -> D" }
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

// Hàng đợi lộ trình đa chặng (Multi-leg Dynamic Queue - tối đa 16 chặng)
#define MAX_ROUTE_LEGS 16
DynamicLeg legQueue[MAX_ROUTE_LEGS];
int   queueLength = 0;
int   queueIndex  = 0;

volatile StationId     currentStation       = STATION_A;
volatile LegId         currentLeg           = LEG_AB;
char                   currentStationName[12] = "A";
char                   currentLegName[28]     = "A -> B";
char                   currentTargetName[12]  = "B";
float                  currentTargetX       = 0.00f;
float                  currentTargetY       = 1.51f;
bool                   currentTargetHasCoord = true;

volatile float         legTargetDistM      = 1.51f;
volatile float         legTargetHeadingDeg = 90.0f;
volatile float         startTurnYaw        = 0.0f;
volatile unsigned long turnSearchStartMs   = 0;
volatile bool          reverseSearchDoneForLeg = false;
volatile int           missionTurnDir      = 1;

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
volatile bool          requestGyroCalib    = false;

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
const char* volatile robotStateStr = "STOPPED";

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

static inline void ekfSyncFromPoseLocked(float x, float y, float yawD, bool clearBiasDelta, float posStdM = 0.015f, float yawStdDeg = 2.0f);
static inline void resetWheelCascadeLocked();

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

// Phanh ngắn mạch chủ động trên cầu H L298N (triệt tiêu quán tính dừng/xoay tải nặng)
void brakeMotorsHard() {
  cmdPwmL = 0;
  cmdPwmR = 0;
  digitalWrite(L298N_IN1_PIN, HIGH);
  digitalWrite(L298N_IN2_PIN, HIGH);
  writeLeftPwm(255);
  digitalWrite(L298N_IN3_PIN, HIGH);
  digitalWrite(L298N_IN4_PIN, HIGH);
  writeRightPwm(255);
}

StationId parseStationId(const char* name) {
  if (!name || name[0] == '\0') return STATION_UNKNOWN;
  if (strcmp(name, "A") == 0) return STATION_A;
  if (strcmp(name, "B") == 0) return STATION_B;
  if (strcmp(name, "C") == 0) return STATION_C;
  if (strcmp(name, "D") == 0) return STATION_D;
  return STATION_UNKNOWN;
}

DynamicLeg makeDynamicLegFromId(LegId leg) {
  DynamicLeg d;
  memset(&d, 0, sizeof(d));
  const char* stNames[4] = {"A", "B", "C", "D"};
  int fIdx = (int)LEG_TABLE[leg].fromStation;
  int tIdx = (int)LEG_TABLE[leg].toStation;
  snprintf(d.fromName, sizeof(d.fromName), "%s", (fIdx >= 0 && fIdx < 4) ? stNames[fIdx] : "A");
  snprintf(d.toName, sizeof(d.toName), "%s", (tIdx >= 0 && tIdx < 4) ? stNames[tIdx] : "B");
  d.distM = LEG_TABLE[leg].targetDistM;
  d.headingDeg = LEG_TABLE[leg].targetHeadingDeg;
  d.targetX = (tIdx >= 0 && tIdx < 4) ? STATION_COORDS[tIdx].x : 0.0f;
  d.targetY = (tIdx >= 0 && tIdx < 4) ? STATION_COORDS[tIdx].y : 0.0f;
  d.hasTargetCoord = (tIdx >= 0 && tIdx < 4);
  return d;
}

LegId findOfflineLegId(const char* fromName, const char* toName) {
  StationId fSt = parseStationId(fromName);
  StationId tSt = parseStationId(toName);
  for (int i = 0; i < 8; i++) {
    if (LEG_TABLE[i].fromStation == fSt && LEG_TABLE[i].toStation == tSt) {
      return (LegId)i;
    }
  }
  return LEG_NONE;
}

// Khởi chạy DynamicLeg bên trong stateMux (không gọi Serial hoặc mutex lồng nhau)
void applyDynamicLegLocked(const DynamicLeg &leg, float &outHeadingErr, bool &outTurning) {
  currentLeg = findOfflineLegId(leg.fromName, leg.toName);
  snprintf(currentStationName, sizeof(currentStationName), "%s", leg.fromName);
  currentStation = parseStationId(currentStationName);
  snprintf(currentTargetName, sizeof(currentTargetName), "%s", leg.toName);
  currentTargetX = leg.targetX;
  currentTargetY = leg.targetY;
  currentTargetHasCoord = leg.hasTargetCoord;
  snprintf(currentLegName, sizeof(currentLegName), "%s -> %s", leg.fromName, leg.toName);

  legTargetDistM = leg.distM;
  legTargetHeadingDeg = leg.headingDeg;
  legStartX = odomX;
  legStartY = odomY;

  float legRad = (legTargetHeadingDeg * PI) / 180.0f;
  float ux = cosf(legRad);
  float uy = sinf(legRad);
  float nx = -uy;
  float ny =  ux;
  if (currentTargetHasCoord) {
    legLineOriginX = currentTargetX - legTargetDistM * ux;
    legLineOriginY = currentTargetY - legTargetDistM * uy;
  } else {
    legLineOriginX = legStartX;
    legLineOriginY = legStartY;
  }

  float alongStart = (odomX - legLineOriginX) * ux + (odomY - legLineOriginY) * uy;
  float crossStart = currentTargetHasCoord
                       ? ((odomX - currentTargetX) * nx + (odomY - currentTargetY) * ny)
                       : 0.0f;
  useGlobalLineProgress = (currentTargetHasCoord && fabsf(alongStart) <= 0.30f && fabsf(crossStart) <= 0.35f);

  prevTicksL = globalTicksL;
  prevTicksR = globalTicksR;
  legProgressM = useGlobalLineProgress ? alongStart : 0.0f;

  outHeadingErr = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
  lastLineError     = 0.0f;
  lastValidError    = 0.0f;
  lineLostTimeMs    = 0;
  inGapBridgeMode   = false;
  gapStartAlongM    = 0.0f;
  cornerApproachLatched = false;
  hasSeenLineOnLeg  = false;
  lineTrackedOnLeg  = false;
  lastConfirmedLineProgM = -1.0f;
  bridgeExitLineErr = 0.0f;
  bridgeTurnAligning = false;
  reverseSearchDoneForLeg = false;
  turnSearchStage   = 0;
  probeStage        = 0;
  probeSweepSign    = (crossStart > 0.015f) ? -1 : 1;
  probeStageStartDistM = 0.0f;
  simulateGapActive = false;
  turnBrakeUntilMs  = 0;
  stationBrakeUntilMs = 0;
  turnCatchGraceUntilMs = 0;
  lastTurnSpinDir   = 0;
  pendingStationSnap = false;
  consecutiveZeroLineFrames = 0;
  consecutiveGoodLineFrames = 0;
  wheelSlipDetected = false;
  resetWheelCascadeLocked();

  if (fabsf(outHeadingErr) > 6.0f) {
    smoothedBaseSpeed = 0.0f;
    startTurnYaw = yawDeg;
    missionTurnDir = (outHeadingErr >= 0.0f) ? 1 : -1;
    reverseSearchPhase1Dir = (currentTargetHasCoord && fabsf(crossStart) > 0.015f)
                               ? ((crossStart < 0.0f) ? 1 : -1)
                               : -missionTurnDir;
    robotSubState = SUB_TURNING_FOR_MISSION;
    robotStateStr = "TURNING_STATION";
    isRunning = true;
    outTurning = true;
  } else {
    smoothedBaseSpeed = (float)minMovePwm;
    robotSubState = SUB_LINE_FOLLOWING;
    robotStateStr = "RUNNING_FAST";
    isRunning = true;
    outTurning = false;
  }
}

void startDynamicLeg(const DynamicLeg &leg) {
  float hErr = 0.0f;
  bool turning = false;
  float snapYaw = 0.0f;

  portENTER_CRITICAL(&stateMux);
  applyDynamicLegLocked(leg, hErr, turning);
  snapYaw = yawDeg;
  portEXIT_CRITICAL(&stateMux);

  if (turning) {
    Serial.printf("\n>> [BẮT ĐẦU CHẶNG: %s -> %s] Yaw = %.1f° | Cần xoay %+.1f° tới %.1f°...\n",
                  leg.fromName, leg.toName, snapYaw, hErr, leg.headingDeg);
  } else {
    Serial.printf("\n>> [BẮT ĐẦU CHẶNG: %s -> %s] Đúng hướng %.1f° -> Khởi hành bám line!\n",
                  leg.fromName, leg.toName, leg.headingDeg);
  }
}

void startMission(LegId leg) {
  if (leg >= LEG_NONE) leg = LEG_AB;
  DynamicLeg d = makeDynamicLegFromId(leg);
  startDynamicLeg(d);
}

// Lập kế hoạch lộ trình Điểm - Điểm tự động (dự phòng khi offline hoặc không có route_legs)
void planAndExecuteRoute(StationId fromSt, StationId toSt) {
  if (toSt >= STATION_UNKNOWN) {
    Serial.println("\n>> [CẢNH BÁO] Trạm đích không nằm trong bảng 4 trạm dự phòng offline!");
    return;
  }
  if (fromSt == toSt) {
    stopMotors();
    portENTER_CRITICAL(&stateMux);
    queueLength = 0;
    queueIndex = 0;
    currentStation = toSt;
    snprintf(currentStationName, sizeof(currentStationName), "%c", "ABCD"[toSt]);
    ekfSyncFromPoseLocked(STATION_COORDS[toSt].x, STATION_COORDS[toSt].y, yawDeg, false, 0.01f, 2.5f);
    resetWheelCascadeLocked();
    robotSubState = SUB_STOPPED;
    robotStateStr = "AT_STATION";
    isRunning = false;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("\n>> [THÔNG BÁO] Xe đang ở sẵn trạm đích: %c!\n", "ABCD"[toSt]);
    return;
  }

  DynamicLeg firstLeg;
  int len = 0;
  bool turning = false;
  float hErr = 0.0f;
  float snapYaw = 0.0f;

  portENTER_CRITICAL(&stateMux);
  queueLength = 0;
  queueIndex = 0;

  for (int i = 0; i < 8; i++) {
    if (LEG_TABLE[i].fromStation == fromSt && LEG_TABLE[i].toStation == toSt) {
      legQueue[0] = makeDynamicLegFromId((LegId)i);
      queueLength = 1;
      break;
    }
  }

  if (queueLength == 0) {
    if (fromSt == STATION_A && toSt == STATION_C) {
      legQueue[0] = makeDynamicLegFromId(LEG_AB);
      legQueue[1] = makeDynamicLegFromId(LEG_BC);
      queueLength = 2;
    } else if (fromSt == STATION_C && toSt == STATION_A) {
      legQueue[0] = makeDynamicLegFromId(LEG_CD);
      legQueue[1] = makeDynamicLegFromId(LEG_DA);
      queueLength = 2;
    } else if (fromSt == STATION_B && toSt == STATION_D) {
      legQueue[0] = makeDynamicLegFromId(LEG_BC);
      legQueue[1] = makeDynamicLegFromId(LEG_CD);
      queueLength = 2;
    } else if (fromSt == STATION_D && toSt == STATION_B) {
      legQueue[0] = makeDynamicLegFromId(LEG_DA);
      legQueue[1] = makeDynamicLegFromId(LEG_AB);
      queueLength = 2;
    }
  }

  len = queueLength;
  if (queueLength > 0) {
    firstLeg = legQueue[0];
    applyDynamicLegLocked(firstLeg, hErr, turning);
    snapYaw = yawDeg;
  }
  portEXIT_CRITICAL(&stateMux);

  if (len > 0) {
    Serial.printf("\n🚀 [ĐIỀU PHỐI OFFLINE] Lộ trình %d chặng: %c -> %c (Khởi hành %s -> %s, Yaw=%.1f°)\n",
                  len, "ABCD"[fromSt], "ABCD"[toSt], firstLeg.fromName, firstLeg.toName, snapYaw);
  }
}

void executeServerRouteLegs(DynamicLeg legs[], int count) {
  if (count <= 0) return;
  if (count > MAX_ROUTE_LEGS) count = MAX_ROUTE_LEGS;
  bool turning = false;
  float hErr = 0.0f;

  portENTER_CRITICAL(&stateMux);
  autoLoopEnabled = false;
  for (int i = 0; i < count; i++) {
    legQueue[i] = legs[i];
  }
  queueLength = count;
  queueIndex = 0;
  applyDynamicLegLocked(legQueue[0], hErr, turning);
  portEXIT_CRITICAL(&stateMux);

  Serial.printf("\n🛰️ [ĐIỀU PHỐI DIJKSTRA SERVER] Nạp %d chặng, khởi hành: %s -> %s (dist=%.2fm, head=%.1f°)\n",
                count, legs[0].fromName, legs[0].toName, legs[0].distM, legs[0].headingDeg);
}

LegId nextOfflineLegLocked(bool reverseDir) {
  if (reverseDir) {
    if (currentStation == STATION_B) return LEG_BA;
    if (currentStation == STATION_C) return LEG_CB;
    if (currentStation == STATION_D) return LEG_DC;
    return LEG_AD;
  }
  if (currentStation == STATION_A) return LEG_AB;
  if (currentStation == STATION_B) return LEG_BC;
  if (currentStation == STATION_C) return LEG_CD;
  return LEG_DA;
}

// Chuyển sang chặng kế tiếp (tôn trọng autoLoopReverse & legQueue)
void advanceToNextLeg() {
  DynamicLeg nextDyn;
  bool turning = false;
  float hErr = 0.0f;

  portENTER_CRITICAL(&stateMux);
  if (queueLength > 0 && queueIndex + 1 < queueLength) {
    queueIndex++;
    nextDyn = legQueue[queueIndex];
  } else {
    queueLength = 0;
    queueIndex = 0;
    nextDyn = makeDynamicLegFromId(nextOfflineLegLocked(autoLoopReverse));
  }
  applyDynamicLegLocked(nextDyn, hErr, turning);
  portEXIT_CRITICAL(&stateMux);

  Serial.printf("\n>> [CHẶNG KẾ TIẾP]: %s -> %s (head=%.1f°)\n",
                nextDyn.fromName, nextDyn.toName, nextDyn.headingDeg);
}

// Chuyển sang chặng lùi về (Ngược chiều)
void advanceToPrevLeg() {
  DynamicLeg prevDyn;
  bool turning = false;
  float hErr = 0.0f;

  portENTER_CRITICAL(&stateMux);
  queueLength = 0;
  queueIndex = 0;
  prevDyn = makeDynamicLegFromId(nextOfflineLegLocked(true));
  applyDynamicLegLocked(prevDyn, hErr, turning);
  portEXIT_CRITICAL(&stateMux);

  Serial.printf("\n>> [CHẶNG NGƯỢC CHIỀU]: %s -> %s (head=%.1f°)\n",
                prevDyn.fromName, prevDyn.toName, prevDyn.headingDeg);
}

// Khởi chạy xoay thủ công góc deltaDeg
void startManualTurn(float deltaDeg) {
  float targetH = 0.0f;
  portENTER_CRITICAL(&stateMux);
  legTargetHeadingDeg = normalizeAngleDeg(yawDeg + deltaDeg);
  targetH = legTargetHeadingDeg;
  startTurnYaw = yawDeg;
  robotSubState = SUB_TURNING_MANUAL;
  robotStateStr = "TURNING_MANUAL";
  isRunning = true;
  portEXIT_CRITICAL(&stateMux);
  Serial.printf("\n>> [XOAY THỦ CÔNG]: Cần xoay %+.1f° -> Hướng mới: %.1f°\n", deltaDeg, targetH);
}

// ===================================================================================
// CẢM BIẾN DÒ LINE TCRT5000 (5 MẮT - TÁCH USB-JTAG GPIO 19/20 + LỌC CHÓI)
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

  int mask = (sL2 << 4) | (sL1 << 3) | (sC << 2) | (sR1 << 1) | sR2;

  // Lọc nhiễu quang học khi chỉ 2 mắt rìa kích cùng lúc (10001)
  if (mask == 0b10001) {
    sL2 = sL1 = sC = sR1 = sR2 = 0;
    return 0;
  }

  return mask;
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
// TẦNG 1 & 2: EKF 4 TRẠNG THÁI + CASCADE VÒNG KÍN TỐC ĐỘ BÁNH & TỐC ĐỘ GÓC (GIỮ TẢI)
// ===================================================================================
constexpr float DEG2RAD_F = PI / 180.0f;
constexpr float RAD2DEG_F = 180.0f / PI;

static inline float wrapRad(float r) {
  while (r >  PI) r -= 2.0f * PI;
  while (r < -PI) r += 2.0f * PI;
  return r;
}

static inline void ekfSymmetrizeAndFloor() {
  for (int i = 0; i < 4; i++) {
    for (int j = i; j < 4; j++) {
      float avg = 0.5f * (ekfP[i][j] + ekfP[j][i]);
      ekfP[i][j] = avg;
      ekfP[j][i] = avg;
    }
    if (ekfP[i][i] < 1.0e-8f) ekfP[i][i] = 1.0e-8f;
  }
}

static inline void resetWheelCascadeLocked() {
  vLMeasFiltered = 0.0f;
  vRMeasFiltered = 0.0f;
  wheelIntL = 0.0f;
  wheelIntR = 0.0f;
}

static inline void ekfSyncFromPoseLocked(float x, float y, float yawD, bool clearBiasDelta, float posStdM, float yawStdDeg) {
  ekfX = x;
  ekfY = y;
  ekfYawRad = wrapRad(normalizeAngleDeg(yawD) * DEG2RAD_F);
  if (clearBiasDelta) ekfGyroBiasRadS = 0.0f;

  float pPos = posStdM * posStdM;
  float yStdRad = yawStdDeg * DEG2RAD_F;
  float pYaw = yStdRad * yStdRad;
  float bStdRad = 1.2f * DEG2RAD_F;
  float pBg = bStdRad * bStdRad;

  memset(ekfP, 0, sizeof(ekfP));
  ekfP[0][0] = pPos;
  ekfP[1][1] = pPos;
  ekfP[2][2] = pYaw;
  ekfP[3][3] = pBg;

  odomX = ekfX;
  odomY = ekfY;
  yawDeg = normalizeAngleDeg(ekfYawRad * RAD2DEG_F);
}

static inline void ekfPredictLocked(float deltaS, float deltaSL, float deltaSR,
                                    float gzRawDegS, bool isTurningInPlace, float dt) {
  float omegaRadS = 0.0f;
  if (mpuConnected) {
    omegaRadS = ((gzRawDegS - gyroZOffset) * DEG2RAD_F) - ekfGyroBiasRadS;
  } else if (dt > 0.0005f) {
    omegaRadS = (deltaSR - deltaSL) / (WHEEL_BASE_W_M * dt);
  }

  float dTheta = omegaRadS * dt;
  float yawMid = ekfYawRad + 0.5f * dTheta;
  float dsEff = isTurningInPlace ? 0.0f : deltaS;

  ekfX += dsEff * cosf(yawMid);
  ekfY += dsEff * sinf(yawMid);
  ekfYawRad = wrapRad(ekfYawRad + dTheta);

  // Phát hiện trượt bánh thực tế dựa trên chênh lệch Encoder so với Gyro khi chạy thẳng
  wheelSlipDetected = false;
  if (!isTurningInPlace && mpuConnected && deltaSL > 0.0004f && deltaSR > 0.0004f && dt > 0.001f) {
    float dPsiEnc = (deltaSR - deltaSL) / WHEEL_BASE_W_M;
    float slipRate = fabsf(dPsiEnc - dTheta) / dt;
    if (slipRate > 0.25f) {
      wheelSlipDetected = true;
    }
  }

  float f02 = -dsEff * sinf(yawMid);
  float f12 =  dsEff * cosf(yawMid);
  float f23 = -dt;

  float F[4][4] = {
    { 1.0f, 0.0f, f02,  0.0f },
    { 0.0f, 1.0f, f12,  0.0f },
    { 0.0f, 0.0f, 1.0f, f23  },
    { 0.0f, 0.0f, 0.0f, 1.0f }
  };

  float FP[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      FP[i][j] = F[i][0]*ekfP[0][j] + F[i][1]*ekfP[1][j] + F[i][2]*ekfP[2][j] + F[i][3]*ekfP[3][j];
    }
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ekfP[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1] + FP[i][2]*F[j][2] + FP[i][3]*F[j][3];
    }
  }

  float slipMult = wheelSlipDetected ? 3.0f : 1.0f;
  float sigS = (0.015f * fabsf(dsEff) + 0.0002f) * slipMult;
  float sigLat = (0.045f * fabsf(dsEff) + 0.0001f) * slipMult;
  float s2 = sigS * sigS;
  float l2 = sigLat * sigLat;
  float c = cosf(yawMid);
  float s = sinf(yawMid);
  ekfP[0][0] += s2 * c * c + l2 * s * s;
  ekfP[1][1] += s2 * s * s + l2 * c * c;
  float qxy = (s2 - l2) * s * c;
  ekfP[0][1] += qxy;
  ekfP[1][0] += qxy;

  float sigYaw = 0.018f * fabsf(dTheta) + (0.010f * dt * slipMult);
  ekfP[2][2] += sigYaw * sigYaw;
  float sigBg = 0.0010f * dt;
  ekfP[3][3] += sigBg * sigBg;

  ekfSymmetrizeAndFloor();
  odomX = ekfX;
  odomY = ekfY;
  yawDeg = normalizeAngleDeg(ekfYawRad * RAD2DEG_F);
}

// Cập nhật đo quang học 5-IR vào EKF: chỉ kéo đúng sai lệch ngang + covariance, KHÔNG ép cứng yaw=90°
static inline void ekfUpdateOpticalLineLocked(float lineErrVal, int activeCnt, float legHeadDeg,
                                              float refX, float refY, bool recoveringBridge) {
  if (!currentTargetHasCoord || activeCnt < 1 || activeCnt > 3 || fabsf(lineErrVal) > 2.05f) return;

  float legRad = legHeadDeg * DEG2RAD_F;
  float nx = -sinf(legRad);
  float ny =  cosf(legRad);
  float relPsi = wrapRad(ekfYawRad - legRad);

  float zCross = (lineErrVal * 0.018f) * cosf(relPsi);
  float sx = ekfX + DIST_AXLE_TO_SENSOR_M * cosf(ekfYawRad);
  float sy = ekfY + DIST_AXLE_TO_SENSOR_M * sinf(ekfYawRad);
  float hCross = (sx - refX) * nx + (sy - refY) * ny;
  float yInnov = zCross - hCross;

  float H[4] = { nx, ny, DIST_AXLE_TO_SENSOR_M * cosf(relPsi), 0.0f };

  float sig = 0.012f;
  if (activeCnt == 2) sig = 0.008f;
  else if (fabsf(lineErrVal) > 1.5f) sig = 0.014f;
  if (recoveringBridge && fabsf(yInnov) > 0.02f) sig = 0.009f;
  if (fabsf(yInnov) > 0.12f) sig *= 2.0f;
  float R = sig * sig;

  float PHt[4];
  for (int i = 0; i < 4; i++) {
    PHt[i] = ekfP[i][0]*H[0] + ekfP[i][1]*H[1] + ekfP[i][2]*H[2];
  }
  float S = H[0]*PHt[0] + H[1]*PHt[1] + H[2]*PHt[2] + R;
  if (S < 1.0e-8f) return;

  float K[4] = { PHt[0]/S, PHt[1]/S, PHt[2]/S, PHt[3]/S };
  // Không cho cú giật phục hồi sau bridge đổ lỗi cho bias con quay
  float bgGain = (recoveringBridge || fabsf(yInnov) > 0.025f) ? 0.0f : K[3];

  ekfX += K[0] * yInnov;
  ekfY += K[1] * yInnov;
  ekfYawRad = wrapRad(ekfYawRad + K[2] * yInnov);
  float maxBgRad = 1.8f * DEG2RAD_F;
  ekfGyroBiasRadS = constrain(ekfGyroBiasRadS + bgGain * yInnov, -maxBgRad, maxBgRad);

  float K_eff[4] = { K[0], K[1], K[2], bgGain };
  float newP[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      newP[i][j] = ekfP[i][j]
                   - K_eff[i]*(H[0]*ekfP[0][j] + H[1]*ekfP[1][j] + H[2]*ekfP[2][j]);
    }
  }
  memcpy(ekfP, newP, sizeof(ekfP));
  ekfSymmetrizeAndFloor();

  odomX = ekfX;
  odomY = ekfY;
  yawDeg = normalizeAngleDeg(ekfYawRad * RAD2DEG_F);
}

// Ánh xạ PWM chịu tải -> Tốc độ tuyến tính tham chiếu (m/s)
static inline float pwmToTargetSpeedMS(float pwmCmd) {
  float clamped = constrain(pwmCmd, (float)minMovePwm, (float)maxPwm);
  float spanPwm = (float)max(straightPwm - minMovePwm, 10);
  return 0.24f + (clamped - (float)minMovePwm) * (0.26f / spanPwm);
}

// Bộ điều khiển Cascade 2 vòng: Tốc độ góc Gyro (outer) + Vận tốc 2 bánh PI & Feedforward chịu tải (inner)
// Bất di bất dịch: KHÔNG bao giờ giảm lực đẩy dưới sàn minMovePwm khi mang tải.
static inline void computeLoadedCascadePwm(float basePwm, float omegaRefRadS, float omegaMeasDegS,
                                           float dsL, float dsR, float dt,
                                           int &outLeft, int &outRight) {
  float safeDt = fmaxf(dt, 0.002f);
  vLMeasFiltered = 0.65f * vLMeasFiltered + 0.35f * (dsL / safeDt);
  vRMeasFiltered = 0.65f * vRMeasFiltered + 0.35f * (dsR / safeDt);

  float clampedBase = constrain(basePwm, (float)minMovePwm, (float)maxPwm);
  float vBaseRef = pwmToTargetSpeedMS(clampedBase);
  float halfDiffV = 0.5f * omegaRefRadS * WHEEL_BASE_W_M;
  float vLRef = fmaxf(0.14f, vBaseRef - halfDiffV);
  float vRRef = fmaxf(0.14f, vBaseRef + halfDiffV);

  float slope = (float)max(straightPwm - minMovePwm, 10) / 0.26f;
  float ffL = clampedBase - halfDiffV * slope;
  float ffR = clampedBase + halfDiffV * slope;

  float errVL = vLRef - vLMeasFiltered;
  float errVR = vRRef - vRMeasFiltered;

  // Chặn tích phân bất đối xứng: xả giảm tối đa -10 PWM, nhưng cho phép bù tăng +28 PWM khi tải nặng/sụt áp
  wheelIntL = constrain(wheelIntL + 42.0f * errVL * safeDt, -10.0f, 28.0f);
  wheelIntR = constrain(wheelIntR + 42.0f * errVR * safeDt, -10.0f, 28.0f);

  float piL = constrain(55.0f * errVL + wheelIntL, -14.0f, 34.0f);
  float piR = constrain(55.0f * errVR + wheelIntR, -14.0f, 34.0f);

  float omegaMeasRadS = omegaMeasDegS * DEG2RAD_F;
  float yawRateTrim = mpuConnected ? constrain(16.0f * (omegaRefRadS - omegaMeasRadS), -24.0f, 24.0f) : 0.0f;

  int steerMin = max(148, minMovePwm - 38);
  outLeft  = constrain((int)lroundf(ffL + piL - yawRateTrim), steerMin, maxPwm);
  outRight = constrain((int)lroundf(ffR + piR + yawRateTrim), steerMin, maxPwm);

  if (max(outLeft, outRight) < minMovePwm) {
    int boost = minMovePwm - max(outLeft, outRight);
    outLeft  = constrain(outLeft + boost, steerMin, maxPwm);
    outRight = constrain(outRight + boost, steerMin, maxPwm);
  }
}

// Tính PWM tiến chậm giữ hướng thẳng khi tiếp cận góc trạm đích (giữ >= minMovePwm qua Cascade)
static inline void computeCornerApproachPwm(float headingDeg, float curYawDeg,
                                            int baseBrakePwm, float omegaMeasDegS,
                                            float dsL, float dsR, float dt,
                                            int& outLeft, int& outRight) {
  float hErrDeg = normalizeAngleDeg(headingDeg - curYawDeg);
  float omegaRefRadS = constrain(2.4f * (hErrDeg * DEG2RAD_F), -0.95f, 0.95f);
  computeLoadedCascadePwm((float)max(baseBrakePwm, minMovePwm), omegaRefRadS, omegaMeasDegS, dsL, dsR, dt, outLeft, outRight);
}

// Điều tiết vận tốc góc xoay bằng xung mô-men đủ tải (>= minMovePwm) + phanh chủ động (-1)
static inline int computeRegulatedTurnPwm(float absErrDeg, bool slowMode, float curOmegaAbsDegS) {
  float targetOmegaDegS = constrain(2.6f * absErrDeg, 58.0f, 120.0f);
  int drivePwm = (absErrDeg > 42.0f) ? turnPwm : max(minMovePwm, turnPwm - 6);
  if (slowMode) {
    targetOmegaDegS = fminf(targetOmegaDegS, 54.0f);
    drivePwm = minMovePwm;
  }
  if (mpuConnected && curOmegaAbsDegS > targetOmegaDegS) {
    return -1; // Vượt tốc độ góc mục tiêu -> kích brakeMotorsHard() hãm ngắn tránh vọt lố 20-24 độ
  }
  return constrain(drivePwm, minMovePwm, maxPwm);
}

// Chiếu tọa độ 2D trên hệ quy chiếu chặng (+ = Trái, - = Phải)
static inline float computeLegCrossTrackM() {
  float refX = currentTargetHasCoord ? currentTargetX : legStartX;
  float refY = currentTargetHasCoord ? currentTargetY : legStartY;
  float legRad = legTargetHeadingDeg * DEG2RAD_F;
  return (odomX - refX) * (-sinf(legRad)) + (odomY - refY) * cosf(legRad);
}

static inline float computeLegSensorCrossTrackM() {
  float refX = currentTargetHasCoord ? currentTargetX : legStartX;
  float refY = currentTargetHasCoord ? currentTargetY : legStartY;
  float legRad = legTargetHeadingDeg * DEG2RAD_F;
  float yawRad = yawDeg * DEG2RAD_F;
  float sx = odomX + DIST_AXLE_TO_SENSOR_M * cosf(yawRad);
  float sy = odomY + DIST_AXLE_TO_SENSOR_M * sinf(yawRad);
  return (sx - refX) * (-sinf(legRad)) + (sy - refY) * cosf(legRad);
}

static inline float computeLegAlongM() {
  float legRad = legTargetHeadingDeg * DEG2RAD_F;
  return (odomX - legLineOriginX) * cosf(legRad) + (odomY - legLineOriginY) * sinf(legRad);
}

// Đánh giá hành lang hợp lệ để vào góc / snap tọa độ trạm
static inline bool isCorridorValidNearCornerLocked(float lastProgM, float crossM, float yawErrDeg, bool forSnap) {
  if (!lineTrackedOnLeg || lastProgM < 0.0f) return false;
  float maxCross = forSnap ? 0.16f : 0.18f;
  float maxYaw   = forSnap ? 18.0f : 20.0f;
  if (currentTargetHasCoord && fabsf(crossM) > maxCross) return false;
  if (yawErrDeg > maxYaw) return false;

  float gapToTarget = legTargetDistM - lastProgM;
  if (gapToTarget <= (forSnap ? 0.48f : 0.52f)) {
    return true;
  }
  bool straightCorridor = (!currentTargetHasCoord || fabsf(crossM) <= 0.14f) &&
                          (yawErrDeg <= 18.0f) &&
                          (fabsf(bridgeExitLineErr) < 1.15f);
  float maxBridgeToEnd = MAX_BRIDGE_DISTANCE_M + DIST_AXLE_TO_SENSOR_M + 0.14f;
  return straightCorridor && (gapToTarget <= maxBridgeToEnd);
}

// Dừng an toàn kèm phanh ngắn chủ động 90ms khi vượt giới hạn bridge/mất vạch
static inline void enterGapTimeoutStopLocked(unsigned long nowMs) {
  smoothedBaseSpeed = 0.0f;
  inGapBridgeMode = false;
  pendingStationSnap = false;
  resetWheelCascadeLocked();
  robotSubState = SUB_STOPPED;
  robotStateStr = "GAP_TIMEOUT_STOP";
  isRunning = false;
  stationBrakeUntilMs = nowMs + 90;
}

// Bộ điều khiển Stanley + Quét hình sin thích nghi khi vượt đứt đoạn ở tốc độ chịu tải:
//   - Pha 0: Stanley hội tụ trục giữa tại bridgePwm (222)
//   - Pha 1: Nếu qua đoạn đứt danh định (~0.36m) mà chưa thấy line, tự kích hoạt quét hình sin mở rộng ±7.8cm
//            (tầm phủ thanh 5-IR ~±11.4cm) ở PWM 216 (>= curvePwm), kết thúc êm trước góc trạm.
static inline void computeBridgeCascadeTargetLocked(float curCrossSensM, float gapAlongM, float remainingLegM,
                                                    bool isSideSkidExit, float &outDesiredHeadingDeg,
                                                    int &outTargetBasePwm, bool &outStageChanged) {
  uint8_t prevStage = probeStage;
  float sweepStartGapM = isSideSkidExit ? 0.14f : (hasSeenLineOnLeg ? 0.36f : 0.24f);

  float dTarget = 0.0f;
  float tangentDeg = 0.0f;

  if (gapAlongM < sweepStartGapM || remainingLegM <= 0.19f) {
    probeStage = 0;
    outTargetBasePwm = (remainingLegM < 0.36f) ? curvePwm : bridgePwm;
  } else {
    probeStage = 1;
    float sw = fmaxf(0.0f, gapAlongM - sweepStartGapM);
    float taper = constrain((remainingLegM - 0.19f) / 0.06f, 0.0f, 1.0f);
    float ampM = taper * constrain(0.068f + 0.024f * sw, 0.068f, 0.078f);
    float phase = (2.0f * PI * sw) / 0.46f;
    dTarget = (float)probeSweepSign * ampM * sinf(phase);
    float dSlope = (float)probeSweepSign * ampM * ((2.0f * PI) / 0.46f) * cosf(phase);
    tangentDeg = constrain(atanf(dSlope) * RAD2DEG_F, -30.0f, 30.0f);
    outTargetBasePwm = constrain(bridgePwm - 6, curvePwm, bridgePwm);
  }

  float vEff = fmaxf(0.28f, fabsf(currentSpeed) * 0.01f);
  float stanleyDeg = atan2f(3.0f * (dTarget - curCrossSensM), vEff + 0.15f) * RAD2DEG_F;
  float relSteerDeg = constrain(tangentDeg + stanleyDeg, -34.0f, 34.0f);
  outDesiredHeadingDeg = normalizeAngleDeg(legTargetHeadingDeg + relSteerDeg);
  outStageChanged = (probeStage != prevStage);
}

// Khởi tạo trạng thái chạy bám line/bridge ngay sau khi kết thúc xoay tại trạm
static inline void enterLineFollowingFromTurnLocked(bool lineSeen, float initialErr = 0.0f) {
  smoothedBaseSpeed = (float)minMovePwm;
  prevTicksL = globalTicksL;
  prevTicksR = globalTicksR;
  legProgressM = useGlobalLineProgress ? computeLegAlongM() : 0.0f;
  lastLineError = lineSeen ? initialErr : 0.0f;
  lastValidError = lineSeen ? initialErr : 0.0f;
  inGapBridgeMode = false;
  gapStartAlongM = 0.0f;
  cornerApproachLatched = false;
  hasSeenLineOnLeg = lineSeen;
  lineTrackedOnLeg = false;
  lastConfirmedLineProgM = -1.0f;
  bridgeExitLineErr = 0.0f;
  bridgeTurnAligning = false;
  probeStage = 0;
  stationBrakeUntilMs = 0;
  pendingStationSnap = false;
  unsigned long nowMsSnap = millis();
  turnCatchGraceUntilMs = lineSeen ? (nowMsSnap + 420UL) : 0UL;
  consecutiveZeroLineFrames = 0;
  consecutiveGoodLineFrames = 0;
  wheelSlipDetected = false;
  resetWheelCascadeLocked();
  robotSubState = SUB_LINE_FOLLOWING;
  robotStateStr = "RUNNING_FAST";
  isRunning = true;
}

static inline void resetStopStateLocked(const char* stateLabel) {
  autoLoopEnabled = false;
  queueLength = 0;
  queueIndex = 0;
  turnBrakeUntilMs = 0;
  stationBrakeUntilMs = 0;
  turnCatchGraceUntilMs = 0;
  pendingStationSnap = false;
  inGapBridgeMode = false;
  gapStartAlongM = 0.0f;
  bridgeTurnAligning = false;
  cornerApproachLatched = false;
  smoothedBaseSpeed = 0.0f;
  wheelSlipDetected = false;
  resetWheelCascadeLocked();
  robotSubState = SUB_STOPPED;
  robotStateStr = stateLabel;
  isRunning = false;
}

// ===================================================================================
// CORE 1: ĐIỀU KHIỂN THỜI GIAN THỰC 100Hz (AN TOÀN ĐA NHÂN & KHÔNG CHẠM I2C TRONG MUTEX)
// ===================================================================================
void updateControlAndOdometry(float dt = 0.010f) {
  // Thực thi CALIBRATE_GYRO trên Core 1 để tránh đụng độ I2C0 với Core 0
  if (requestGyroCalib) {
    requestGyroCalib = false;
    stopMotors();
    portENTER_CRITICAL(&stateMux);
    resetStopStateLocked("CALIBRATING_GYRO");
    portEXIT_CRITICAL(&stateMux);

    calibrateGyroZ(400);

    portENTER_CRITICAL(&stateMux);
    ekfSyncFromPoseLocked(odomX, odomY, 90.0f, true, 0.015f, 1.5f);
    resetWheelCascadeLocked();
    robotStateStr = "STOPPED";
    portEXIT_CRITICAL(&stateMux);
    return;
  }

  // Đọc phần cứng I2C / Encoder / Line TRƯỚC khi vào stateMux
  float gzRaw = 0.0f;
  if (mpuConnected) {
    int16_t rGz = readReg16(MPU6050_ADDR, 0x47);
    gzRaw = (rGz / 32.8f);
  }

  long curL = 0, curR = 0;
  getCalibratedTicks(curL, curR);

  static int powerDiv = 0;
  if (++powerDiv >= 5) {
    powerDiv = 0;
    updatePowerSensors();
  }

  int sL2, sL1, sC, sR1, sR2;
  int maskSnap = readLineSensors(sL2, sL1, sC, sR1, sR2);
  int activeCount = sL2 + sL1 + sC + sR1 + sR2;

  unsigned long nowMs = millis();
  unsigned long nowUs = micros();
  float realDt = (lastControlLoopUs == 0) ? dt : ((float)(nowUs - lastControlLoopUs) / 1000000.0f);
  lastControlLoopUs = nowUs;
  if (realDt < 0.002f || realDt > 0.050f) realDt = dt;

  int outLeftPwm = 0;
  int outRightPwm = 0;

  // Flags log ngoài stateMux (tránh chặn ngắt khi in Serial)
  bool logCaught = false, logArrived = false, logGapStart = false, logGapTimeout = false, logGapRecovered = false;
  char logLegBuf[28] = {0};
  char logStBuf[12]  = {0};
  float logYaw = 0.0f, logX = 0.0f, logY = 0.0f, logGapVal = 0.0f;

  portENTER_CRITICAL(&stateMux);

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
                           robotSubState == SUB_TURNING_MANUAL ||
                           (robotSubState == SUB_LINE_FOLLOWING && inGapBridgeMode && bridgeTurnAligning) ||
                           (turnBrakeUntilMs > nowMs));
  if (isTurningInPlace) {
    deltaS = 0.0f;
  }

  float liveGzDegPerSec = 0.0f;
  float rawDelta = mpuConnected ? (gzRaw - gyroZOffset) : 0.0f;
  bool stationaryZupt = mpuConnected && !isRunning && (testMotorUntilMs < nowMs) &&
                        (stationBrakeUntilMs <= nowMs) &&
                        (robotSubState == SUB_STOPPED || robotSubState == SUB_PAUSE_AT_STATION) &&
                        fabsf(rawDelta) < 2.0f && fabsf(currentSpeed) < 2.0f;
  if (stationaryZupt) {
    // ZUPT: Chỉ học bias con quay khi xe đứng yên thật (không chạy lúc đang phanh ngắn)
    gyroZOffset = 0.995f * gyroZOffset + 0.005f * gzRaw;
    ekfGyroBiasRadS *= 0.985f;
  } else {
    ekfPredictLocked(deltaS, deltaSL, deltaSR, gzRaw, isTurningInPlace, realDt);
    liveGzDegPerSec = mpuConnected
                        ? (rawDelta - ekfGyroBiasRadS * RAD2DEG_F)
                        : (((deltaSR - deltaSL) / (WHEEL_BASE_W_M * fmaxf(realDt, 0.002f))) * RAD2DEG_F);
  }

  currentSpeed = (deltaS / realDt) * 100.0f;
  totalDistM += fabsf(deltaS);

  lineMask = maskSnap;

  if (turnBrakeUntilMs > nowMs && isRunning) {
    smoothedBaseSpeed = 0.0f;
    portEXIT_CRITICAL(&stateMux);
    brakeMotorsHard();
    return;
  }

  if (stationBrakeUntilMs > nowMs) {
    smoothedBaseSpeed = 0.0f;
    portEXIT_CRITICAL(&stateMux);
    brakeMotorsHard();
    return;
  }

  // Post-Stop Pose Settling: Chốt tọa độ trạm sau khi phanh ngắn tắt hết quán tính
  if (pendingStationSnap && (fabsf(currentSpeed) < 3.0f || (nowMs - stationBrakeUntilMs) >= 80UL)) {
    pendingStationSnap = false;
    ekfSyncFromPoseLocked(pendingSnapX, pendingSnapY, pendingSnapYaw, false, 0.012f, 2.0f);
  }

  // TRẠNG THÁI 0: NGHỈ TẠI TRẠM (2 GIÂY TRONG CHẾ ĐỘ TRANSIT / AUTO LOOP)
  if (robotSubState == SUB_PAUSE_AT_STATION) {
    smoothedBaseSpeed = 0.0f;
    if (nowMs >= stationPauseUntilMs) {
      DynamicLeg nextDyn;
      float hErr = 0.0f;
      bool turning = false;
      if (queueLength > 0 && queueIndex + 1 < queueLength) {
        queueIndex++;
        nextDyn = legQueue[queueIndex];
      } else {
        queueLength = 0;
        queueIndex = 0;
        nextDyn = makeDynamicLegFromId(nextOfflineLegLocked(autoLoopReverse));
      }
      applyDynamicLegLocked(nextDyn, hErr, turning);
    }
    portEXIT_CRITICAL(&stateMux);
    setMotors(0, 0);
    return;
  }

  float liveMaskErr = (activeCount > 0)
                        ? ((float)(-2*sL2 + -1*sL1 + 0*sC + 1*sR1 + 2*sR2) / (float)activeCount)
                        : 0.0f;

  // TRẠNG THÁI 1A: XOAY TẠI TRẠM (KHÓA VẠCH + ĐIỀU TIẾT TỐC ĐỘ GÓC CHỐNG VỌT LỐ)
  if (robotSubState == SUB_TURNING_FOR_MISSION) {
    float err = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
    float absErr = fabsf(err);
    float rotatedSoFar = fabsf(normalizeAngleDeg(yawDeg - startTurnYaw));
    bool turnProgOk = (rotatedSoFar >= 20.0f || reverseSearchDoneForLeg);
    bool lineCaught = turnProgOk &&
                      (((absErr <= 30.0f) && ((lineMask & 0b01110) != 0)) ||
                       ((absErr <= 18.0f) && (lineMask != 0)));

    if (lineCaught) {
      if (absErr <= 2.5f && (lineMask & 0b00100) != 0) {
        ekfSyncFromPoseLocked(odomX, odomY, legTargetHeadingDeg, false, 0.015f, 1.5f);
      }
      enterLineFollowingFromTurnLocked(true, liveMaskErr);
      turnBrakeUntilMs = nowMs + 25;
      logCaught = true;
      snprintf(logLegBuf, sizeof(logLegBuf), "%s", currentLegName);
      logYaw = yawDeg;
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      Serial.printf("\n🎯 [KHÓA VẠCH THÀNH CÔNG] Đã bắt đúng vạch chặng %s (mask=%d) tại Yaw = %.1f° -> Khởi hành!\n",
                    logLegBuf, maskSnap, logYaw);
      return;
    }

    float stopTurnThresh = reverseSearchDoneForLeg ? 3.5f : 2.5f;
    if (absErr <= stopTurnThresh) {
      if (lineMask != 0) {
        float snapH = ((lineMask & 0b00100) != 0) ? legTargetHeadingDeg : yawDeg;
        ekfSyncFromPoseLocked(odomX, odomY, snapH, false, 0.015f, 2.0f);
        enterLineFollowingFromTurnLocked(true, liveMaskErr);
        turnBrakeUntilMs = nowMs + 25;
      } else if (!reverseSearchDoneForLeg) {
        smoothedBaseSpeed = 0.0f;
        reverseSearchDoneForLeg = true;
        turnSearchStage = 1;
        robotSubState = SUB_TURNING_REVERSE_SEARCH;
        robotStateStr = "TURNING_STATION";
        turnBrakeUntilMs = nowMs + 35;
        turnSearchStartMs = turnBrakeUntilMs;
      } else {
        // Đã quét 2 phía nhưng đầu chặng mất line -> vào Stanley Bridge giữ hướng thẳng về tim đường
        enterLineFollowingFromTurnLocked(false, 0.0f);
        turnBrakeUntilMs = nowMs + 25;
      }
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }

    int turnSpeed = computeRegulatedTurnPwm(absErr, reverseSearchDoneForLeg, fabsf(liveGzDegPerSec));
    lastTurnSpinDir = (err >= 0.0f) ? 1 : -1;
    if (turnSpeed < 0) {
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }
    if (err > 0.0f) { outLeftPwm = -turnSpeed; outRightPwm = turnSpeed; }
    else { outLeftPwm = turnSpeed; outRightPwm = -turnSpeed; }
    portEXIT_CRITICAL(&stateMux);
    setMotors(outLeftPwm, outRightPwm);
    return;
  }

  // TRẠNG THÁI 1B: QUÉT TÌM VẠCH ĐỐI XỨNG ±16° (~±9.9cm tại thanh IR) KHI QUAY LỐ / MẤT LINE GÓC TRẠM
  if (robotSubState == SUB_TURNING_REVERSE_SEARCH) {
    float revAbsErr = fabsf(normalizeAngleDeg(legTargetHeadingDeg - yawDeg));
    bool found = (((lineMask & 0b01110) != 0) && revAbsErr <= 30.0f) ||
                 ((lineMask != 0) && revAbsErr <= 22.0f);

    if (found) {
      enterLineFollowingFromTurnLocked(true, liveMaskErr);
      turnBrakeUntilMs = nowMs + 25;
      logCaught = true;
      snprintf(logLegBuf, sizeof(logLegBuf), "%s", currentLegName);
      logYaw = yawDeg;
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      Serial.printf("\n🎯 [QUÉT KHÓA VẠCH] Bắt trúng vạch chặng %s (mask=%d) tại Yaw = %.1f° -> Bám line!\n",
                    logLegBuf, maskSnap, logYaw);
      return;
    }

    int activeSweepDir = (turnSearchStage <= 1) ? reverseSearchPhase1Dir : -reverseSearchPhase1Dir;
    if (activeSweepDir == 0) activeSweepDir = 1;
    float progressAlongSweep = (float)activeSweepDir * normalizeAngleDeg(yawDeg - legTargetHeadingDeg);
    unsigned long phaseTimeoutMs = (turnSearchStage <= 1) ? 900UL : 1500UL;

    if (progressAlongSweep >= 16.0f || (nowMs - turnSearchStartMs > phaseTimeoutMs)) {
      if (turnSearchStage <= 1) {
        turnSearchStage = 2;
        turnBrakeUntilMs = nowMs + 35;
        turnSearchStartMs = turnBrakeUntilMs;
      } else {
        smoothedBaseSpeed = 0.0f;
        turnSearchStage = 0;
        robotSubState = SUB_TURNING_FOR_MISSION;
        robotStateStr = "TURNING_STATION";
        isRunning = true;
        turnBrakeUntilMs = nowMs + 35;
      }
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }

    int sweepSpeed = computeRegulatedTurnPwm(0.0f, true, fabsf(liveGzDegPerSec));
    lastTurnSpinDir = activeSweepDir;
    if (sweepSpeed < 0) {
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }
    outLeftPwm  = -activeSweepDir * sweepSpeed;
    outRightPwm =  activeSweepDir * sweepSpeed;
    portEXIT_CRITICAL(&stateMux);
    setMotors(outLeftPwm, outRightPwm);
    return;
  }

  // TRẠNG THÁI 1C: XOAY THỦ CÔNG
  if (robotSubState == SUB_TURNING_MANUAL) {
    float err = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
    float absErr = fabsf(err);

    if (absErr <= 2.5f) {
      smoothedBaseSpeed = 0.0f;
      ekfSyncFromPoseLocked(odomX, odomY, legTargetHeadingDeg, false, 0.015f, 1.5f);
      robotSubState = SUB_STOPPED;
      robotStateStr = "STOPPED";
      isRunning = false;
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }

    int turnSpeed = computeRegulatedTurnPwm(absErr, absErr <= 28.0f, fabsf(liveGzDegPerSec));
    if (turnSpeed < 0) {
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }
    if (err > 0.0f) { outLeftPwm = -turnSpeed; outRightPwm = turnSpeed; }
    else { outLeftPwm = turnSpeed; outRightPwm = -turnSpeed; }
    portEXIT_CRITICAL(&stateMux);
    setMotors(outLeftPwm, outRightPwm);
    return;
  }

  // TRẠNG THÁI 2: TEST ĐỘNG CƠ 1 GIÂY
  if (testMotorUntilMs > nowMs) {
    robotStateStr = "MOTOR_TEST";
    portEXIT_CRITICAL(&stateMux);
    setMotors(230, 230);
    return;
  }

  // Giữ nguyên trạng thái đích/lỗi ("AT_STATION", "GAP_TIMEOUT_STOP", "STOPPED") khi đang dừng
  if (!isRunning || robotSubState == SUB_STOPPED) {
    smoothedBaseSpeed = 0.0f;
    bridgeTurnAligning = false;
    portEXIT_CRITICAL(&stateMux);
    setMotors(0, 0);
    return;
  }

  // TRẠNG THÁI 3: THEO DÕI TIẾN ĐỘ CHẶNG & TIẾP CẬN GÓC TRẠM
  if (useGlobalLineProgress) {
    float globalAlong = computeLegAlongM();
    if (inGapBridgeMode && !hasSeenLineOnLeg) {
      if (globalAlong > legProgressM) legProgressM = globalAlong;
    } else {
      legProgressM = globalAlong;
    }
  } else {
    float relRad = (normalizeAngleDeg(yawDeg - legTargetHeadingDeg) * PI) / 180.0f;
    float fwdDelta = deltaS * cosf(relRad);
    if (inGapBridgeMode && !hasSeenLineOnLeg) {
      if (fwdDelta > 0.0f) legProgressM += fwdDelta;
    } else {
      legProgressM += fwdDelta;
    }
  }
  float remainingLegM = legTargetDistM - legProgressM;
  float sensorReachCornerDistM = legTargetDistM - DIST_AXLE_TO_SENSOR_M;

  int leftPwm  = 0;
  int rightPwm = 0;

  if (legProgressM >= sensorReachCornerDistM || cornerApproachLatched) {
    bridgeTurnAligning = false;
    float curCrossCorner = computeLegCrossTrackM();
    float yawErrCorner = fabsf(normalizeAngleDeg(legTargetHeadingDeg - yawDeg));
    bool nearLineRecent = isCorridorValidNearCornerLocked(lastConfirmedLineProgM, curCrossCorner, yawErrCorner, false);
    bool onValidCorridor = currentTargetHasCoord
                             ? (fabsf(curCrossCorner) <= 0.16f && yawErrCorner <= 20.0f)
                             : (yawErrCorner <= 22.0f);
    bool cornerEntryAllowed = cornerApproachLatched ||
                              nearLineRecent ||
                              (onValidCorridor && (activeCount >= 2 || (activeCount == 1 && (consecutiveGoodLineFrames >= 1 || hasSeenLineOnLeg))));
    if (!cornerEntryAllowed) {
      enterGapTimeoutStopLocked(nowMs);
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      Serial.printf("\n❌ [LẠC LINE CUỐI CHẶNG] %s | cross=%.2fm | yawErr=%.1f° -> Dừng an toàn!\n",
                    currentLegName, curCrossCorner, yawErrCorner);
      return;
    }

    float absSpd = fabsf(currentSpeed);
    float stopLeadM = (absSpd > 22.0f) ? 0.024f : ((absSpd > 14.0f) ? 0.015f : 0.006f);
    if (remainingLegM <= stopLeadM) {
      bool lineRecentNearEnd = isCorridorValidNearCornerLocked(lastConfirmedLineProgM, curCrossCorner, yawErrCorner, true);
      smoothedBaseSpeed = 0.0f;
      cornerApproachLatched = false;
      inGapBridgeMode = false;
      gapStartAlongM = 0.0f;
      hasSeenLineOnLeg = false;
      lineTrackedOnLeg = false;
      lastConfirmedLineProgM = -1.0f;
      bridgeExitLineErr = 0.0f;
      consecutiveGoodLineFrames = 0;
      resetWheelCascadeLocked();

      snprintf(currentStationName, sizeof(currentStationName), "%s", currentTargetName);
      currentStation = parseStationId(currentStationName);
      if (lineRecentNearEnd) {
        float snapX = odomX;
        float snapY = odomY;
        if (currentTargetHasCoord) {
          snapX = currentTargetX;
          snapY = currentTargetY;
        } else if (currentStation < STATION_UNKNOWN) {
          snapX = STATION_COORDS[currentStation].x;
          snapY = STATION_COORDS[currentStation].y;
        }
        float snapYaw = ((activeCount >= 3) || (yawErrCorner <= 3.5f)) ? legTargetHeadingDeg : yawDeg;
        ekfSyncFromPoseLocked(snapX, snapY, snapYaw, false, 0.012f, 2.0f);
        // Post-Stop Settling: Chốt lại tọa độ sau khi phanh ngắn 110ms tắt hết quán tính
        pendingSnapX = snapX;
        pendingSnapY = snapY;
        pendingSnapYaw = snapYaw;
        pendingStationSnap = true;
      } else {
        pendingStationSnap = false;
      }

      logArrived = true;
      snprintf(logLegBuf, sizeof(logLegBuf), "%s", currentLegName);
      snprintf(logStBuf, sizeof(logStBuf), "%s", currentStationName);
      logX = odomX;
      logY = odomY;
      logYaw = yawDeg;

      if (queueLength > 0 && queueIndex + 1 < queueLength) {
        stationPauseUntilMs = nowMs + 1500;
        robotSubState = SUB_PAUSE_AT_STATION;
        robotStateStr = "PAUSED_TRANSIT";
        isRunning = true;
      } else if (autoLoopEnabled) {
        queueLength = 0;
        queueIndex = 0;
        stationPauseUntilMs = nowMs + 2000;
        robotSubState = SUB_PAUSE_AT_STATION;
        robotStateStr = "PAUSED_LOOP";
        isRunning = true;
      } else {
        queueLength = 0;
        queueIndex = 0;
        robotSubState = SUB_STOPPED;
        robotStateStr = "AT_STATION";
        isRunning = false;
      }

      stationBrakeUntilMs = nowMs + 110;
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      Serial.printf("\n🏁 [ĐÃ TỚI ĐÍCH CHẶNG: %s] Xe đang ở TRẠM: %s | Tọa độ: X=%.3fm, Y=%.3fm, Yaw=%.1f°\n",
                    logLegBuf, logStBuf, logX, logY, logYaw);
      return;
    }

    // Hồ sơ giảm tốc căn bậc hai + điều chế phanh ngắn tránh trượt lố 6cm qua trạm
    float targetCornerSpdCmS = constrain(11.0f + 35.0f * sqrtf(fmaxf(remainingLegM, 0.0f)), 13.0f, 32.0f);
    if (absSpd > targetCornerSpdCmS && remainingLegM > 0.025f) {
      robotStateStr = "APPROACH_CORNER";
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }

    int approachBase = (remainingLegM <= 0.16f && absSpd >= 5.0f) ? (minMovePwm + 8) : brakePwm;
    float cornerHeading = legTargetHeadingDeg;
    if (currentTargetHasCoord && remainingLegM > 0.06f) {
      float alongRem = legTargetDistM - computeLegAlongM();
      if (alongRem > 0.06f) {
        float rawCornerDeg = atan2f(currentTargetY - odomY, currentTargetX - odomX) * (180.0f / PI);
        float aimSteerDeg  = constrain(normalizeAngleDeg(rawCornerDeg - legTargetHeadingDeg), -18.0f, 18.0f);
        cornerHeading = normalizeAngleDeg(legTargetHeadingDeg + aimSteerDeg);
      } else {
        float curCrossM = computeLegCrossTrackM();
        float aimSteerDeg = constrain(atan2f(-curCrossM, fmaxf(remainingLegM, 0.16f)) * (180.0f / PI), -18.0f, 18.0f);
        cornerHeading = normalizeAngleDeg(legTargetHeadingDeg + aimSteerDeg);
      }
    }
    computeCornerApproachPwm(cornerHeading, yawDeg, approachBase, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
    robotStateStr = "APPROACH_CORNER";
    portEXIT_CRITICAL(&stateMux);
    setMotors(leftPwm, rightPwm);
    return;
  }

  // GIAI ĐOẠN 3.2: XỬ LÝ VƯỢT ĐỨT ĐOẠN (EKF + STANLEY + QUÉT SIN THÍCH NGHI + CASCADE BÁNH XE) & BÁM LINE ROBUST
  if (activeCount == 0) {
    if (turnCatchGraceUntilMs > nowMs && hasSeenLineOnLeg) {
      int8_t recenterDir = 0;
      if (fabsf(lastValidError) >= 0.4f) {
        recenterDir = (lastValidError < 0.0f) ? 1 : -1;
      } else if (lastTurnSpinDir != 0) {
        recenterDir = -lastTurnSpinDir;
      } else {
        recenterDir = (normalizeAngleDeg(legTargetHeadingDeg - yawDeg) >= 0.0f) ? 1 : -1;
      }
      int recenterPwm = computeRegulatedTurnPwm(0.0f, true, fabsf(liveGzDegPerSec));
      if (recenterPwm < 0) {
        portEXIT_CRITICAL(&stateMux);
        brakeMotorsHard();
        return;
      }
      leftPwm  = -recenterDir * recenterPwm;
      rightPwm =  recenterDir * recenterPwm;
      robotStateStr = "TURNING_LINE";
      portEXIT_CRITICAL(&stateMux);
      setMotors(leftPwm, rightPwm);
      return;
    }

    consecutiveZeroLineFrames++;

    if (hasSeenLineOnLeg && consecutiveZeroLineFrames <= 3 && !inGapBridgeMode) {
      int u = constrain((int)(Kp_line * lastValidError * 0.35f), -35, 35);
      leftPwm  = constrain((int)(smoothedBaseSpeed + u), minMovePwm, maxPwm);
      rightPwm = constrain((int)(smoothedBaseSpeed - u), minMovePwm, maxPwm);
      robotStateStr = "LINE_FLICKER_FILTER";
      portEXIT_CRITICAL(&stateMux);
      setMotors(leftPwm, rightPwm);
      return;
    }

    if (!inGapBridgeMode) {
      inGapBridgeMode = true;
      gapStartTimeMs = nowMs;
      gapStartDistM = totalDistM;
      gapStartAlongM = legProgressM;
      bridgeExitLineErr = hasSeenLineOnLeg ? lastValidError : 0.0f;
      lastValidError = 0.0f;
      consecutiveGoodLineFrames = 0;
      probeStage = 0;
      probeStageStartDistM = totalDistM;
      float initSensCrossM = computeLegSensorCrossTrackM();
      if (hasSeenLineOnLeg && fabsf(bridgeExitLineErr) >= 0.45f) {
        float refX = currentTargetHasCoord ? currentTargetX : legStartX;
        float refY = currentTargetHasCoord ? currentTargetY : legStartY;
        ekfUpdateOpticalLineLocked(bridgeExitLineErr, 1, legTargetHeadingDeg, refX, refY, true);
        initSensCrossM = computeLegSensorCrossTrackM();
        probeSweepSign = (bridgeExitLineErr < 0.0f) ? 1 : -1;
      } else {
        probeSweepSign = (initSensCrossM > 0.015f) ? -1 : 1;
      }
      bool dummyStageChg = false;
      int dummyBasePwm = bridgePwm;
      bool initSkid = hasSeenLineOnLeg && (fabsf(bridgeExitLineErr) >= 1.15f);
      computeBridgeCascadeTargetLocked(initSensCrossM, 0.0f, remainingLegM, initSkid, gapHeadingHoldDeg, dummyBasePwm, dummyStageChg);
      lastGapHeadingErr = normalizeAngleDeg(gapHeadingHoldDeg - yawDeg);
      float alignThreshDeg = (!hasSeenLineOnLeg || fabsf(currentSpeed) < 12.0f) ? 20.0f : 26.0f;
      bridgeTurnAligning = (fabsf(lastGapHeadingErr) > alignThreshDeg);
      if (smoothedBaseSpeed < (float)curvePwm) {
        smoothedBaseSpeed = (float)curvePwm;
      }
      logGapStart = true;
      snprintf(logLegBuf, sizeof(logLegBuf), "%s", currentLegName);
      logYaw = gapHeadingHoldDeg;
    }

    float gapTraveled = totalDistM - gapStartDistM;
    float gapAlong = fmaxf(0.0f, legProgressM - gapStartAlongM);
    bool isSideSkidExit = hasSeenLineOnLeg && (fabsf(bridgeExitLineErr) >= 1.15f);
    float maxGapDist = isSideSkidExit ? 0.55f : MAX_BRIDGE_DISTANCE_M;
    unsigned long maxGapMs = isSideSkidExit ? 3200UL : MAX_BRIDGE_TIME_MS;
    float bridgeCrossM = computeLegCrossTrackM();
    float bridgeCrossSensM = computeLegSensorCrossTrackM();
    bool driftedFarOffLine = currentTargetHasCoord &&
                             ((fabsf(bridgeCrossSensM) > 0.25f) || (fabsf(bridgeCrossM) > 0.22f)) &&
                             (gapAlong >= 0.22f);

    if (gapAlong > maxGapDist || gapTraveled > (maxGapDist + 0.22f) ||
        (nowMs - gapStartTimeMs > maxGapMs) || driftedFarOffLine) {
      bridgeTurnAligning = false;
      float yawErrBridge = fabsf(normalizeAngleDeg(legTargetHeadingDeg - yawDeg));
      bool corridorOkAtEnd = (!currentTargetHasCoord || fabsf(bridgeCrossM) <= 0.15f) && (yawErrBridge <= 18.0f);
      bool canEnterCornerSafely = (legTargetDistM > 0.45f) &&
                                  (remainingLegM <= ((legTargetDistM > 0.70f) ? 0.42f : 0.35f)) &&
                                  !driftedFarOffLine &&
                                  corridorOkAtEnd &&
                                  (isCorridorValidNearCornerLocked(lastConfirmedLineProgM, bridgeCrossM, yawErrBridge, true) || !hasSeenLineOnLeg);
      if (canEnterCornerSafely) {
        inGapBridgeMode = false;
        consecutiveZeroLineFrames = 0;
        cornerApproachLatched = true;
        computeCornerApproachPwm(legTargetHeadingDeg, yawDeg, brakePwm, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
        robotStateStr = "APPROACH_CORNER";
        portEXIT_CRITICAL(&stateMux);
        setMotors(leftPwm, rightPwm);
        return;
      }
      enterGapTimeoutStopLocked(nowMs);
      logGapTimeout = true;
      logGapVal = gapTraveled;
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      Serial.printf("\n❌ [DỪNG AN TOÀN] Mất vạch %.2fm (lệch ngang=%.2fm)!\n", logGapVal, bridgeCrossSensM);
      return;
    }

    float curCrossSensM = computeLegSensorCrossTrackM();
    bool probeStageChanged = false;
    float desiredBridgeHeadingDeg = legTargetHeadingDeg;
    int targetBase = bridgePwm;
    computeBridgeCascadeTargetLocked(curCrossSensM, gapAlong, remainingLegM, isSideSkidExit,
                                     desiredBridgeHeadingDeg, targetBase, probeStageChanged);
    float hErr = normalizeAngleDeg(desiredBridgeHeadingDeg - yawDeg);

    if (bridgeTurnAligning) {
      if (fabsf(hErr) <= 4.0f) {
        bridgeTurnAligning = false;
        lastGapHeadingErr = hErr;
        smoothedBaseSpeed = (float)curvePwm;
        turnBrakeUntilMs = nowMs + 25;
        portEXIT_CRITICAL(&stateMux);
        brakeMotorsHard();
        return;
      } else {
        int alignPwm = computeRegulatedTurnPwm(fabsf(hErr), true, fabsf(liveGzDegPerSec));
        if (alignPwm < 0) {
          portEXIT_CRITICAL(&stateMux);
          brakeMotorsHard();
          return;
        }
        if (hErr > 0.0f) { leftPwm = -alignPwm; rightPwm = alignPwm; }
        else { leftPwm = alignPwm; rightPwm = -alignPwm; }
        robotStateStr = "DEAD_RECKONING_BRIDGE";
        portEXIT_CRITICAL(&stateMux);
        setMotors(leftPwm, rightPwm);
        return;
      }
    }

    lastGapHeadingErr = hErr;
    float omegaRefRadS = constrain(3.0f * (hErr * DEG2RAD_F), -1.35f, 1.35f);

    if (targetBase > smoothedBaseSpeed) smoothedBaseSpeed = min((float)targetBase, smoothedBaseSpeed + MAX_RAMP_STEP);
    else if (targetBase < smoothedBaseSpeed) smoothedBaseSpeed = max((float)targetBase, smoothedBaseSpeed - MAX_RAMP_STEP);

    computeLoadedCascadePwm(smoothedBaseSpeed, omegaRefRadS, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
    robotStateStr = "DEAD_RECKONING_BRIDGE";
    portEXIT_CRITICAL(&stateMux);
    setMotors(leftPwm, rightPwm);
    if (logGapStart) {
      Serial.printf("\n⚡ [STANLEY+EKF BRIDGE] Dẫn đường vòng kín %s | Hướng mục tiêu = %.1f°\n",
                    logLegBuf, logYaw);
    }
    return;

  } else {
    hasSeenLineOnLeg = true;
    bridgeTurnAligning = false;
    probeStage = 0;
    consecutiveZeroLineFrames = 0;

    bool recoveringFromBridge = inGapBridgeMode;
    if (inGapBridgeMode) {
      logGapVal = totalDistM - gapStartDistM;
      inGapBridgeMode = false;
      logGapRecovered = true;
      snprintf(logLegBuf, sizeof(logLegBuf), "%s", currentLegName);
    }

    lineLostTimeMs = 0;

    currentLineErr = (float)(-2*sL2 + -1*sL1 + 0*sC + 1*sR1 + 2*sR2) / (float)activeCount;
    lastValidError = currentLineErr;
    if (recoveringFromBridge) {
      // Triệt tiêu xung dErr giả ngay khung hình đầu tiên vừa bắt lại vạch
      lastLineError = currentLineErr;
    }

    float yawErrToLeg = fabsf(normalizeAngleDeg(legTargetHeadingDeg - yawDeg));
    if (fabsf(currentLineErr) <= 1.25f && yawErrToLeg <= 14.0f) {
      if (consecutiveGoodLineFrames < 50) consecutiveGoodLineFrames++;
    } else {
      consecutiveGoodLineFrames = 0;
    }
    if (consecutiveGoodLineFrames >= 2 || (fabsf(currentLineErr) <= 0.70f && yawErrToLeg <= 14.0f)) {
      lineTrackedOnLeg = true;
      lastConfirmedLineProgM = legProgressM;
    }

    float dErr = (currentLineErr - lastLineError) / fmaxf(realDt, 0.001f);
    lastLineError = currentLineErr;

    if (fabsf(currentLineErr) < 0.6f && yawErrToLeg <= 15.0f) {
      bridgeExitLineErr = 0.0f;
    }

    // Cập nhật vị trí ngang & góc hướng thật qua ma trận hiệp phương sai EKF (không xóa lệch góc giả về 90°)
    if (yawErrToLeg <= 26.0f && activeCount >= 1 && activeCount <= 3) {
      float refX = currentTargetHasCoord ? currentTargetX : legStartX;
      float refY = currentTargetHasCoord ? currentTargetY : legStartY;
      ekfUpdateOpticalLineLocked(currentLineErr, activeCount, legTargetHeadingDeg, refX, refY, recoveringFromBridge);
    }

    float absErr = fabsf(currentLineErr);
    int targetBase = straightPwm - (int)(absErr * (straightPwm - curvePwm));
    targetBase = constrain(targetBase, curvePwm, straightPwm);

    if (remainingLegM < 0.40f) {
      targetBase = min(targetBase, brakePwm);
      robotStateStr = "BRAKING_NEAR_STATION";
      if (remainingLegM < 0.36f && fabsf(currentSpeed) > 36.0f) {
        portEXIT_CRITICAL(&stateMux);
        brakeMotorsHard();
        return;
      }
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
      float slope = (float)max(straightPwm - minMovePwm, 10) / 0.26f;
      float omegaRefRadS = constrain(-(2.0f * u) / (slope * WHEEL_BASE_W_M), -2.4f, 2.4f);
      computeLoadedCascadePwm(smoothedBaseSpeed, omegaRefRadS, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
    }
  }

  portEXIT_CRITICAL(&stateMux);
  setMotors(leftPwm, rightPwm);
  if (logGapRecovered) {
    Serial.printf("\n🎯 [BẮT LẠI VẠCH] Đã vượt qua khoảng đứt %.2fm -> Tiếp tục bám line %s!\n",
                  logGapVal, logLegBuf);
  }
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

bool extractJsonStrField(const String &obj, const char* key, char* dest, size_t maxLen) {
  String pat = String("\"") + key + "\":";
  int idx = obj.indexOf(pat);
  if (idx < 0) return false;
  int pos = idx + pat.length();
  while (pos < (int)obj.length() && (obj.charAt(pos) == ' ' || obj.charAt(pos) == '\t')) pos++;
  if (pos >= (int)obj.length() || obj.charAt(pos) != '"') return false;
  int endQ = obj.indexOf('"', pos + 1);
  if (endQ <= pos) return false;
  String val = obj.substring(pos + 1, endQ);
  val.trim();
  snprintf(dest, maxLen, "%s", val.c_str());
  return dest[0] != '\0';
}

bool extractJsonFloatField(const String &obj, const char* key, float &outVal) {
  String pat = String("\"") + key + "\":";
  int idx = obj.indexOf(pat);
  if (idx < 0) return false;
  int pos = idx + pat.length();
  while (pos < (int)obj.length() && (obj.charAt(pos) == ' ' || obj.charAt(pos) == '\t')) pos++;
  if (pos >= (int)obj.length()) return false;
  char c = obj.charAt(pos);
  if (c != '-' && c != '+' && !isDigit(c)) return false;
  outVal = obj.substring(pos).toFloat();
  return true;
}

int parseRouteLegsFromResponse(const String &resp, DynamicLeg outLegs[], int maxLegs) {
  int rlIdx = resp.indexOf("\"route_legs\":");
  if (rlIdx < 0) return 0;
  int arrStart = resp.indexOf('[', rlIdx);
  int arrEnd   = resp.indexOf(']', arrStart);
  if (arrStart < 0 || arrEnd <= arrStart + 1) return 0;

  int count = 0;
  int cursor = arrStart + 1;
  while (cursor < arrEnd && count < maxLegs) {
    int objStart = resp.indexOf('{', cursor);
    if (objStart < 0 || objStart >= arrEnd) break;
    int objEnd = resp.indexOf('}', objStart);
    if (objEnd < 0 || objEnd > arrEnd) break;

    String objStr = resp.substring(objStart, objEnd + 1);
    DynamicLeg leg;
    memset(&leg, 0, sizeof(leg));
    bool okFrom = extractJsonStrField(objStr, "from", leg.fromName, sizeof(leg.fromName));
    bool okTo   = extractJsonStrField(objStr, "to",   leg.toName,   sizeof(leg.toName));
    bool okDist = extractJsonFloatField(objStr, "dist", leg.distM);
    bool okHead = extractJsonFloatField(objStr, "head", leg.headingDeg);
    bool okTx   = extractJsonFloatField(objStr, "tx",   leg.targetX);
    bool okTy   = extractJsonFloatField(objStr, "ty",   leg.targetY);
    leg.hasTargetCoord = (okTx && okTy);

    if (okFrom && okTo && okDist && okHead && leg.distM > 0.05f) {
      outLegs[count++] = leg;
    }
    cursor = objEnd + 1;
  }
  return count;
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
      char  snapStation[12];
      char  snapLeg[28];
      snprintf(snapStation, sizeof(snapStation), "%s", currentStationName);
      snprintf(snapLeg, sizeof(snapLeg), "%s", currentLegName);
      bool  snapAutoLoop = autoLoopEnabled;
      const char* snapStatePtr = robotStateStr ? robotStateStr : "STOPPED";
      portEXIT_CRITICAL(&stateMux);

      String snapState = String(snapStatePtr);

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
      payload += "\"station\":\"" + String(snapStation) + "\",";
      payload += "\"leg\":\"" + String(snapLeg) + "\",";
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
            resetStopStateLocked("STOPPED");
            portEXIT_CRITICAL(&stateMux);
            stopMotors();
          } else if (cmdStr == "RESET_ODOM") {
            stopMotors();
            noInterrupts(); rawTicksL = 0; rawTicksR = 0; interrupts();
            portENTER_CRITICAL(&stateMux);
            ekfSyncFromPoseLocked(0.0f, 0.0f, 90.0f, true, 0.010f, 1.5f);
            totalDistM = 0.0f; legProgressM = 0.0f;
            currentLeg = LEG_AB;
            currentStation = STATION_A;
            snprintf(currentStationName, sizeof(currentStationName), "A");
            snprintf(currentTargetName, sizeof(currentTargetName), "B");
            snprintf(currentLegName, sizeof(currentLegName), "A -> B");
            currentTargetX = STATION_COORDS[STATION_B].x;
            currentTargetY = STATION_COORDS[STATION_B].y;
            currentTargetHasCoord = true;
            legTargetDistM = activeMapLengthM;
            legTargetHeadingDeg = 90.0f;
            legStartX = 0.0f;
            legStartY = 0.0f;
            legLineOriginX = 0.0f;
            legLineOriginY = 0.0f;
            useGlobalLineProgress = true;
            autoLoopReverse = false;
            simulateGapActive = false;
            hasSeenLineOnLeg = false;
            lineTrackedOnLeg = false;
            lastConfirmedLineProgM = -1.0f;
            bridgeExitLineErr = 0.0f;
            reverseSearchDoneForLeg = false;
            turnSearchStage = 0;
            probeStage = 0;
            probeSweepSign = 1;
            reverseSearchPhase1Dir = 1;
            probeStageStartDistM = 0.0f;
            lastTurnSpinDir = 0;
            consecutiveZeroLineFrames = 0;
            consecutiveGoodLineFrames = 0;
            lastLineError = 0.0f;
            lastValidError = 0.0f;
            prevTicksL = 0; prevTicksR = 0;
            resetStopStateLocked("STOPPED");
            portEXIT_CRITICAL(&stateMux);
            Serial.println(">> [LỆNH]: RESET ODOMETRY VỀ GỐC A (0,0) 90°!");
          } else if (cmdStr == "CALIBRATE_GYRO") {
            requestGyroCalib = true;
            Serial.println(">> [LỆNH]: Xếp lịch hiệu chỉnh con quay trên Core 1...");
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
          // 4. Lập lộ trình Điểm - Điểm tự động (Ưu tiên route_legs Dijkstra từ Server, dự phòng offline)
          else if (cmdStr.startsWith("GO_TO:")) {
            String targetName = cmdStr.substring(6);
            targetName.trim();
            targetName.toUpperCase();

            DynamicLeg parsedLegs[MAX_ROUTE_LEGS];
            int nLegs = parseRouteLegsFromResponse(resp, parsedLegs, MAX_ROUTE_LEGS);
            if (nLegs > 0) {
              executeServerRouteLegs(parsedLegs, nLegs);
            } else {
              StationId destSt = parseStationId(targetName.c_str());
              if (targetName.equals(String(snapStation))) {
                stopMotors();
                portENTER_CRITICAL(&stateMux);
                queueLength = 0;
                queueIndex = 0;
                robotSubState = SUB_STOPPED;
                robotStateStr = "AT_STATION";
                isRunning = false;
                portEXIT_CRITICAL(&stateMux);
                Serial.printf("\n>> [THÔNG BÁO] Xe đang ở sẵn trạm đích: %s!\n", targetName.c_str());
              } else {
                planAndExecuteRoute(currentStation, destSt);
              }
            }
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
          // 6. Xoay thủ công & Giả lập đứt vạch (Trái CCW = +90°, Phải CW = -90°)
          else if (cmdStr == "TURN_LEFT_90") {
            startManualTurn(90.0f);
          } else if (cmdStr == "TURN_RIGHT_90") {
            startManualTurn(-90.0f);
          } else if (cmdStr == "TURN_180") {
            startManualTurn(180.0f);
          } else if (cmdStr == "SIMULATE_GAP") {
            portENTER_CRITICAL(&stateMux);
            simulateGapActive = true;
            simGapStartLegDistM = legProgressM;
            portEXIT_CRITICAL(&stateMux);
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

  ekfSyncFromPoseLocked(0.00f, 0.00f, 90.0f, true, 0.010f, 1.5f);
  resetWheelCascadeLocked();
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
      bool nowRunning = false;
      if (isRunning) {
        stopMotors();
        portENTER_CRITICAL(&stateMux);
        resetStopStateLocked("STOPPED");
        nowRunning = false;
        portEXIT_CRITICAL(&stateMux);
      } else {
        startMission(currentLeg);
        nowRunning = true;
      }
      Serial.printf("\n[SERIAL KEY]: %s\n", nowRunning ? "START" : "STOP");
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
        portENTER_CRITICAL(&stateMux);
        simulateGapActive = true;
        simGapStartLegDistM = legProgressM;
        portEXIT_CRITICAL(&stateMux);
        Serial.printf("\n⚡ [TEST ĐỨT VẠCH]: Bật giả lập đứt 25cm tại %.2fm!\n", legProgressM);
      }
    }
    else if (ch == 'z' || ch == 'Z') {
      liveEnergyMah = 0.0f;
      liveEnergyWh = 0.0f;
      Serial.println("\n⚡ [SERIAL KEY]: ĐÃ reset năng lượng tiêu thụ về 0.0 mAh!");
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
        portENTER_CRITICAL(&stateMux);
        resetStopStateLocked("STOPPED");
        portEXIT_CRITICAL(&stateMux);
      } else {
        advanceToNextLeg();
      }
    }
  }
  lastBtn = reading;

  vTaskDelay(pdMS_TO_TICKS(50));
}
