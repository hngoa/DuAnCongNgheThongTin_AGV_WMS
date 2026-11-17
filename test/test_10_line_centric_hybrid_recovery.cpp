#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <math.h>
#include <esp_arduino_version.h>
#include "soc/usb_serial_jtag_reg.h"
#include "driver/gpio.h"
#include "mbedtls/md.h"
#include "secrets.h"

/*
 * ===================================================================================
 * AGV_Ver2 - TEST 10: CÂN BẰNG ỔN ĐỊNH TRẠM (TEST 09) & BÁM LINE TỰ DO (CONG / ZIGZAG)
 * -----------------------------------------------------------------------------------
 * ĐẶC TẢ KIẾN TRÚC LAI THÍCH NGHI (CHỈ CẢM BIẾN TRÊN XE, GIỮ NGUYÊN SÀN PWM TẢI):
 *   1. MPU-6050 LÀ TRỤC CHUẨN GÓC + EKF QUANG HỌC CÓ CỔNG NHẤT QUÁN ĐỘNG HỌC:
 *      - Duy trì pureMpuYawDeg song song để 5-IR KHÔNG BAO GIỜ kéo hỏng góc thật khi rẽ
 *        nhánh chéo < (giới hạn lệch góc quang học <= ±2.5°, ngắt ngay nếu |pureYaw| > 14°).
 *      - Trên tuyến thẳng đã kiểm chứng nhất quán với MPU (<= 7°, |Gz| <= 12°/s), neo EKF
 *        quang học giữ ổn định di chuyển trạm như Test 09.
 *   2. PHÂN TÁCH MẤT LINE THẲNG VS GÓC GẬP / LINE BIẾN DẠNG + NÂNG CẤP BÁM LINE:
 *      - Lấy mẫu IR đa số 5 lần lọc chói gạch bóng; nhận diện mẫu góc 7/28/15/30 -> ±1.75;
 *        mở khóa tốc độ bánh trong khi ôm cua & pivot sớm từ |err| >= 1.40.
 *      - Đứt vạch thẳng -> Bridge quét sin đối xứng 0.72m (tối đa 1.12m).
 *      - Mất ở góc gập/< (lịch sử 120ms + MPU |Gz|) -> SUB_CORNER_BEND_CATCH dùng MPU quét
 *        cung hẹp có giới hạn (±78°/±88°) đón nhánh mới, rồi mới chuyển Bridge nếu đứt hẳn.
 *   3. DỪNG TRẠM CHUẨN XÁC & BẢO TOÀN GÓC GYRO + KHÓA ZUPT CƠ KHÍ:
 *      - Chặn dừng nhầm khi lệch ngang trạm (> 24cm), ưu tiên bám line quang học khi vào trạm,
 *        không snapYaw giả khi sai > 2.5° và đóng băng yaw khi đứng yên.
 * ===================================================================================
 */

// --- 1. CẤU HÌNH KẾT NỐI WI-FI, TÊN MIỀN mDNS & BẢO MẬT TOKEN + HMAC-SHA256 (NẠP TỪ include/secrets.h) ---
const char* wifi_ssid        = WIFI_SSID_CFG;
const char* wifi_pass        = WIFI_PASS_CFG;
const char* mdns_hostname    = MDNS_HOSTNAME_CFG;                 // Tên miền mDNS duy nhất: agv-wms.local
const char* agv_device_token = AGV_DEVICE_TOKEN_CFG;              // Mã khóa bảo mật RBAC của Robot AGV
const char* agv_hmac_secret  = AGV_HMAC_SECRET_CFG;               // Khóa ký số HMAC-SHA256 chống giả mạo lệnh
String      serverUrl        = "http://agv-wms.local:5000/api/telemetry"; // Kết nối qua tên miền mDNS chuẩn

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

// Hệ số PD bám line (tăng nhẹ độ nhạy ôm cua cho đường cong/gấp khúc trên tải nặng)
float Kp_line                = 38.0f;
float Kd_line                = 16.0f;

// --- 4. THAM SỐ BÁM LINE TỰ DO, CỨU HỘ ĐỨT ĐOẠN & EKF + CASCADE (GIỮ TẢI NẶNG) ---
const float MAX_BRIDGE_DISTANCE_M       = 1.12f; // Đủ biên độ vượt khoảng hở đầu chặng (tới ~65cm) & quét tìm vạch trước khi cắt an toàn
const unsigned long MAX_BRIDGE_TIME_MS  = 6000;  // 6000ms an toàn khi xe mang tải
const float START_CORNER_GUARD_ARC_M    = 0.20f; // Giữ thẳng đầu chặng (< 20cm) chống nhiễu mép vạch trạm
const float TRACK_CONE_HALF_DEG         = 78.0f; // Nón hướng tiến về đích (±78° giữ chữ < / cong, chặn tuyến vuông/ngược)
const float TRACK_CONE_CURVE_DEG        = 88.0f; // Nón mở rộng tạm thời sau 25cm trên nhánh cong/< gập ~90°
const float CURVE_DETECT_YAW_DEG        = 16.0f; // Vượt ±16° lệch chặng -> nhận diện đường cong / chữ <
const float CURVE_DETECT_CROSS_M        = 0.14f; // Hoặc lệch ngang > 14cm -> tắt neo thẳng EKF, chuyển tự do
const float BRIDGE_CURVE_BLEND_START_M  = 0.03f; // Trên đường cong/<: phản ứng uốn sớm tránh văng xa khỏi đỉnh góc nhọn <
const float BRIDGE_CURVE_BLEND_SPAN_M   = 0.16f; // Quãng uốn gọn ôm cắt lại nhánh quay về / tâm trạm đích
const int   LINE_HIST_LEN               = 12;    // Cửa sổ trượt 120ms (12 khung @ 100Hz) phân tách mất vạch thẳng vs góc gập
int   histMasks[LINE_HIST_LEN]       = {0};
float histErrs[LINE_HIST_LEN]        = {0.0f};
float histGz[LINE_HIST_LEN]          = {0.0f};
int   histHead                       = 0;
int   histCount                      = 0;
bool  bendCatchAttemptedForGap       = false;
uint8_t bendCatchStage               = 0;
int8_t bendCatchDir                  = 1;
float bendCatchStartYawDeg           = 90.0f;
unsigned long bendCatchStartMs       = 0;
float gapStartDistM                  = 0.0f;
float gapStartArcM                   = 0.0f;
float legArcDistM                    = 0.0f;
float lastLineYawDeg                 = 90.0f;
float bridgeOriginX                  = 0.0f;
float bridgeOriginY                  = 0.0f;
float bridgeRefHeadingDeg            = 90.0f;
float finalAimUx                     = 0.0f;
float finalAimUy                     = 1.0f;
bool  stationCrossBarLatched         = false;
bool  curvedExcursionDetected        = false;
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
unsigned long lastWheelMotionMs      = 0;
int8_t lastTurnSpinDir               = 0;
bool  pendingStationSnap             = false;
float pendingSnapX                   = 0.0f;
float pendingSnapY                   = 0.0f;
float pendingSnapYaw                 = 90.0f;
unsigned long lastControlLoopUs      = 0;
int   consecutiveZeroLineFrames      = 0;
int   consecutiveGoodLineFrames      = 0;

// Phục hồi offset Gyro an toàn khi xe đứng yên thật sự
int   stillGyroSamples               = 0;
float stillGyroSum                   = 0.0f;
float stillGyroMin                   = 0.0f;
float stillGyroMax                   = 0.0f;

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
// Góc & quãng đường tham chiếu thuần MPU-6050 + Encoder (không bị 5-IR bẻ sai khi rẽ nhánh < / cong)
float pureMpuYawDeg                  = 90.0f;
float pureMpuLegX                    = 0.0f;
float pureMpuLegY                    = 0.0f;

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
  SUB_CORNER_BEND_CATCH,
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

static inline void clearLineHistoryLocked() {
  histHead = 0;
  histCount = 0;
  for (int i = 0; i < LINE_HIST_LEN; i++) {
    histMasks[i] = 0;
    histErrs[i] = 0.0f;
    histGz[i] = 0.0f;
  }
}

static inline void pushLineHistoryLocked(int mask, float err, float gzDegS) {
  histMasks[histHead] = mask;
  histErrs[histHead] = err;
  histGz[histHead] = gzDegS;
  histHead = (histHead + 1) % LINE_HIST_LEN;
  if (histCount < LINE_HIST_LEN) histCount++;
}

// Phát hiện mẫu góc gập bất đối xứng bên trái (28, 30) hoặc phải (7, 15) ngoài vùng trạm
static inline bool isAsymmetricBendMask(int mask, int &outSideSign) {
  if (mask == 0b00111 || mask == 0b01111) { outSideSign =  1; return true; }
  if (mask == 0b11100 || mask == 0b11110) { outSideSign = -1; return true; }
  outSideSign = 0;
  return false;
}

static inline float computePureMpuCrossM() {
  float refX = currentTargetHasCoord ? currentTargetX : legStartX;
  float refY = currentTargetHasCoord ? currentTargetY : legStartY;
  float legRad = (legTargetHeadingDeg * PI) / 180.0f;
  return (pureMpuLegX - refX) * (-sinf(legRad)) + (pureMpuLegY - refY) * cosf(legRad);
}

static inline void summarizeRecentBendSignalsLocked(bool outsideStationZone,
                                                    bool &outHadBendMask,
                                                    float &outDirErr,
                                                    float &outMeanGzAbs) {
  outHadBendMask = false;
  outDirErr = 0.0f;
  outMeanGzAbs = 0.0f;
  float weightedErrSum = 0.0f;
  float gzSum = 0.0f;
  float weightSum = 0.0f;
  for (int age = 0; age < histCount; age++) {
    int idx = (histHead - 1 - age + LINE_HIST_LEN) % LINE_HIST_LEN;
    int m = histMasks[idx];
    float e = histErrs[idx];
    gzSum += histGz[idx];
    int bits = 0;
    for (int b = 0; b < 5; b++) bits += ((m >> b) & 1);
    int bSign = 0;
    bool asym = outsideStationZone && isAsymmetricBendMask(m, bSign);
    if (outsideStationZone && (asym || bits >= 3 || (m & 0b10001) != 0 || fabsf(e) >= 0.95f)) {
      outHadBendMask = true;
    }
    float sigErr = asym ? ((float)bSign * 1.75f) : e;
    float w = (float)(LINE_HIST_LEN - age);
    weightedErrSum += w * sigErr;
    weightSum += w;
  }
  if (weightSum > 0.0f) {
    outDirErr = weightedErrSum / weightSum;
  }
  if (histCount > 0) {
    outMeanGzAbs = fabsf(gzSum / (float)histCount);
  }
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
  pureMpuYawDeg = yawDeg;
  pureMpuLegX = odomX;
  pureMpuLegY = odomY;

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
  legArcDistM  = 0.0f;
  lastLineYawDeg = yawDeg;
  bridgeOriginX = odomX;
  bridgeOriginY = odomY;
  bridgeRefHeadingDeg = legTargetHeadingDeg;
  finalAimUx = ux;
  finalAimUy = uy;
  stationCrossBarLatched = false;
  curvedExcursionDetected = false;
  bendCatchAttemptedForGap = false;
  bendCatchStage    = 0;
  clearLineHistoryLocked();

  outHeadingErr = normalizeAngleDeg(legTargetHeadingDeg - yawDeg);
  lastLineError     = 0.0f;
  lastValidError    = 0.0f;
  lineLostTimeMs    = 0;
  inGapBridgeMode   = false;
  gapStartArcM      = 0.0f;
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

  // Lấy mẫu đa số 5 lần siêu nhanh (~160us tổng) lọc nhiễu xung & phản chiếu bóng gạch đèn tuýp
  int cL2 = 0, cL1 = 0, cC = 0, cR1 = 0, cR2 = 0;
  for (int k = 0; k < 5; k++) {
    cL2 += (digitalRead(LINE_L2_PIN) == HIGH) ? 1 : 0;
    cL1 += (digitalRead(LINE_L1_PIN) == HIGH) ? 1 : 0;
    cC  += (digitalRead(LINE_C_PIN)  == HIGH) ? 1 : 0;
    cR1 += (digitalRead(LINE_R1_PIN) == HIGH) ? 1 : 0;
    cR2 += (digitalRead(LINE_R2_PIN) == HIGH) ? 1 : 0;
    if (k < 4) delayMicroseconds(40);
  }
  sL2 = (cL2 >= 3) ? 1 : 0;
  sL1 = (cL1 >= 3) ? 1 : 0;
  sC  = (cC  >= 3) ? 1 : 0;
  sR1 = (cR1 >= 3) ? 1 : 0;
  sR2 = (cR2 >= 3) ? 1 : 0;

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
bool readReg16Checked(uint8_t addr, uint8_t reg, int16_t &outVal) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  outVal = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

bool readReg8Checked(uint8_t addr, uint8_t reg, uint8_t &outVal) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  outVal = (uint8_t)Wire.read();
  return true;
}

int16_t readReg16(uint8_t addr, uint8_t reg) {
  int16_t v = 0;
  return readReg16Checked(addr, reg, v) ? v : 0;
}

void writeReg8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void configureMPUHardware() {
  writeReg8(MPU6050_ADDR, 0x6B, 0x00);
  delay(20);
  writeReg8(MPU6050_ADDR, 0x1A, 0x03);
  writeReg8(MPU6050_ADDR, 0x1B, 0x10);
}

void calibrateGyroZ(int samples = 400) {
  if (!mpuConnected) return;
  stopMotors();
  delay(100);

  for (int attempt = 0; attempt < 3; attempt++) {
    double sumDegS = 0.0;
    float minDegS = 1e6f, maxDegS = -1e6f;
    int validCount = 0;
    for (int i = 0; i < samples; i++) {
      int16_t raw = 0;
      if (readReg16Checked(MPU6050_ADDR, 0x47, raw)) {
        float degS = (float)raw / 32.8f;
        sumDegS += degS;
        if (degS < minDegS) minDegS = degS;
        if (degS > maxDegS) maxDegS = degS;
        validCount++;
      }
      delay(2);
    }
    if (validCount >= samples / 2) {
      float avg = (float)(sumDegS / validCount);
      if ((maxDegS - minDegS) <= 12.0f && fabsf(avg) <= 15.0f) {
        gyroZOffset = avg;
        stillGyroSamples = 0;
        Serial.printf("\n>> [GYRO CALIB]: Offset Z = %.3f deg/s (spread=%.2f)\n", gyroZOffset, maxDegS - minDegS);
        return;
      }
      Serial.printf("\n⚠️ [GYRO CALIB THỬ %d]: Xe đang rung/xoay tay (spread=%.1f, avg=%.1f) -> Thử lại...\n",
                    attempt + 1, maxDegS - minDegS, avg);
    }
    delay(250);
  }
  Serial.printf("\n⚠️ [GYRO CALIB]: Giữ nguyên Offset Z an toàn = %.3f deg/s (xe sẽ tự ổn định khi đứng yên)\n", gyroZOffset);
}

void initMPU() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  Wire.setTimeOut(25);

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    mpuConnected = true;
    configureMPUHardware();
    delay(30);
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
  pureMpuYawDeg = yawDeg;
  pureMpuLegX = ekfX;
  pureMpuLegY = ekfY;
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

  // Tích phân song song trục tham chiếu thuần MPU-6050 + Encoder (miễn nhiễm trước giả thiết đường thẳng của 5-IR)
  float pureDeltaDeg = mpuConnected ? ((gzRawDegS - gyroZOffset) * dt) : (dTheta * RAD2DEG_F);
  float pureYawMidRad = wrapRad((pureMpuYawDeg + 0.5f * pureDeltaDeg) * DEG2RAD_F);
  pureMpuYawDeg = normalizeAngleDeg(pureMpuYawDeg + pureDeltaDeg);
  pureMpuLegX += dsEff * cosf(pureYawMidRad);
  pureMpuLegY += dsEff * sinf(pureYawMidRad);

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

// Neo EKF quang học nhẹ 5-IR CHỈ dùng khi đang bám thẳng đúng tuyến (!curvedExcursionDetected)
// Giúp triệt tiêu trôi ngang/lệch góc trên các cạnh thẳng như Test 09 mà không kéo hỏng đoạn cong / chữ <
static inline void ekfUpdateOpticalLineLocked(float lineErrVal, int activeCnt, float legHeadDeg,
                                              float refX, float refY, bool recoveringBridge) {
  if (recoveringBridge || !currentTargetHasCoord || activeCnt < 1 || activeCnt > 2 || fabsf(lineErrVal) > 1.20f) return;

  float legRad = legHeadDeg * DEG2RAD_F;
  float nx = -sinf(legRad);
  float ny =  cosf(legRad);
  float relPsi = wrapRad(ekfYawRad - legRad);
  if (fabsf(relPsi) > (18.0f * DEG2RAD_F)) return;

  float zCross = (lineErrVal * 0.018f) * cosf(relPsi);
  float sx = ekfX + DIST_AXLE_TO_SENSOR_M * cosf(ekfYawRad);
  float sy = ekfY + DIST_AXLE_TO_SENSOR_M * sinf(ekfYawRad);
  float hCross = (sx - refX) * nx + (sy - refY) * ny;
  float yInnov = zCross - hCross;
  if (fabsf(yInnov) > 0.18f) return;

  float H[4] = { nx, ny, DIST_AXLE_TO_SENSOR_M * cosf(relPsi), 0.0f };

  float sig = (activeCnt == 2) ? 0.020f : 0.026f;
  if (fabsf(yInnov) > 0.08f) sig *= 2.0f;
  float R = sig * sig;

  float PHt[4];
  for (int i = 0; i < 4; i++) {
    PHt[i] = ekfP[i][0]*H[0] + ekfP[i][1]*H[1] + ekfP[i][2]*H[2];
  }
  float S = H[0]*PHt[0] + H[1]*PHt[1] + H[2]*PHt[2] + R;
  if (S < 1.0e-8f) return;

  float K[4] = { PHt[0]/S, PHt[1]/S, PHt[2]/S, PHt[3]/S };
  // Không cho 5-IR chỉnh ekfGyroBiasRadS khi đang chạy (ZUPT đứng yên chuẩn xác hơn và không phá góc nhánh <)
  float bgGain = 0.0f;

  ekfX += K[0] * yInnov;
  ekfY += K[1] * yInnov;
  float nextYawDeg = normalizeAngleDeg(wrapRad(ekfYawRad + K[2] * yInnov) * RAD2DEG_F);
  float yawOffsetFromPure = constrain(normalizeAngleDeg(nextYawDeg - pureMpuYawDeg), -2.5f, 2.5f);
  ekfYawRad = wrapRad((pureMpuYawDeg + yawOffsetFromPure) * DEG2RAD_F);

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
  float minWheelV = (fabsf(omegaRefRadS) > 1.25f) ? 0.02f : 0.14f;
  float vLRef = fmaxf(minWheelV, vBaseRef - halfDiffV);
  float vRRef = fmaxf(minWheelV, vBaseRef + halfDiffV);

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

static inline float computeBridgeSensorCrossTrackM() {
  float brRad = bridgeRefHeadingDeg * DEG2RAD_F;
  float yawRad = yawDeg * DEG2RAD_F;
  float sx = odomX + DIST_AXLE_TO_SENSOR_M * cosf(yawRad);
  float sy = odomY + DIST_AXLE_TO_SENSOR_M * sinf(yawRad);
  return (sx - bridgeOriginX) * (-sinf(brRad)) + (sy - bridgeOriginY) * cosf(brRad);
}

static inline float computeDistToTarget2DM(float remainingProjM) {
  return currentTargetHasCoord
           ? hypotf(currentTargetX - odomX, currentTargetY - odomY)
           : fmaxf(0.0f, remainingProjM);
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

// Cầu nối lai tự động thích nghi (Hybrid Test 09 Straight Stanley + Test 10 Curve/Zigzag Goal-Blend):
//   - Trên đường cong / chữ < (curvedExcursionDetected): uốn sớm về nhánh quay lại / tâm trạm đích,
//     mở nón mục tiêu tới ±110° để không văng xa khỏi đỉnh góc nhọn < và không trượt lố qua trạm.
//   - Trên đoạn thẳng (!curvedExcursionDetected): chạy Test 09 Stanley + quét sin đối xứng 2 bên trục.
static inline void computeBridgeCascadeTargetLocked(float curBridgeCrossM, float gapArcM, float remainingToTargetM,
                                                    bool isSideSkidExit, float &outDesiredHeadingDeg,
                                                    int &outTargetBasePwm, bool &outStageChanged) {
  uint8_t prevStage = probeStage;

  if (curvedExcursionDetected && lineTrackedOnLeg) {
    float curAxleCrossM = currentTargetHasCoord ? computeLegCrossTrackM() : 0.0f;
    float relRefFromLeg = normalizeAngleDeg(bridgeRefHeadingDeg - legTargetHeadingDeg);
    bool divergingOutward = currentTargetHasCoord && (fabsf(curAxleCrossM) > 0.11f) &&
                            ((curAxleCrossM * relRefFromLeg) > 0.0f);
    float rawGoalDeg = (currentTargetHasCoord && remainingToTargetM > 0.10f)
                         ? (atan2f(currentTargetY - odomY, currentTargetX - odomX) * RAD2DEG_F)
                         : legTargetHeadingDeg;
    float goalAllowanceDeg = 55.0f + 55.0f * constrain((fabsf(curAxleCrossM) - 0.08f) / 0.18f, 0.0f, 1.0f);
    float relGoalDeg = constrain(normalizeAngleDeg(rawGoalDeg - legTargetHeadingDeg), -goalAllowanceDeg, goalAllowanceDeg);
    float safeGoalDeg = normalizeAngleDeg(legTargetHeadingDeg + relGoalDeg);
    float turnToGoalDeg = fabsf(normalizeAngleDeg(safeGoalDeg - bridgeRefHeadingDeg));

    float blendStartM = (turnToGoalDeg > 35.0f || divergingOutward) ? 0.02f : BRIDGE_CURVE_BLEND_START_M;
    float blendSpanM  = (turnToGoalDeg > 35.0f || divergingOutward) ? 0.14f : BRIDGE_CURVE_BLEND_SPAN_M;

    if (gapArcM >= blendStartM) {
      probeStage = 1;
      float alpha = constrain((gapArcM - blendStartM) / blendSpanM, 0.0f, 1.0f);
      int8_t turnSign = (normalizeAngleDeg(safeGoalDeg - bridgeRefHeadingDeg) < 0.0f) ? -1 : 1;
      float inwardBiasDeg = (turnToGoalDeg > 35.0f && remainingToTargetM > 0.28f && alpha >= 0.60f)
                              ? ((float)turnSign * 8.0f * constrain((alpha - 0.60f) / 0.40f, 0.0f, 1.0f))
                              : 0.0f;
      float blendedHeadingDeg = normalizeAngleDeg(bridgeRefHeadingDeg + alpha * normalizeAngleDeg(safeGoalDeg - bridgeRefHeadingDeg) + inwardBiasDeg);
      float taper = constrain((remainingToTargetM - 0.24f) / 0.10f, 0.0f, 1.0f);
      float sw = fmaxf(0.0f, gapArcM - blendStartM - blendSpanM);
      float sweepDeg = (alpha >= 0.85f && remainingToTargetM > 0.24f)
                         ? ((float)turnSign * taper * 10.0f * sinf((2.0f * PI * sw) / 0.48f))
                         : 0.0f;
      outDesiredHeadingDeg = normalizeAngleDeg(blendedHeadingDeg + sweepDeg);
      outTargetBasePwm = (turnToGoalDeg > 35.0f || remainingToTargetM < 0.40f)
                           ? curvePwm
                           : constrain(bridgePwm - 6, curvePwm, bridgePwm);
      outStageChanged = (probeStage != prevStage);
      return;
    }
  }

  float sweepStartGapM = isSideSkidExit ? 0.14f : (lineTrackedOnLeg ? 0.30f : 0.22f);
  float sweepPeriodM = lineTrackedOnLeg ? 0.56f : 0.72f;
  float dTarget = 0.0f;
  float tangentDeg = 0.0f;

  if (gapArcM < sweepStartGapM || remainingToTargetM <= 0.18f) {
    probeStage = 0;
    outTargetBasePwm = (remainingToTargetM < 0.36f) ? curvePwm : bridgePwm;
  } else {
    probeStage = 1;
    float sw = fmaxf(0.0f, gapArcM - sweepStartGapM);
    float taper = constrain((remainingToTargetM - 0.18f) / 0.08f, 0.0f, 1.0f);
    float maxAmp = lineTrackedOnLeg ? 0.068f : 0.076f;
    float ampM = taper * constrain(0.066f + 0.020f * sw, 0.066f, maxAmp);
    float phase = (2.0f * PI * sw) / sweepPeriodM;
    dTarget = (float)probeSweepSign * ampM * sinf(phase);
    float dSlope = (float)probeSweepSign * ampM * ((2.0f * PI) / sweepPeriodM) * cosf(phase);
    tangentDeg = constrain(atanf(dSlope) * RAD2DEG_F, -28.0f, 28.0f);
    outTargetBasePwm = constrain(bridgePwm - 6, curvePwm, bridgePwm);
  }

  float vEff = fmaxf(0.28f, fabsf(currentSpeed) * 0.01f);
  float stanleyDeg = atan2f(3.0f * (dTarget - curBridgeCrossM), vEff + 0.15f) * RAD2DEG_F;
  float relSteerDeg = constrain(tangentDeg + stanleyDeg, -34.0f, 34.0f);
  outDesiredHeadingDeg = normalizeAngleDeg(bridgeRefHeadingDeg + relSteerDeg);
  outStageChanged = (probeStage != prevStage);
}

static inline float computeLegBridgeLookaheadCrossM() {
  float refX = currentTargetHasCoord ? currentTargetX : legStartX;
  float refY = currentTargetHasCoord ? currentTargetY : legStartY;
  float legRad = legTargetHeadingDeg * DEG2RAD_F;
  float yawRad = yawDeg * DEG2RAD_F;
  // Look-ahead ngắn 9cm giúp thân xe quét sin đối xứng thật qua trục y=0 thay vì lệch 1 phía như dùng mũi 23cm
  float lx = odomX + 0.09f * cosf(yawRad);
  float ly = odomY + 0.09f * sinf(yawRad);
  return (lx - refX) * (-sinf(legRad)) + (ly - refY) * cosf(legRad);
}

static inline void enterBridgeModeLocked(unsigned long nowMs, float remainingToTargetM, bool allowAlignTurn) {
  inGapBridgeMode = true;
  gapStartTimeMs = nowMs;
  gapStartDistM = totalDistM;
  gapStartArcM = legArcDistM;
  bridgeOriginX = odomX;
  bridgeOriginY = odomY;
  probeStage = 0;
  probeStageStartDistM = totalDistM;
  consecutiveGoodLineFrames = 0;

  if (curvedExcursionDetected && lineTrackedOnLeg && legArcDistM >= START_CORNER_GUARD_ARC_M) {
    float relLast = constrain(normalizeAngleDeg(lastLineYawDeg - legTargetHeadingDeg), -TRACK_CONE_HALF_DEG, TRACK_CONE_HALF_DEG);
    bridgeRefHeadingDeg = normalizeAngleDeg(legTargetHeadingDeg + relLast);
  } else if (!curvedExcursionDetected) {
    bridgeRefHeadingDeg = legTargetHeadingDeg;
  } else if (currentTargetHasCoord && remainingToTargetM > 0.25f) {
    float rawGoalDeg = atan2f(currentTargetY - odomY, currentTargetX - odomX) * RAD2DEG_F;
    float relGoalDeg = constrain(normalizeAngleDeg(rawGoalDeg - legTargetHeadingDeg), -65.0f, 65.0f);
    bridgeRefHeadingDeg = normalizeAngleDeg(legTargetHeadingDeg + relGoalDeg);
  } else {
    bridgeRefHeadingDeg = legTargetHeadingDeg;
  }

  if (lineTrackedOnLeg && fabsf(bridgeExitLineErr) >= 0.45f) {
    probeSweepSign = (bridgeExitLineErr < 0.0f) ? 1 : -1;
  } else {
    probeSweepSign = (computeLegCrossTrackM() > 0.015f) ? -1 : 1;
  }

  bool dummyChg = false;
  int dummyBase = bridgePwm;
  bool initSkid = lineTrackedOnLeg && (fabsf(bridgeExitLineErr) >= 1.15f);
  float initCrossM = curvedExcursionDetected ? computeBridgeSensorCrossTrackM() : computeLegBridgeLookaheadCrossM();
  computeBridgeCascadeTargetLocked(initCrossM, 0.0f, remainingToTargetM,
                                   initSkid, gapHeadingHoldDeg, dummyBase, dummyChg);
  lastGapHeadingErr = normalizeAngleDeg(gapHeadingHoldDeg - yawDeg);
  float alignThreshDeg = (!lineTrackedOnLeg || fabsf(currentSpeed) < 12.0f) ? 20.0f : 26.0f;
  bridgeTurnAligning = allowAlignTurn && (fabsf(lastGapHeadingErr) > alignThreshDeg);
  if (smoothedBaseSpeed < (float)curvePwm) {
    smoothedBaseSpeed = (float)curvePwm;
  }
  robotSubState = SUB_LINE_FOLLOWING;
  robotStateStr = "DEAD_RECKONING_BRIDGE";
  isRunning = true;
}

// Khởi tạo trạng thái chạy bám line/bridge ngay sau khi kết thúc xoay tại trạm
static inline void enterLineFollowingFromTurnLocked(bool lineSeen, float initialErr = 0.0f) {
  smoothedBaseSpeed = (float)minMovePwm;
  prevTicksL = globalTicksL;
  prevTicksR = globalTicksR;
  legProgressM = useGlobalLineProgress ? computeLegAlongM() : 0.0f;
  legArcDistM  = 0.0f;
  lastLineYawDeg = yawDeg;
  bridgeOriginX = odomX;
  bridgeOriginY = odomY;
  pureMpuLegX = odomX;
  pureMpuLegY = odomY;
  bridgeRefHeadingDeg = yawDeg;
  stationCrossBarLatched = false;
  curvedExcursionDetected = false;
  bendCatchAttemptedForGap = false;
  bendCatchStage    = 0;
  clearLineHistoryLocked();
  lastLineError = lineSeen ? initialErr : 0.0f;
  lastValidError = lineSeen ? initialErr : 0.0f;
  inGapBridgeMode = false;
  gapStartArcM = 0.0f;
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
  gapStartArcM = 0.0f;
  bridgeTurnAligning = false;
  cornerApproachLatched = false;
  stationCrossBarLatched = false;
  curvedExcursionDetected = false;
  bendCatchAttemptedForGap = false;
  bendCatchStage    = 0;
  clearLineHistoryLocked();
  smoothedBaseSpeed = 0.0f;
  wheelSlipDetected = false;
  stillGyroSamples = 0;
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
  bool gzReadOk = false;
  if (mpuConnected) {
    int16_t rGz = 0;
    gzReadOk = readReg16Checked(MPU6050_ADDR, 0x47, rGz);
    if (gzReadOk) {
      gzRaw = ((float)rGz / 32.8f);
    }
  }

  static int mpuHealthDiv = 0;
  if (mpuConnected && !isRunning && (++mpuHealthDiv >= 100)) {
    mpuHealthDiv = 0;
    uint8_t pwr1 = 0, gCfg = 0;
    if (readReg8Checked(MPU6050_ADDR, 0x6B, pwr1) && readReg8Checked(MPU6050_ADDR, 0x1B, gCfg)) {
      if ((pwr1 & 0x40) != 0 || (gCfg & 0x18) != 0x10) {
        configureMPUHardware();
        gzReadOk = false;
      }
    }
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
  bool needReinitMpu = false, logZuptRecover = false;
  char logLegBuf[28] = {0};
  char logStBuf[12]  = {0};
  float logYaw = 0.0f, logX = 0.0f, logY = 0.0f, logGapVal = 0.0f, recoveredGyroOffset = gyroZOffset;

  portENTER_CRITICAL(&stateMux);

  globalTicksL = curL;
  globalTicksR = curR;

  long dL = curL - prevTicksL;
  long dR = curR - prevTicksR;
  prevTicksL = curL;
  prevTicksR = curR;
  if (dL != 0 || dR != 0) {
    lastWheelMotionMs = nowMs;
  }

  float deltaSL = (float)dL / TICKS_PER_METER;
  float deltaSR = (float)dR / TICKS_PER_METER;
  float deltaS  = (deltaSL + deltaSR) / 2.0f;

  bool isTurningInPlace = (robotSubState == SUB_TURNING_FOR_MISSION ||
                           robotSubState == SUB_TURNING_REVERSE_SEARCH ||
                           robotSubState == SUB_CORNER_BEND_CATCH ||
                           robotSubState == SUB_TURNING_MANUAL ||
                           (robotSubState == SUB_LINE_FOLLOWING && inGapBridgeMode && bridgeTurnAligning) ||
                           (turnBrakeUntilMs > nowMs));
  if (isTurningInPlace) {
    deltaS = 0.0f;
  }

  // Khóa ZUPT cơ khí cứng theo Encoder: 2 bánh đứng yên hoàn toàn (dL=dR=0 >= 40ms) -> Đóng băng YawDeg chống nhảy góc MPU
  bool wheelsPhysicallyStill = (!isRunning || robotSubState == SUB_STOPPED || robotSubState == SUB_PAUSE_AT_STATION) &&
                               (testMotorUntilMs < nowMs) && (stationBrakeUntilMs <= nowMs) && (turnBrakeUntilMs <= nowMs) &&
                               (dL == 0 && dR == 0) && ((nowMs - lastWheelMotionMs) >= 40UL);

  float liveGzDegPerSec = 0.0f;
  if (wheelsPhysicallyStill) {
    ekfGyroBiasRadS *= 0.96f;
    if (gzReadOk) {
      float rawDelta = gzRaw - gyroZOffset;
      if (fabsf(rawDelta) < 3.0f) {
        gyroZOffset = 0.992f * gyroZOffset + 0.008f * gzRaw;
        stillGyroSamples = 0;
      } else {
        if (stillGyroSamples == 0) {
          stillGyroSum = gzRaw;
          stillGyroMin = gzRaw;
          stillGyroMax = gzRaw;
          stillGyroSamples = 1;
        } else {
          stillGyroSum += gzRaw;
          if (gzRaw < stillGyroMin) stillGyroMin = gzRaw;
          if (gzRaw > stillGyroMax) stillGyroMax = gzRaw;
          stillGyroSamples++;
          if (stillGyroSamples >= 35) {
            if ((stillGyroMax - stillGyroMin) <= 6.0f) {
              float avgOffset = stillGyroSum / (float)stillGyroSamples;
              if (fabsf(avgOffset) <= 20.0f) {
                gyroZOffset = avgOffset;
                ekfGyroBiasRadS = 0.0f;
                recoveredGyroOffset = avgOffset;
                logZuptRecover = true;
              } else {
                needReinitMpu = true;
              }
            }
            stillGyroSamples = 0;
          }
        }
      }
    }
  } else {
    stillGyroSamples = 0;
    if (!gzReadOk && mpuConnected) {
      mpuConnected = false;
      ekfPredictLocked(deltaS, deltaSL, deltaSR, 0.0f, isTurningInPlace, realDt);
      mpuConnected = true;
      liveGzDegPerSec = ((deltaSR - deltaSL) / (WHEEL_BASE_W_M * fmaxf(realDt, 0.002f))) * RAD2DEG_F;
    } else {
      ekfPredictLocked(deltaS, deltaSL, deltaSR, gzRaw, isTurningInPlace, realDt);
      float rawDelta = mpuConnected ? (gzRaw - gyroZOffset) : 0.0f;
      liveGzDegPerSec = mpuConnected
                          ? (rawDelta - ekfGyroBiasRadS * RAD2DEG_F)
                          : (((deltaSR - deltaSL) / (WHEEL_BASE_W_M * fmaxf(realDt, 0.002f))) * RAD2DEG_F);
    }
  }

  currentSpeed = (deltaS / realDt) * 100.0f;
  totalDistM  += fabsf(deltaS);
  legArcDistM += fabsf(deltaS);

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
    if (needReinitMpu) configureMPUHardware();
    if (logZuptRecover) Serial.printf("\n>> [ZUPT PHỤC HỒI]: Gyro Offset Z -> %.3f deg/s\n", recoveredGyroOffset);
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
                      (((absErr <= 12.0f) && ((lineMask & 0b01110) != 0)) ||
                       ((absErr <= 8.0f) && (lineMask != 0)));

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

    float stopTurnThresh = reverseSearchDoneForLeg ? 2.0f : 2.5f;
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

  // TRẠNG THÁI 1B: QUÉT TÌM VẠCH ĐỐI XỨNG ±11° (~±6.8cm tại thanh IR) KHI QUAY LỐ / MẤT LINE GÓC TRẠM
  if (robotSubState == SUB_TURNING_REVERSE_SEARCH) {
    float revAbsErr = fabsf(normalizeAngleDeg(legTargetHeadingDeg - yawDeg));
    bool found = (((lineMask & 0b01110) != 0) && revAbsErr <= 20.0f) ||
                 ((lineMask != 0) && revAbsErr <= 14.0f);

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
    unsigned long phaseTimeoutMs = (turnSearchStage <= 1) ? 650UL : 1000UL;

    if (progressAlongSweep >= 11.0f || (nowMs - turnSearchStartMs > phaseTimeoutMs)) {
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

  // TRẠNG THÁI 1B-2: BẮT CUA GÓC GẬP / LINE BIẾN DẠNG (<) BẰNG CUNG QUÉT CÓ DẪN HƯỚNG MPU-6050
  // Chỉ kích hoạt 1 lần mỗi nhịp mất vạch tại đỉnh góc gập (KHÔNG chạy trên đứt vạch thẳng),
  // khóa giới hạn quét an toàn trong nón hướng đích (±88°/±92°) rồi bàn giao lại bám line hoặc Bridge.
  if (robotSubState == SUB_CORNER_BEND_CATCH) {
    float curRelFromLeg = normalizeAngleDeg(yawDeg - legTargetHeadingDeg);
    float catchConeDeg = (legArcDistM >= 0.25f) ? TRACK_CONE_CURVE_DEG : TRACK_CONE_HALF_DEG;
    if (activeCount > 0 && fabsf(curRelFromLeg) <= catchConeDeg) {
      int asymSign = 0;
      bool outsideStationZone = currentTargetHasCoord
                                  ? (hypotf(currentTargetX - odomX, currentTargetY - odomY) > 0.35f)
                                  : true;
      float initErr = (outsideStationZone && isAsymmetricBendMask(maskSnap, asymSign))
                        ? ((float)asymSign * 1.75f)
                        : liveMaskErr;
      inGapBridgeMode = false;
      consecutiveZeroLineFrames = 0;
      consecutiveGoodLineFrames = 1;
      bendCatchStage = 0;
      lastLineYawDeg = yawDeg;
      currentLineErr = initErr;
      lastLineError = initErr;
      lastValidError = initErr;
      pushLineHistoryLocked(maskSnap, initErr, liveGzDegPerSec);
      turnBrakeUntilMs = nowMs + 25;
      robotSubState = SUB_LINE_FOLLOWING;
      robotStateStr = "TURNING_LINE";
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }

    int8_t activeDir = (bendCatchStage == 0) ? bendCatchDir : -bendCatchDir;
    if (activeDir == 0) activeDir = 1;
    float sweptDeg = (float)activeDir * normalizeAngleDeg(yawDeg - bendCatchStartYawDeg);
    float sweepLimitDeg = (bendCatchStage == 0) ? 78.0f : 35.0f;
    unsigned long phaseMs = (bendCatchStage == 0) ? 950UL : 1100UL;
    bool outOfDestCone = (fabsf(curRelFromLeg) > 92.0f) && (((float)activeDir * curRelFromLeg) > 0.0f);

    if (sweptDeg >= sweepLimitDeg || outOfDestCone || (nowMs - bendCatchStartMs > phaseMs)) {
      if (bendCatchStage == 0 && !outOfDestCone) {
        bendCatchStage = 1;
        turnBrakeUntilMs = nowMs + 30;
        bendCatchStartMs = turnBrakeUntilMs;
      } else {
        bendCatchStage = 0;
        bendCatchAttemptedForGap = true;
        float remTargetM = currentTargetHasCoord
                             ? hypotf(currentTargetX - odomX, currentTargetY - odomY)
                             : fmaxf(0.0f, legTargetDistM - legProgressM);
        enterBridgeModeLocked(nowMs, remTargetM, true);
        turnBrakeUntilMs = nowMs + 20;
      }
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }

    int bendPwm = computeRegulatedTurnPwm(0.0f, true, fabsf(liveGzDegPerSec));
    if (bendPwm < 0) {
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }
    outLeftPwm  = -activeDir * bendPwm;
    outRightPwm =  activeDir * bendPwm;
    robotStateStr = "TURNING_LINE";
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
    if (needReinitMpu) configureMPUHardware();
    if (logZuptRecover) Serial.printf("\n>> [ZUPT PHỤC HỒI]: Gyro Offset Z -> %.3f deg/s\n", recoveredGyroOffset);
    return;
  }

  // TRẠNG THÁI 3: THEO DÕI TIẾN ĐỘ CHẶNG & TIẾP CẬN VÙNG TRẠM 2D (KẾT HỢP TỌA ĐỘ + VẠCH NGANG)
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
  float distToTarget2D = computeDistToTarget2DM(remainingLegM);
  float sensorReachCornerDistM = legTargetDistM - DIST_AXLE_TO_SENSOR_M;
  bool pastMinArc = (legArcDistM >= 0.65f * legTargetDistM);
  bool stationCrossBarSeen = (activeCount >= 3) || ((lineMask & 0b01110) == 0b01110);
  bool straightCorridorNearEnd = useGlobalLineProgress &&
                                 (legProgressM >= sensorReachCornerDistM) &&
                                 (fabsf(computeLegCrossTrackM()) <= 0.14f) &&
                                 (distToTarget2D <= 0.34f);

  if (!cornerApproachLatched && pastMinArc) {
    bool reachByZoneAndCrossBar     = currentTargetHasCoord && (distToTarget2D <= 0.30f) && stationCrossBarSeen;
    bool reachByAxleRadius          = currentTargetHasCoord && (distToTarget2D <= (DIST_AXLE_TO_SENSOR_M + 0.02f));
    bool reachByCornerBarAlongPlane = currentTargetHasCoord && (legArcDistM >= 0.72f * legTargetDistM) &&
                                      stationCrossBarSeen && (remainingLegM <= 0.24f) &&
                                      (remainingLegM >= -0.20f) && (distToTarget2D <= 0.30f);
    bool reachByForwardPlane        = (legArcDistM >= 0.85f * legTargetDistM) &&
                                      (remainingLegM <= (DIST_AXLE_TO_SENSOR_M + 0.02f)) &&
                                      (!currentTargetHasCoord || (distToTarget2D <= 0.24f && fabsf(computeLegCrossTrackM()) <= 0.18f));
    bool reachBy1DFallback          = (!currentTargetHasCoord && legProgressM >= sensorReachCornerDistM);
    if (reachByZoneAndCrossBar || reachByAxleRadius || reachByCornerBarAlongPlane ||
        reachByForwardPlane || straightCorridorNearEnd || reachBy1DFallback) {
      cornerApproachLatched = true;
      stationCrossBarLatched = stationCrossBarSeen;
      inGapBridgeMode = false;
      bridgeTurnAligning = false;
      if (currentTargetHasCoord && distToTarget2D <= 0.36f && distToTarget2D > 0.04f) {
        finalAimUx = (currentTargetX - odomX) / distToTarget2D;
        finalAimUy = (currentTargetY - odomY) / distToTarget2D;
      } else {
        float legRad = legTargetHeadingDeg * DEG2RAD_F;
        finalAimUx = cosf(legRad);
        finalAimUy = sinf(legRad);
      }
    }
  }

  int leftPwm  = 0;
  int rightPwm = 0;

  if (cornerApproachLatched) {
    bridgeTurnAligning = false;
    float absSpd = fabsf(currentSpeed);
    float stopLeadM = (absSpd > 22.0f) ? 0.024f : ((absSpd > 14.0f) ? 0.015f : 0.006f);
    float remAlongFinal = currentTargetHasCoord
                            ? ((currentTargetX - odomX) * finalAimUx + (currentTargetY - odomY) * finalAimUy)
                            : remainingLegM;

    if (remAlongFinal <= stopLeadM) {
      float yawErrCorner = fabsf(normalizeAngleDeg(legTargetHeadingDeg - yawDeg));
      bool crossBarWasSeen = stationCrossBarLatched;
      bool allowStationSnap = (distToTarget2D <= 0.25f) && (crossBarWasSeen || lineTrackedOnLeg);

      smoothedBaseSpeed = 0.0f;
      cornerApproachLatched = false;
      stationCrossBarLatched = false;
      curvedExcursionDetected = false;
      bendCatchAttemptedForGap = false;
      bendCatchStage = 0;
      clearLineHistoryLocked();
      inGapBridgeMode = false;
      gapStartArcM = 0.0f;
      hasSeenLineOnLeg = false;
      lineTrackedOnLeg = false;
      lastConfirmedLineProgM = -1.0f;
      bridgeExitLineErr = 0.0f;
      consecutiveGoodLineFrames = 0;
      resetWheelCascadeLocked();

      snprintf(currentStationName, sizeof(currentStationName), "%s", currentTargetName);
      currentStation = parseStationId(currentStationName);
      if (allowStationSnap) {
        float snapX = odomX;
        float snapY = odomY;
        if (currentTargetHasCoord) {
          snapX = currentTargetX;
          snapY = currentTargetY;
        } else if (currentStation < STATION_UNKNOWN) {
          snapX = STATION_COORDS[currentStation].x;
          snapY = STATION_COORDS[currentStation].y;
        }
        // Bảo toàn góc thật của Gyro (chỉ tinh chỉnh nếu sai số <= 2.5°) tránh bơm sai số hướng cho chặng sau
        float snapYaw = (yawErrCorner <= 2.5f) ? legTargetHeadingDeg : yawDeg;
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
      Serial.printf("\n🏁 [ĐÃ TỚI ĐÍCH CHẶNG: %s] Xe đang ở TRẠM: %s | Tọa độ: X=%.3fm, Y=%.3fm, Yaw=%.1f°%s\n",
                    logLegBuf, logStBuf, logX, logY, logYaw,
                    allowStationSnap ? " [Khớp Tọa Độ Trạm]" : " [Giữ Odometry Thực]");
      return;
    }

    // Hồ sơ giảm tốc căn bậc hai + điều chế phanh ngắn tránh trượt lố 6cm qua trạm
    float targetCornerSpdCmS = constrain(11.0f + 35.0f * sqrtf(fmaxf(remAlongFinal, 0.0f)), 13.0f, 32.0f);
    if (absSpd > targetCornerSpdCmS && remAlongFinal > 0.025f) {
      robotStateStr = "APPROACH_CORNER";
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      return;
    }

    int approachBase = (remAlongFinal <= 0.16f && absSpd >= 5.0f) ? (minMovePwm + 8) : brakePwm;
    float baseAimHeadingDeg = atan2f(finalAimUy, finalAimUx) * RAD2DEG_F;
    if (activeCount >= 3) {
      stationCrossBarLatched = true;
    }

    bool followOpticalIntoStation = (activeCount >= 1 && activeCount <= 2) &&
                                    !stationCrossBarLatched &&
                                    (fabsf(liveMaskErr) <= 1.5f) &&
                                    (fabsf(normalizeAngleDeg(baseAimHeadingDeg - yawDeg)) <= 45.0f);
    if (followOpticalIntoStation) {
      if (consecutiveZeroLineFrames > 0) {
        lastLineError = liveMaskErr;
      }
      consecutiveZeroLineFrames = 0;
      lineTrackedOnLeg = true;
      lastLineYawDeg = yawDeg;
      currentLineErr = liveMaskErr;
      float dErrApp = (liveMaskErr - lastLineError) / fmaxf(realDt, 0.001f);
      lastLineError = liveMaskErr;
      lastValidError = liveMaskErr;
      float uApp = (Kp_line * liveMaskErr) + (Kd_line * dErrApp);
      float slope = (float)max(straightPwm - minMovePwm, 10) / 0.26f;
      float omegaRefRadS = constrain(-(2.0f * uApp) / (slope * WHEEL_BASE_W_M), -1.8f, 1.8f);
      computeLoadedCascadePwm((float)max(approachBase, minMovePwm), omegaRefRadS, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
    } else {
      float aimClampDeg = (curvedExcursionDetected && currentTargetHasCoord && fabsf(computeLegCrossTrackM()) > 0.07f) ? 68.0f : 12.0f;
      float relAimFromLeg = constrain(normalizeAngleDeg(baseAimHeadingDeg - legTargetHeadingDeg), -aimClampDeg, aimClampDeg);
      float cornerHeading = normalizeAngleDeg(legTargetHeadingDeg + relAimFromLeg);
      if (remAlongFinal <= 0.14f && (!curvedExcursionDetected || !currentTargetHasCoord || fabsf(computeLegCrossTrackM()) <= 0.07f)) {
        cornerHeading = legTargetHeadingDeg;
      } else if (currentTargetHasCoord && distToTarget2D > 0.10f) {
        float rawCornerDeg = atan2f(currentTargetY - odomY, currentTargetX - odomX) * RAD2DEG_F;
        float relRaw = constrain(normalizeAngleDeg(rawCornerDeg - legTargetHeadingDeg), -aimClampDeg, aimClampDeg);
        cornerHeading = normalizeAngleDeg(legTargetHeadingDeg + relRaw);
      }
      computeCornerApproachPwm(cornerHeading, yawDeg, approachBase, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
    }
    robotStateStr = "APPROACH_CORNER";
    portEXIT_CRITICAL(&stateMux);
    setMotors(leftPwm, rightPwm);
    return;
  }

  // GIAI ĐOẠN 3.2: BÁM LINE TỰ DO + PHÂN NHÁNH CỨU HỘ MẤT LINE THEO NGỮ CẢNH (QUÉT CUA ±100° VS CẦU ĐỨT ĐOẠN)
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

    if (lineTrackedOnLeg && consecutiveZeroLineFrames <= 3 && !inGapBridgeMode) {
      float flickerBase = (curvedExcursionDetected || fabsf(lastValidError) >= 1.15f)
                            ? fminf(smoothedBaseSpeed, (float)curvePwm)
                            : smoothedBaseSpeed;
      float steerGain = curvedExcursionDetected ? 0.80f : 0.55f;
      int u = constrain((int)(Kp_line * lastValidError * steerGain), -58, 58);
      int steerMin = max(148, minMovePwm - 38);
      leftPwm  = constrain((int)(flickerBase + u), steerMin, maxPwm);
      rightPwm = constrain((int)(flickerBase - u), steerMin, maxPwm);
      if (max(leftPwm, rightPwm) < minMovePwm) {
        int boost = minMovePwm - max(leftPwm, rightPwm);
        leftPwm  = constrain(leftPwm + boost, steerMin, maxPwm);
        rightPwm = constrain(rightPwm + boost, steerMin, maxPwm);
      }
      robotStateStr = "LINE_FLICKER_FILTER";
      portEXIT_CRITICAL(&stateMux);
      setMotors(leftPwm, rightPwm);
      return;
    }

    if (!inGapBridgeMode) {
      // Phân tách bằng lịch sử 120ms + MPU-6050: Đứt vạch thẳng -> Bridge; Đỉnh góc gập/< -> SUB_CORNER_BEND_CATCH
      bool recentHadBendMask = false;
      float recentDirErr = 0.0f;
      float recentMeanGzAbs = 0.0f;
      bool outsideStationZone = !currentTargetHasCoord || (distToTarget2D > 0.35f);
      summarizeRecentBendSignalsLocked(outsideStationZone, recentHadBendMask, recentDirErr, recentMeanGzAbs);
      float pureYawErrLeg = fabsf(normalizeAngleDeg(pureMpuYawDeg - legTargetHeadingDeg));
      bool isBendLoss = lineTrackedOnLeg && !bendCatchAttemptedForGap && !stationCrossBarLatched &&
                        (distToTarget2D > 0.28f) &&
                        (curvedExcursionDetected || pureYawErrLeg > 14.0f || recentMeanGzAbs >= 26.0f ||
                         (recentHadBendMask && fabsf(recentDirErr) >= 1.15f));

      if (isBendLoss) {
        bendCatchAttemptedForGap = true;
        if (fabsf(normalizeAngleDeg(yawDeg - pureMpuYawDeg)) > 2.5f) {
          ekfSyncFromPoseLocked(pureMpuLegX, pureMpuLegY, pureMpuYawDeg, true, 0.015f, 1.5f);
        }
        curvedExcursionDetected = true;
        if (fabsf(recentDirErr) >= 0.70f) {
          bendCatchDir = (recentDirErr > 0.0f) ? -1 : 1;
        } else if (fabsf(lastValidError) >= 0.65f) {
          bendCatchDir = (lastValidError > 0.0f) ? -1 : 1;
        } else if (currentTargetHasCoord && distToTarget2D > 0.20f) {
          float rawGoalD = atan2f(currentTargetY - odomY, currentTargetX - odomX) * RAD2DEG_F;
          bendCatchDir = (normalizeAngleDeg(rawGoalD - yawDeg) >= 0.0f) ? 1 : -1;
        } else {
          bendCatchDir = (normalizeAngleDeg(legTargetHeadingDeg - yawDeg) >= 0.0f) ? 1 : -1;
        }
        smoothedBaseSpeed = 0.0f;
        bendCatchStage = 0;
        bendCatchStartYawDeg = yawDeg;
        turnBrakeUntilMs = nowMs + 25;
        bendCatchStartMs = turnBrakeUntilMs;
        robotSubState = SUB_CORNER_BEND_CATCH;
        robotStateStr = "TURNING_LINE";
        portEXIT_CRITICAL(&stateMux);
        brakeMotorsHard();
        return;
      }

      float exitErrSnap = lineTrackedOnLeg ? lastValidError : 0.0f;
      bridgeExitLineErr = exitErrSnap;
      lastValidError = 0.0f;

      enterBridgeModeLocked(nowMs, distToTarget2D, true);
      logGapStart = true;
      snprintf(logLegBuf, sizeof(logLegBuf), "%s", currentLegName);
      logYaw = gapHeadingHoldDeg;
    }

    float gapTraveled = totalDistM - gapStartDistM;
    float gapArc = fmaxf(0.0f, legArcDistM - gapStartArcM);
    bool isSideSkidExit = lineTrackedOnLeg && (fabsf(bridgeExitLineErr) >= 1.15f);
    float maxGapDist = isSideSkidExit ? 0.60f : MAX_BRIDGE_DISTANCE_M;
    unsigned long maxGapMs = isSideSkidExit ? 3400UL : MAX_BRIDGE_TIME_MS;
    bool straightBridgeDrifted = !curvedExcursionDetected && currentTargetHasCoord &&
                                 ((fabsf(computeLegSensorCrossTrackM()) > 0.32f) || (fabsf(computeLegCrossTrackM()) > 0.28f)) &&
                                 (gapArc >= 0.35f);

    if (gapArc > maxGapDist || gapTraveled > (maxGapDist + 0.25f) ||
        (nowMs - gapStartTimeMs > maxGapMs) || straightBridgeDrifted) {
      bridgeTurnAligning = false;
      bool canEnterCornerSafely = currentTargetHasCoord && (distToTarget2D <= 0.28f) &&
                                  !straightBridgeDrifted &&
                                  (lineTrackedOnLeg || !hasSeenLineOnLeg) &&
                                  (!curvedExcursionDetected || distToTarget2D <= 0.25f);
      if (canEnterCornerSafely) {
        inGapBridgeMode = false;
        consecutiveZeroLineFrames = 0;
        cornerApproachLatched = true;
        stationCrossBarLatched = false;
        if (curvedExcursionDetected && distToTarget2D > 0.04f) {
          finalAimUx = (currentTargetX - odomX) / distToTarget2D;
          finalAimUy = (currentTargetY - odomY) / distToTarget2D;
        } else {
          float legRad = legTargetHeadingDeg * DEG2RAD_F;
          finalAimUx = cosf(legRad);
          finalAimUy = sinf(legRad);
        }
        float appDeg = atan2f(finalAimUy, finalAimUx) * RAD2DEG_F;
        computeCornerApproachPwm(appDeg, yawDeg, brakePwm, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
        robotStateStr = "APPROACH_CORNER";
        portEXIT_CRITICAL(&stateMux);
        setMotors(leftPwm, rightPwm);
        return;
      }
      enterGapTimeoutStopLocked(nowMs);
      logGapTimeout = true;
      logGapVal = fmaxf(gapArc, gapTraveled);
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      Serial.printf("\n❌ [DỪNG AN TOÀN] Mất vạch %.2fm (cách đích=%.2fm)!\n", logGapVal, distToTarget2D);
      return;
    }

    float curBridgeCross = curvedExcursionDetected ? computeBridgeSensorCrossTrackM() : computeLegBridgeLookaheadCrossM();
    bool probeStageChanged = false;
    float desiredBridgeHeadingDeg = bridgeRefHeadingDeg;
    int targetBase = bridgePwm;
    computeBridgeCascadeTargetLocked(curBridgeCross, gapArc, distToTarget2D, isSideSkidExit,
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
    float omegaMaxBridge = (curvedExcursionDetected && lineTrackedOnLeg) ? 1.85f : 1.35f;
    float omegaGainBridge = (curvedExcursionDetected && lineTrackedOnLeg) ? 3.6f : 3.0f;
    float omegaRefRadS = constrain(omegaGainBridge * (hErr * DEG2RAD_F), -omegaMaxBridge, omegaMaxBridge);

    if (targetBase > smoothedBaseSpeed) smoothedBaseSpeed = min((float)targetBase, smoothedBaseSpeed + MAX_RAMP_STEP);
    else if (targetBase < smoothedBaseSpeed) smoothedBaseSpeed = max((float)targetBase, smoothedBaseSpeed - MAX_RAMP_STEP);

    computeLoadedCascadePwm(smoothedBaseSpeed, omegaRefRadS, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
    robotStateStr = "DEAD_RECKONING_BRIDGE";
    portEXIT_CRITICAL(&stateMux);
    setMotors(leftPwm, rightPwm);
    if (logGapStart) {
      Serial.printf("\n⚡ [STANLEY+EKF BRIDGE] Dẫn đường tiếp tuyến %s | Hướng mục tiêu = %.1f°\n",
                    logLegBuf, logYaw);
    }
    return;

  } else {
    float relYawFromLeg = normalizeAngleDeg(yawDeg - legTargetHeadingDeg);
    float effectiveTrackCone = (curvedExcursionDetected && legArcDistM >= 0.25f)
                                 ? TRACK_CONE_CURVE_DEG
                                 : TRACK_CONE_HALF_DEG;
    if (fabsf(relYawFromLeg) > effectiveTrackCone) {
      // Nón hướng đích (±78°, mở ±88° trên khúc gập <): Bỏ qua tuyến vuông góc/ngược và lái về hướng đích
      consecutiveGoodLineFrames = 0;
      consecutiveZeroLineFrames = 0;
      inGapBridgeMode = false;
      computeCornerApproachPwm(legTargetHeadingDeg, yawDeg, curvePwm, liveGzDegPerSec, deltaSL, deltaSR, realDt, leftPwm, rightPwm);
      robotStateStr = "TURNING_LINE";
      portEXIT_CRITICAL(&stateMux);
      setMotors(leftPwm, rightPwm);
      return;
    }

    hasSeenLineOnLeg = true;
    bridgeTurnAligning = false;
    probeStage = 0;
    consecutiveZeroLineFrames = 0;

    // Hàng rào An toàn Quỹ đạo Rộng: Giữ tự do cho chữ Z / < / đường cong (~0.45m), nhưng cắt an toàn
    // nếu xe bám nhầm sang tuyến đường khác (> 0.85m hoặc cung chạy gấp > 2.2x chiều dài chặng + 0.5m).
    float maxCorridorM = currentTargetHasCoord
                           ? fmaxf(0.85f, 0.55f * fminf(activeMapWidthM, activeMapLengthM))
                           : 1.50f;
    float maxArcOnLegM = fmaxf(legTargetDistM * 2.2f + 0.50f, 3.00f);
    float curCrossTrackM = computeLegCrossTrackM();
    bool offMapCorridor = currentTargetHasCoord && (fabsf(curCrossTrackM) > maxCorridorM);
    bool excessiveArcOnLeg = (legArcDistM > maxArcOnLegM);
    if (offMapCorridor || excessiveArcOnLeg) {
      enterGapTimeoutStopLocked(nowMs);
      portEXIT_CRITICAL(&stateMux);
      brakeMotorsHard();
      Serial.printf("\n❌ [LẠC TUYẾN AN TOÀN %s]: cross=%.2fm (max=%.2f), arc=%.2fm (max=%.2f) -> Dừng!\n",
                    currentLegName, curCrossTrackM, maxCorridorM, legArcDistM, maxArcOnLegM);
      return;
    }

    bool recoveringFromBridge = inGapBridgeMode;
    if (inGapBridgeMode) {
      logGapVal = totalDistM - gapStartDistM;
      inGapBridgeMode = false;
      logGapRecovered = true;
      snprintf(logLegBuf, sizeof(logLegBuf), "%s", currentLegName);
    }

    lineLostTimeMs = 0;

    int asymMaskSign = 0;
    bool outsideStationForBend = !currentTargetHasCoord || (distToTarget2D > 0.35f);
    bool isBendMask = outsideStationForBend && isAsymmetricBendMask(maskSnap, asymMaskSign);
    float effectiveMaskErr = isBendMask ? ((float)asymMaskSign * 1.75f) : liveMaskErr;

    currentLineErr = effectiveMaskErr;
    lastValidError = currentLineErr;
    if (recoveringFromBridge) {
      // Triệt tiêu xung dErr giả ngay khung hình đầu tiên vừa bắt lại vạch
      lastLineError = currentLineErr;
    }
    pushLineHistoryLocked(maskSnap, currentLineErr, liveGzDegPerSec);

    // Bám Line Tự Do trong Nón Đích: Chỉ ghi nhận bám line vững (lineTrackedOnLeg) khi 4 khung hình
    // liên tiếp giữ gần tâm vạch (chống chạm mép rìa 1 khung lúc mới xuất phát rồi quay đầu).
    if (fabsf(currentLineErr) <= 1.0f && !isBendMask) {
      if (consecutiveGoodLineFrames < 50) consecutiveGoodLineFrames++;
      if (activeCount <= 3) {
        lastLineYawDeg = yawDeg;
      }
    } else {
      consecutiveGoodLineFrames = 0;
    }
    if (consecutiveGoodLineFrames >= 4) {
      lineTrackedOnLeg = true;
      bendCatchAttemptedForGap = false;
      lastConfirmedLineProgM = legProgressM;
    }

    // Phân biệt tự động Tuyến Thẳng vs Tuyến Cong / Chữ < dùng CẢ EKF & trục thuần MPU-6050:
    //   - Nếu pureMpuYawDeg rẽ chéo hoặc lệch > 14° khi đang bám line -> ngắt kéo góc EKF và trả về MPU thật!
    //   - Chỉ neo EKF quang học trên tuyến thẳng đã kiểm chứng nhất quán với MPU (<= 7° và |Gz| <= 12°/s).
    float yawErrToLeg = fabsf(relYawFromLeg);
    float pureRelLeg = normalizeAngleDeg(pureMpuYawDeg - legTargetHeadingDeg);
    float pureYawErr = fabsf(pureRelLeg);
    float curCrossAbs = currentTargetHasCoord ? fabsf(curCrossTrackM) : 0.0f;
    float pureCrossM = currentTargetHasCoord ? computePureMpuCrossM() : 0.0f;
    float pureCrossAbs = fabsf(pureCrossM);
    bool pureDiagonalDiverging = (pureYawErr > 14.0f && pureCrossAbs > 0.08f && (pureCrossM * pureRelLeg) > 0.0f);
    bool pureSustainedTurn = (consecutiveGoodLineFrames >= 8 && pureYawErr > 15.0f && !recoveringFromBridge);
    bool sustainedCurveHeading = (consecutiveGoodLineFrames >= 12 && yawErrToLeg > (CURVE_DETECT_YAW_DEG + 2.0f) && !recoveringFromBridge);
    bool divergingDiagonalLine = (yawErrToLeg > (CURVE_DETECT_YAW_DEG + 4.0f) && curCrossAbs > 0.12f && (curCrossTrackM * relYawFromLeg) > 0.0f);
    if (lineTrackedOnLeg && legArcDistM >= 0.14f &&
        (curCrossAbs > (CURVE_DETECT_CROSS_M + 0.02f) || pureCrossAbs > 0.14f ||
         pureDiagonalDiverging || pureSustainedTurn || divergingDiagonalLine || sustainedCurveHeading)) {
      if (!curvedExcursionDetected) {
        curvedExcursionDetected = true;
        if (fabsf(normalizeAngleDeg(yawDeg - pureMpuYawDeg)) > 2.5f) {
          ekfSyncFromPoseLocked(pureMpuLegX, pureMpuLegY, pureMpuYawDeg, true, 0.015f, 1.5f);
        }
      }
    }

    if (!curvedExcursionDetected && !recoveringFromBridge &&
        consecutiveGoodLineFrames >= 12 && pureYawErr <= 7.0f &&
        fabsf(liveGzDegPerSec) <= 12.0f && pureCrossAbs <= 0.10f &&
        yawErrToLeg <= 10.0f && curCrossAbs <= 0.10f &&
        activeCount >= 1 && activeCount <= 2 && fabsf(currentLineErr) <= 1.0f) {
      float refX = currentTargetHasCoord ? currentTargetX : legStartX;
      float refY = currentTargetHasCoord ? currentTargetY : legStartY;
      ekfUpdateOpticalLineLocked(currentLineErr, activeCount, legTargetHeadingDeg, refX, refY, recoveringFromBridge);
    }

    float dErr = (currentLineErr - lastLineError) / fmaxf(realDt, 0.001f);
    lastLineError = currentLineErr;

    if (fabsf(currentLineErr) < 0.6f) {
      bridgeExitLineErr = 0.0f;
    }

    float absErr = fabsf(currentLineErr);
    bool inCurveBranch = curvedExcursionDetected || (yawErrToLeg > CURVE_DETECT_YAW_DEG) ||
                         (pureYawErr > 14.0f) || (fabsf(liveGzDegPerSec) > 26.0f);
    int baseUpperCap = inCurveBranch
                         ? ((absErr >= 1.10f || fabsf(liveGzDegPerSec) > 30.0f) ? curvePwm : min(curvePwm + 6, brakePwm))
                         : straightPwm;
    int targetBase = straightPwm - (int)(absErr * (straightPwm - curvePwm));
    targetBase = constrain(targetBase, curvePwm, baseUpperCap);

    if (distToTarget2D < 0.40f && pastMinArc) {
      targetBase = min(targetBase, brakePwm);
      robotStateStr = "BRAKING_NEAR_STATION";
      if (distToTarget2D < 0.36f && fabsf(currentSpeed) > 36.0f) {
        portEXIT_CRITICAL(&stateMux);
        brakeMotorsHard();
        return;
      }
    } else {
      robotStateStr = (absErr < 0.35f && !isBendMask) ? "RUNNING_FAST" : "TURNING_LINE";
    }

    if (targetBase > smoothedBaseSpeed) smoothedBaseSpeed = min((float)targetBase, smoothedBaseSpeed + MAX_RAMP_STEP);
    else if (targetBase < smoothedBaseSpeed) smoothedBaseSpeed = max((float)targetBase, smoothedBaseSpeed - MAX_RAMP_STEP);

    float u = (Kp_line * currentLineErr) + (Kd_line * dErr);
    float pivotThresh = (inCurveBranch || isBendMask) ? 1.40f : 1.65f;

    if (currentLineErr <= -pivotThresh) {
      leftPwm  = -turnPwm;
      rightPwm = curvePwm + 30;
      robotStateStr = "PIVOT_LEFT";
    } else if (currentLineErr >= pivotThresh) {
      leftPwm  = curvePwm + 30;
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
      Serial.printf("🎯 [mDNS THÀNH CÔNG - Lần %d]: Server IP = %s -> URL: %s\n", attempt, mdnsIp.toString().c_str(), serverUrl.c_str());
      return true;
    }
    delay(150);
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

// Tính chữ ký số HMAC-SHA256 (128-bit / 32 ký tự hex) bằng phần cứng mbedTLS trên ESP32-S3
String computeHmacSha256Hex32(const String &message) {
  uint8_t hmacResult[32];
  const mbedtls_md_info_t *mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!mdInfo) return "";
  int rc = mbedtls_md_hmac(
      mdInfo,
      (const unsigned char *)agv_hmac_secret,
      strlen(agv_hmac_secret),
      (const unsigned char *)message.c_str(),
      message.length(),
      hmacResult);
  if (rc != 0) return "";
  char hexBuf[33];
  for (int i = 0; i < 16; i++) {
    snprintf(&hexBuf[i * 2], 3, "%02x", hmacResult[i]);
  }
  hexBuf[32] = '\0';
  return String(hexBuf);
}

// So sánh chuỗi thời gian hằng (Constant-Time) chống tấn công Timing Side-Channel
bool constantTimeEquals(const String &a, const String &b) {
  if (a.length() != b.length() || a.length() == 0) return false;
  uint8_t diff = 0;
  for (unsigned int i = 0; i < a.length(); i++) {
    diff |= (uint8_t)(tolower(a.charAt(i)) ^ tolower(b.charAt(i)));
  }
  return diff == 0;
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

    // Phân giải tên miền mDNS chuẩn agv-wms.local
    if (MDNS.begin("agv-esp32")) {
      delay(200);
      resolveServerMdns(3, 1000);
    }
  } else {
    Serial.println("\n⚠️ [WI-FI]: Không kết nối được Wi-Fi -> Chạy nội bộ.");
  }

  HTTPClient http;
  http.setTimeout(1500);
  http.setReuse(true); // Giữ kết nối TCP Keep-Alive tránh cạn kiệt socket lwIP
  unsigned long lastMdnsRetryMs = 0;
  unsigned long lastHttpErrMs = 0;
  unsigned long lastWifiRetryMs = 0;
  int consecutiveHttpErrors = 0;

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
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

      char payloadBuf[384];
      snprintf(
          payloadBuf, sizeof(payloadBuf),
          "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.1f,\"dist\":%.3f,\"speed\":%.1f,"
          "\"line_mask\":%d,\"line_err\":%.2f,\"ticks_l\":%ld,\"ticks_r\":%ld,"
          "\"pwm_l\":%d,\"pwm_r\":%d,\"station\":\"%s\",\"leg\":\"%s\","
          "\"ack_id\":%d,\"auto_loop\":%s,\"voltage\":%.3f,\"current\":%.3f,"
          "\"power\":%.3f,\"energy_mah\":%.1f,\"ina_ok\":%s,\"state\":\"%s\"}",
          snapX, snapY, snapYaw, snapDist, snapSpeed,
          snapMask, snapLineErr, snapTicksL, snapTicksR,
          snapPwmL, snapPwmR, snapStation, snapLeg,
          lastAckId, snapAutoLoop ? "true" : "false", liveBusV, liveCurrentA,
          livePowerW, liveEnergyMah, inaDetected ? "true" : "false", snapStatePtr);
      String payload = String(payloadBuf);

      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");
      http.addHeader("X-Device-Token", agv_device_token); // Xác thực bảo mật RBAC với Server WMS
      http.addHeader("X-Payload-Signature", computeHmacSha256Hex32(payload)); // Chữ ký toàn vẹn gói tin HMAC-SHA256
      int httpCode = http.POST(payload);

      if (httpCode == 200) {
        consecutiveHttpErrors = 0;
        String resp = http.getString();

        // Đồng bộ kích thước Bản đồ động (map_w, map_h) thời gian thực từ Server
        float srvW = 0.0f, srvH = 0.0f;
        if (extractJsonFloatField(resp, "map_w", srvW) && extractJsonFloatField(resp, "map_h", srvH)) {
          syncMapDimensionsFromServer(srvW, srvH);
        }

        float cmdIdFloat = 0.0f;
        int incomingId = extractJsonFloatField(resp, "cmd_id", cmdIdFloat) ? (int)cmdIdFloat : 0;

        // Trích xuất chuỗi cmd, route và chữ ký cmd_sig từ JSON bằng bộ hàm chung
        char cmdBuf[48] = {0};
        extractJsonStrField(resp, "cmd", cmdBuf, sizeof(cmdBuf));
        char routeBuf[64] = {0};
        extractJsonStrField(resp, "route", routeBuf, sizeof(routeBuf));
        char sigBuf[40] = {0};
        extractJsonStrField(resp, "cmd_sig", sigBuf, sizeof(sigBuf));
        String cmdStr    = String(cmdBuf);
        String routeStr  = String(routeBuf);
        String cmdSigStr = String(sigBuf);

        bool isNewCmd = (incomingId > lastAckId) || (lastAckId > 0 && incomingId == 1);
        if (incomingId > 0 && isNewCmd && cmdStr.length() > 0) {
          bool sigValid = true;
          if (cmdSigStr.length() > 0) {
            String canonical = String(incomingId) + ":" + cmdStr + ":" + routeStr;
            String expectedSig = computeHmacSha256Hex32(canonical);
            sigValid = constantTimeEquals(cmdSigStr, expectedSig);
          }

          if (!sigValid) {
            Serial.printf("\n❌ [BẢO MẬT HMAC]: Từ chối lệnh #%d ('%s') do chữ ký cmd_sig không hợp lệ!\n",
                          incomingId, cmdStr.c_str());
          } else {
            lastAckId = incomingId;
            Serial.printf("\n>> [LỆNH MẠNG #%d]: cmd='%s' (HMAC Verified)\n", incomingId, cmdStr.c_str());

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
            totalDistM = 0.0f; legProgressM = 0.0f; legArcDistM = 0.0f;
            lastLineYawDeg = 90.0f;
            bridgeOriginX = 0.0f;
            bridgeOriginY = 0.0f;
            bridgeRefHeadingDeg = 90.0f;
            finalAimUx = 0.0f;
            finalAimUy = 1.0f;
            stationCrossBarLatched = false;
            curvedExcursionDetected = false;
            bendCatchAttemptedForGap = false;
            bendCatchStage = 0;
            clearLineHistoryLocked();
            gapStartArcM = 0.0f;
            stillGyroSamples = 0;
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
          } // end else (sigValid)
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
  legArcDistM = 0.0f;
  lastLineYawDeg = 90.0f;
  bridgeRefHeadingDeg = 90.0f;
  lastWheelMotionMs = millis();
  currentStation = STATION_A;
  currentLeg = LEG_AB;

  Serial.println("\n=========================================================================");
  Serial.println("   AGV_VER2 - TEST 10: BÁM LINE TỰ DO & CỨU HỘ ĐỨT ĐOẠN (MANG TẢI)");
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
