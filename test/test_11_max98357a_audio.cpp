#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_system.h>
#include <driver/i2s.h>
#include "test_voice_samples.h"

/*
 * ===================================================================================
 * BƯỚC 11: TEST GIỌNG NÓI THẬT ĐỘC LẬP KHÔNG CẦN CÁP USB (NGUỒN ESP / BOARD MỞ RỘNG)
 * -----------------------------------------------------------------------------------
 * VẬN HÀNH HOÀN TOÀN BẰNG PIN + LOA + NÚT BOOT (KHÔNG CẦN CẮM DÂY SERIAL VÀO PC):
 *   1. Ngay khi bật công tắc nguồn:
 *      - Bình thường: sau 1.5s tự nói ở mức tối đa 100%:
 *        "Hệ thống xe tự hành A G V sẵn sàng hoạt động."
 *      - Nếu trước đó bị tụt áp (ESP_RST_BROWNOUT): tự hạ về 15%, kêu 2 bíp trầm và
 *        nói to: "Cảnh báo sụt áp nguồn, hệ thống tự hạ âm lượng về mức an toàn."
 *
 *   2. Điều khiển ngay trên mạch chỉ bằng NÚT BOOT (GPIO 0):
 *      - Bấm 1 cái (Single Click)  : Nghe lần lượt 4 câu giọng tiếng Việt thật (ở mức 100%).
 *      - Bấm đúp 2 cái (Double)    : Chạy TEST BẬC THANG NGUỒN BOARD MỞ RỘNG:
 *                                    Loa tự đọc to "Mức 15%..." -> ... -> "75%..." -> "100%...".
 *                                    Nếu nấc nào làm sập/rè nguồn, bạn nghe bằng tai là biết ngay!
 *      - Nhấn giữ >= 1.2s (Hold)   : Đổi nóng bộ chân I2S (42/41/40/SD=1 <-> 1/2/14/SD=NC)
 *                                    và nói thử ngay nếu vừa chuyển sang bộ chân bạn cắm.
 *
 *   3. (Tùy chọn) Khi cắm PC mở Serial Monitor 115200 baud:
 *      - [1..4], [0], [5], [t], [+/-], [s], [p], [m] vẫn hoạt động song song.
 *
 * SƠ ĐỒ ĐẤU DÂY MẶC ĐỊNH:
 *   - BCLK : GPIO 42 | LRC : GPIO 41 | DIN : GPIO 40 | SD : GPIO 1 (tùy chọn)
 *   - VIN  : Nguồn từ ESP / Board mở rộng (khuyên dùng cọc 5V + tụ >=470uF) | GND : G chung
 * ===================================================================================
 */

#define BUTTON_BOOT_PIN      0

// --- CHÂN CẢM BIẾN SẴN TRÊN XE (TÙY CHỌN CHO LOG SERIAL) ---
#define MPU_SDA_PIN          4
#define MPU_SCL_PIN          5
#define MPU6050_ADDR         0x68

#define INA_SDA_PIN          8
#define INA_SCL_PIN          9
#define INA226_ADDR          0x40
#define INA226_REG_CONFIG    0x00
#define INA226_REG_SHUNTV    0x01
#define INA226_REG_BUSV      0x02
#define INA226_REG_CALIB     0x05

// --- CẤU HÌNH I2S & GIỚI HẠN AN TOÀN NGUỒN BOARD MỞ RỘNG ---
#define I2S_PORT             I2S_NUM_0
#define AUDIO_SAMPLE_RATE    16000
#define AUDIO_CHUNK_FRAMES   128
#define AUDIO_SAFE_MAX_VOL   100  // Cho phép tăng tối đa 100% (có tự bảo vệ Brownout về 15%)

struct AudioPinProfile {
  const char* name;
  int bclkPin;
  int lrcPin;
  int dinPin;
  int sdPin; // -1 nếu không nối GPIO điều khiển SD
};

static const AudioPinProfile PIN_PROFILES[2] = {
  { "Profile 0 (J3 liền kề: BCLK=42, LRC=41, DIN=40, SD=1)", 42, 41, 40,  1 },
  { "Profile 1 (Dự phòng:   BCLK=1,  LRC=2,  DIN=14, SD=-1)", 1,  2, 14, -1 }
};

enum AudioEventType : uint8_t {
  EVT_VOICE_STARTUP = 0,
  EVT_VOICE_DISPATCH,
  EVT_VOICE_ARRIVED,
  EVT_VOICE_ALERT,
  EVT_VOICE_BROWNOUT_WARN,
  EVT_VOICE_ALL,
  EVT_VOICE_LADDER_TEST,
  EVT_QUICK_BEEP,
  EVT_SWITCH_PROFILE
};

struct AudioQueueMsg {
  AudioEventType type;
  int arg;
};

struct PowerGyroMetrics {
  int inaSamples = 0;
  float minBusV = 1e6f;
  float sumBusV = 0.0f;
  float maxCurA = 0.0f;
  float sumCurA = 0.0f;
  int mpuSamples = 0;
  float minGz = 1e6f;
  float maxGz = -1e6f;
};

static QueueHandle_t audioQueue = NULL;
static volatile int currentProfileIdx = 0;
static volatile int volumePercent = 100;     // Mức mặc định MAX 100% (tự giảm về 15% nếu phát hiện sụt áp Brownout)
static volatile bool autoShutdownSd = true;  // Tự tắt ampli khi nghỉ để bảo vệ ổn áp & giảm sôi nền
static volatile bool isAudioBusy = false;
static bool i2sInstalled = false;
static bool bootedFromBrownout = false;

static TwoWire I2C_INA = TwoWire(1);
static bool mpuDetected = false;
static bool inaDetected = false;
static float inaShuntOhm = 0.100f;

// --- NGUYÊN MẪU HÀM ---
void printHelpMenu();
void enqueueAudio(AudioEventType type, int arg = 0);

// ===================================================================================
// CHẨN ĐOÁN PHỤ TRỢ INA226 & MPU-6050 (AN TOÀN KHI CHẠY ĐỘC LẬP)
// ===================================================================================
bool readMpuGyroZDegS(float &outGzDegS) {
  if (!mpuDetected) return false;
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU6050_ADDR, 2) != 2) return false;
  int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
  outGzDegS = (float)raw / 32.8f;
  return true;
}

bool readInaPower(float &outBusV, float &outCurrentA) {
  if (!inaDetected) return false;
  uint16_t rawBus = 0, rawShunt = 0;

  I2C_INA.beginTransmission(INA226_ADDR);
  I2C_INA.write(INA226_REG_BUSV);
  if (I2C_INA.endTransmission(false) != 0) return false;
  if (I2C_INA.requestFrom((int)INA226_ADDR, 2) != 2) return false;
  rawBus = ((uint16_t)I2C_INA.read() << 8) | (uint16_t)I2C_INA.read();

  I2C_INA.beginTransmission(INA226_ADDR);
  I2C_INA.write(INA226_REG_SHUNTV);
  if (I2C_INA.endTransmission(false) != 0) return false;
  if (I2C_INA.requestFrom((int)INA226_ADDR, 2) != 2) return false;
  rawShunt = ((uint16_t)I2C_INA.read() << 8) | (uint16_t)I2C_INA.read();

  outBusV = (float)rawBus * 0.00125f;
  float shuntMv = (float)((int16_t)rawShunt) * 0.0025f;
  outCurrentA = (shuntMv / 1000.0f) / inaShuntOhm;
  return true;
}

void sampleSensorsInto(PowerGyroMetrics *m) {
  if (!m) return;
  float v = 0, a = 0, gz = 0;
  if (readInaPower(v, a)) {
    m->minBusV = min(m->minBusV, v);
    m->sumBusV += v;
    m->maxCurA = max(m->maxCurA, a);
    m->sumCurA += a;
    m->inaSamples++;
  }
  if (readMpuGyroZDegS(gz)) {
    m->minGz = min(m->minGz, gz);
    m->maxGz = max(m->maxGz, gz);
    m->mpuSamples++;
  }
}

void probeOptionalSensors() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000);
  Wire.setTimeOut(25);
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    mpuDetected = true;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();
    delay(20);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A); Wire.write(0x03);
    Wire.endTransmission();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); Wire.write(0x10);
    Wire.endTransmission();
  }

  I2C_INA.begin(INA_SDA_PIN, INA_SCL_PIN, 100000);
  I2C_INA.setTimeOut(25);
  I2C_INA.beginTransmission(INA226_ADDR);
  if (I2C_INA.endTransmission() == 0) {
    inaDetected = true;
    I2C_INA.beginTransmission(INA226_ADDR);
    I2C_INA.write(INA226_REG_CONFIG);
    I2C_INA.write(0x41); I2C_INA.write(0x27);
    I2C_INA.endTransmission();

    uint16_t calVal = (uint16_t)(0.00512f / (0.0001f * inaShuntOhm));
    I2C_INA.beginTransmission(INA226_ADDR);
    I2C_INA.write(INA226_REG_CALIB);
    I2C_INA.write((uint8_t)(calVal >> 8));
    I2C_INA.write((uint8_t)(calVal & 0xFF));
    I2C_INA.endTransmission();
  }
}

// ===================================================================================
// TẦNG PHẦN CỨNG I2S CHO MAX98357A (ESP-IDF v4.4.7)
// ===================================================================================
bool initAudioDriver(const AudioPinProfile &prof) {
  if (i2sInstalled) {
    i2s_zero_dma_buffer(I2S_PORT);
    i2s_driver_uninstall(I2S_PORT);
    i2sInstalled = false;
  }

  if (prof.sdPin >= 0) {
    pinMode(prof.sdPin, OUTPUT);
    digitalWrite(prof.sdPin, autoShutdownSd ? LOW : HIGH);
  }

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = AUDIO_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT; // Xuất L=R tương thích mọi trạng thái chân SD
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 6;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;

  esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  if (err != ESP_OK) return false;

  i2s_pin_config_t pinCfg = {};
  pinCfg.mck_io_num = I2S_PIN_NO_CHANGE;
  pinCfg.bck_io_num = prof.bclkPin;
  pinCfg.ws_io_num = prof.lrcPin;
  pinCfg.data_out_num = prof.dinPin;
  pinCfg.data_in_num = I2S_PIN_NO_CHANGE;

  err = i2s_set_pin(I2S_PORT, &pinCfg);
  if (err != ESP_OK) {
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  i2sInstalled = true;
  Serial.printf("✅ [I2S READY]: %s\n", prof.name);
  return true;
}

void writeSilenceMs(int ms) {
  if (!i2sInstalled || ms <= 0) return;
  int totalFrames = (AUDIO_SAMPLE_RATE * ms) / 1000;
  int16_t buf[AUDIO_CHUNK_FRAMES * 2] = {0};
  int remaining = totalFrames;
  while (remaining > 0) {
    int batch = (remaining > AUDIO_CHUNK_FRAMES) ? AUDIO_CHUNK_FRAMES : remaining;
    size_t written = 0;
    i2s_write(I2S_PORT, buf, (size_t)batch * 2 * sizeof(int16_t), &written, pdMS_TO_TICKS(100));
    remaining -= batch;
  }
}

void setAmpEnabled(bool enable) {
  const AudioPinProfile &prof = PIN_PROFILES[currentProfileIdx];
  if (enable) {
    if (prof.sdPin >= 0) {
      digitalWrite(prof.sdPin, HIGH);
      writeSilenceMs(18);
    }
  } else {
    writeSilenceMs(20);
    if (i2sInstalled) i2s_zero_dma_buffer(I2S_PORT);
    if (prof.sdPin >= 0 && autoShutdownSd) {
      digitalWrite(prof.sdPin, LOW);
    }
  }
}

void playVoiceClip(const int16_t *pcmData, size_t sampleLen, const char *caption,
                   int overrideVolPct = -1, PowerGyroMetrics *metricsOut = NULL) {
  if (!i2sInstalled || !pcmData || sampleLen == 0) return;

  int vol = (overrideVolPct >= 0) ? overrideVolPct : (int)volumePercent;
  vol = constrain(vol, 1, AUDIO_SAFE_MAX_VOL);
  float durSec = (float)sampleLen / (float)AUDIO_SAMPLE_RATE;

  Serial.printf("🗣️ [VOICE @ %d%% | %.2fs]: \"%s\"\n", vol, durSec, caption);

  int16_t buf[AUDIO_CHUNK_FRAMES * 2];
  setAmpEnabled(true);

  size_t pos = 0;
  int chunkCount = 0;
  while (pos < sampleLen) {
    size_t batch = sampleLen - pos;
    if (batch > AUDIO_CHUNK_FRAMES) batch = AUDIO_CHUNK_FRAMES;

    for (size_t i = 0; i < batch; i++) {
      int16_t raw = (int16_t)pgm_read_word(&pcmData[pos + i]);
      int32_t scaled = ((int32_t)raw * vol) / 100;
      if (scaled > 32767) scaled = 32767;
      else if (scaled < -32768) scaled = -32768;
      int16_t s = (int16_t)scaled;
      buf[i * 2]     = s;
      buf[i * 2 + 1] = s;
    }

    size_t written = 0;
    i2s_write(I2S_PORT, buf, batch * 2 * sizeof(int16_t), &written, pdMS_TO_TICKS(200));
    pos += batch;
    chunkCount++;

    if (metricsOut && (chunkCount % 10 == 0)) {
      sampleSensorsInto(metricsOut);
    }
  }

  setAmpEnabled(false);
}

void playBeepPattern(const float *freqs, const int *durs, int count, int overrideVolPct = -1) {
  if (!i2sInstalled || count <= 0) return;
  int vol = (overrideVolPct >= 0) ? overrideVolPct : (int)volumePercent;
  vol = constrain(vol, 1, AUDIO_SAFE_MAX_VOL);
  float amp = (32760.0f * (float)vol) / 100.0f;

  setAmpEnabled(true);
  for (int k = 0; k < count; k++) {
    int totalFrames = (AUDIO_SAMPLE_RATE * durs[k]) / 1000;
    int fadeFrames = (AUDIO_SAMPLE_RATE * 10) / 1000;
    int16_t buf[AUDIO_CHUNK_FRAMES * 2];
    float phase = 0.0f;
    float phaseInc = (2.0f * (float)M_PI * freqs[k]) / (float)AUDIO_SAMPLE_RATE;

    for (int f = 0; f < totalFrames; f += AUDIO_CHUNK_FRAMES) {
      int batch = min(AUDIO_CHUNK_FRAMES, totalFrames - f);
      for (int i = 0; i < batch; i++) {
        int pos = f + i;
        float env = 1.0f;
        if (pos < fadeFrames) env = (float)pos / (float)fadeFrames;
        else if (pos >= totalFrames - fadeFrames) env = (float)(totalFrames - 1 - pos) / (float)fadeFrames;
        int16_t s = (int16_t)(sinf(phase) * amp * env);
        buf[i * 2] = s;
        buf[i * 2 + 1] = s;
        phase += phaseInc;
        if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
      }
      size_t written = 0;
      i2s_write(I2S_PORT, buf, (size_t)batch * 2 * sizeof(int16_t), &written, pdMS_TO_TICKS(100));
    }
    writeSilenceMs(35);
  }
  setAmpEnabled(false);
}

void playQuickBeep() {
  const float freqs[2] = { 440.0f, 880.0f };
  const int durs[2] = { 100, 140 };
  Serial.printf("🔔 [BEEP @ %d%%]: 440Hz -> 880Hz\n", (int)volumePercent);
  playBeepPattern(freqs, durs, 2);
}

// ===================================================================================
// TEST BẬC THANG TỰ ĐỌC MỨC % BẰNG GIỌNG THẬT (KHÔNG CẦN CÁP USB / SERIAL)
// ===================================================================================
void runExtensionPowerVoiceLadderTest() {
  struct LadderStep {
    int vol;
    const int16_t *announcePcm;
    size_t announceLen;
    const char *announceTxt;
  };
  static const LadderStep STEPS[6] = {
    { 15,  VOICE_LV15_PCM,  VOICE_LV15_LEN,  "Mức mười lăm phần trăm." },
    { 25,  VOICE_LV25_PCM,  VOICE_LV25_LEN,  "Mức hai mươi lăm phần trăm." },
    { 35,  VOICE_LV35_PCM,  VOICE_LV35_LEN,  "Mức ba mươi lăm phần trăm." },
    { 50,  VOICE_LV50_PCM,  VOICE_LV50_LEN,  "Mức năm mươi phần trăm." },
    { 75,  VOICE_LV75_PCM,  VOICE_LV75_LEN,  "Mức bảy mươi lăm phần trăm." },
    { 100, VOICE_LV100_PCM, VOICE_LV100_LEN, "Mức một trăm phần trăm." }
  };

  Serial.println("\n⚡ [TEST BẬC THANG NGUỒN BOARD MỞ RỘNG]: 15% -> 25% -> 35% -> 50% -> 75% -> 100%");

  PowerGyroMetrics base;
  for (int i = 0; i < 15; i++) {
    sampleSensorsInto(&base);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  for (int idx = 0; idx < 6; idx++) {
    const LadderStep &st = STEPS[idx];
    PowerGyroMetrics m;

    // Đọc to mức % trước (bằng chính mức % đó) -> nghỉ 150ms -> đọc câu mẫu
    playVoiceClip(st.announcePcm, st.announceLen, st.announceTxt, st.vol, &m);
    vTaskDelay(pdMS_TO_TICKS(150));
    playVoiceClip(VOICE_STARTUP_PCM, VOICE_STARTUP_LEN,
                  "Hệ thống xe tự hành A G V sẵn sàng hoạt động.", st.vol, &m);

    if (m.inaSamples > 0 && base.inaSamples > 0) {
      float baseV = base.sumBusV / base.inaSamples;
      float baseA = base.sumCurA / base.inaSamples;
      float avgV = m.sumBusV / m.inaSamples;
      float avgA = m.sumCurA / m.inaSamples;
      Serial.printf("   ⚡ [INA226 @ %d%%]: TB %.3fV (Min %.3fV, Sụt %+.0fmV) | Dòng %+.0fmA (Đỉnh %.0fmA)\n",
                    st.vol, avgV, m.minBusV, (m.minBusV - baseV) * 1000.0f,
                    (avgA - baseA) * 1000.0f, m.maxCurA * 1000.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(650)); // Nghỉ hồi tụ nguồn giữa các nấc
  }
}

// ===================================================================================
// CORE 0: TASK XỬ LÝ ÂM THANH ĐỘC LẬP
// ===================================================================================
void audioTask(void *pvParameters) {
  initAudioDriver(PIN_PROFILES[currentProfileIdx]);

  AudioQueueMsg msg;
  while (true) {
    if (xQueueReceive(audioQueue, &msg, portMAX_DELAY) == pdTRUE) {
      isAudioBusy = true;
      switch (msg.type) {
        case EVT_VOICE_STARTUP:
          playVoiceClip(VOICE_STARTUP_PCM, VOICE_STARTUP_LEN,
                        "Hệ thống xe tự hành A G V sẵn sàng hoạt động.");
          break;
        case EVT_VOICE_DISPATCH:
          playVoiceClip(VOICE_DISPATCH_PCM, VOICE_DISPATCH_LEN,
                        "Đã nhận lệnh vận chuyển hàng hóa, xe bắt đầu di chuyển.");
          break;
        case EVT_VOICE_ARRIVED:
          playVoiceClip(VOICE_ARRIVED_PCM, VOICE_ARRIVED_LEN,
                        "Xe đã đến trạm kho, vui lòng kiểm tra và dỡ hàng.");
          break;
        case EVT_VOICE_ALERT:
          playVoiceClip(VOICE_ALERT_PCM, VOICE_ALERT_LEN,
                        "Cảnh báo, xe mất vạch dẫn đường, vui lòng kiểm tra.");
          break;
        case EVT_VOICE_BROWNOUT_WARN: {
          const float warnFreqs[2] = { 320.0f, 260.0f };
          const int warnDurs[2] = { 160, 220 };
          playBeepPattern(warnFreqs, warnDurs, 2, 15);
          vTaskDelay(pdMS_TO_TICKS(150));
          playVoiceClip(VOICE_BROWNOUT_PCM, VOICE_BROWNOUT_LEN,
                        "Cảnh báo sụt áp nguồn, hệ thống tự hạ âm lượng về mức an toàn.", 15);
          break;
        }
        case EVT_VOICE_ALL:
          playVoiceClip(VOICE_STARTUP_PCM, VOICE_STARTUP_LEN,
                        "Hệ thống xe tự hành A G V sẵn sàng hoạt động.");
          vTaskDelay(pdMS_TO_TICKS(450));
          playVoiceClip(VOICE_DISPATCH_PCM, VOICE_DISPATCH_LEN,
                        "Đã nhận lệnh vận chuyển hàng hóa, xe bắt đầu di chuyển.");
          vTaskDelay(pdMS_TO_TICKS(450));
          playVoiceClip(VOICE_ARRIVED_PCM, VOICE_ARRIVED_LEN,
                        "Xe đã đến trạm kho, vui lòng kiểm tra và dỡ hàng.");
          vTaskDelay(pdMS_TO_TICKS(450));
          playVoiceClip(VOICE_ALERT_PCM, VOICE_ALERT_LEN,
                        "Cảnh báo, xe mất vạch dẫn đường, vui lòng kiểm tra.");
          break;
        case EVT_VOICE_LADDER_TEST:
          runExtensionPowerVoiceLadderTest();
          break;
        case EVT_QUICK_BEEP:
          playQuickBeep();
          break;
        case EVT_SWITCH_PROFILE: {
          currentProfileIdx = (currentProfileIdx + 1) % 2;
          initAudioDriver(PIN_PROFILES[currentProfileIdx]);
          const float swFreqs[2] = { 660.0f, 880.0f };
          const int swDurs[2] = { 90, 90 };
          playBeepPattern(swFreqs, swDurs, 2);
          vTaskDelay(pdMS_TO_TICKS(120));
          playVoiceClip(VOICE_STARTUP_PCM, VOICE_STARTUP_LEN,
                        "Hệ thống xe tự hành A G V sẵn sàng hoạt động.");
          break;
        }
      }
      isAudioBusy = false;
    }
  }
}

void enqueueAudio(AudioEventType type, int arg) {
  if (!audioQueue) return;
  AudioQueueMsg msg = { type, arg };
  xQueueSend(audioQueue, &msg, 0);
}

void printHelpMenu() {
  const AudioPinProfile &prof = PIN_PROFILES[currentProfileIdx];
  Serial.println("\n=========================================================================");
  Serial.println("   AGV_VER2 - TEST 11: TEST GIỌNG NÓI THẬT ĐỘC LẬP (KHÔNG CẦN PC)");
  Serial.println("=========================================================================");
  Serial.printf("Chân I2S : %s\n", prof.name);
  Serial.printf("An toàn  : Âm lượng = %d%% (Trần %d%%) | Tự ngắt SD khi nghỉ: %s\n",
                volumePercent, AUDIO_SAFE_MAX_VOL, autoShutdownSd ? "BẬT" : "TẮT");
  Serial.println(">> ĐIỀU KHIỂN TRỰC TIẾP TRÊN XE BẰNG NÚT BOOT (GPIO 0):");
  Serial.println("   - Bấm 1 cái        : Nghe luân phiên 4 câu tiếng Việt thật");
  Serial.println("   - Bấm đúp 2 cái    : Chạy TEST BẬC THANG (đọc to 15% -> 25% -> 35% -> 50% -> 75% -> 100%)");
  Serial.println("   - Nhấn giữ >= 1.2s : Đổi bộ chân I2S (42/41/40/1 <-> 1/2/14/NC) & nói thử");
  Serial.println(">> (Tùy chọn Serial 115200): [1..4], [0], [5], [t], [+/-], [s], [p]");
  Serial.println("-------------------------------------------------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

  esp_reset_reason_t rst = esp_reset_reason();
  if (rst == ESP_RST_BROWNOUT) {
    bootedFromBrownout = true;
    volumePercent = 15;
  }

  probeOptionalSensors();

  audioQueue = xQueueCreate(6, sizeof(AudioQueueMsg));
  xTaskCreatePinnedToCore(audioTask, "AudioVoiceTask", 6144, NULL, 1, NULL, 0);

  printHelpMenu();

  if (bootedFromBrownout) {
    delay(1000);
    enqueueAudio(EVT_VOICE_BROWNOUT_WARN);
  } else {
    delay(1500);
    enqueueAudio(EVT_VOICE_STARTUP);
  }
}

void handleBootButtonStandalone() {
  static int stableState = HIGH;
  static int lastRead = HIGH;
  static unsigned long lastBounceMs = 0;
  static unsigned long pressStartMs = 0;
  static unsigned long lastReleaseMs = 0;
  static bool longPressHandled = false;
  static int pendingClicks = 0;
  static int bootClipIdx = 0;

  int raw = digitalRead(BUTTON_BOOT_PIN);
  unsigned long now = millis();
  if (raw != lastRead) {
    lastBounceMs = now;
    lastRead = raw;
  }

  if ((now - lastBounceMs) > 30 && raw != stableState) {
    stableState = raw;
    if (stableState == LOW) {
      pressStartMs = now;
      longPressHandled = false;
    } else {
      if (!longPressHandled) {
        pendingClicks++;
        lastReleaseMs = now;
        if (pendingClicks >= 2) {
          pendingClicks = 0;
          enqueueAudio(EVT_VOICE_LADDER_TEST);
        }
      }
    }
  }

  if (stableState == LOW && !longPressHandled && (now - pressStartMs) >= 1200) {
    longPressHandled = true;
    pendingClicks = 0;
    enqueueAudio(EVT_SWITCH_PROFILE);
  }

  if (pendingClicks == 1 && stableState == HIGH && (now - lastReleaseMs) >= 420) {
    pendingClicks = 0;
    static const AudioEventType clips[4] = {
      EVT_VOICE_STARTUP, EVT_VOICE_DISPATCH, EVT_VOICE_ARRIVED, EVT_VOICE_ALERT
    };
    enqueueAudio(clips[bootClipIdx]);
    bootClipIdx = (bootClipIdx + 1) % 4;
  }
}

void loop() {
  handleBootButtonStandalone();

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == '1' || ch == 'v' || ch == 'V') {
      enqueueAudio(EVT_VOICE_STARTUP);
    } else if (ch == '2') {
      enqueueAudio(EVT_VOICE_DISPATCH);
    } else if (ch == '3') {
      enqueueAudio(EVT_VOICE_ARRIVED);
    } else if (ch == '4') {
      enqueueAudio(EVT_VOICE_ALERT);
    } else if (ch == '0') {
      enqueueAudio(EVT_VOICE_ALL);
    } else if (ch == '5') {
      enqueueAudio(EVT_VOICE_LADDER_TEST);
    } else if (ch == 't' || ch == 'T') {
      enqueueAudio(EVT_QUICK_BEEP);
    } else if (ch == '+' || ch == '=') {
      volumePercent = min((int)AUDIO_SAFE_MAX_VOL, (int)volumePercent + 5);
      Serial.printf("🔊 [VOLUME]: %d%%\n", volumePercent);
      enqueueAudio(EVT_VOICE_STARTUP);
    } else if (ch == '-' || ch == '_') {
      volumePercent = max(5, (int)volumePercent - 5);
      Serial.printf("🔉 [VOLUME]: %d%%\n", volumePercent);
      enqueueAudio(EVT_VOICE_STARTUP);
    } else if (ch == 's' || ch == 'S') {
      autoShutdownSd = !autoShutdownSd;
      const AudioPinProfile &prof = PIN_PROFILES[currentProfileIdx];
      if (!isAudioBusy && prof.sdPin >= 0) {
        digitalWrite(prof.sdPin, autoShutdownSd ? LOW : HIGH);
      }
      Serial.printf("🔇 [AUTO SHUTDOWN SD]: %s\n", autoShutdownSd ? "BẬT" : "TẮT");
    } else if (ch == 'p' || ch == 'P') {
      enqueueAudio(EVT_SWITCH_PROFILE);
    } else if (ch == 'm' || ch == 'M' || ch == '?') {
      printHelpMenu();
    }
  }

  vTaskDelay(pdMS_TO_TICKS(20));
}
