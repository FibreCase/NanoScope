#include "main.h"
#include "TFT_eSPI.h"

#define DAV_0_PIN 25
#define DAC_1_PIN 26
#define LOW_PASS_FILTER_RATE 0.2

#define WAVE_X_RANGE_MIN 1
#define WAVE_X_RANGE_MAX 8000
#define WAVE_Y_RANGE_MIN 16
#define WAVE_Y_RANGE_MAX 1024
#define WAVE_X_OFFSET_MIN 0
#define WAVE_X_OFFSET_MAX 8000 - 960
#define WAVE_Y_OFFSET_MIN 0
#define WAVE_Y_OFFSET_MAX 1024

extern TFT_eSPI tft;
extern SemaphoreHandle_t tft_mutex;
extern SemaphoreHandle_t freq_mutex;
extern SemaphoreHandle_t wave_set_mutex;

struct ChannelData {
    uint16_t vpp[CHANNELS];
    uint16_t vpp_last[CHANNELS];
    uint16_t vmean[CHANNELS];
    float frequency[CHANNELS];
};

struct WaveInfo {
    String axis_info[2];
    String channel_info[CHANNELS];
    String mode_info;
};

struct WaveSettings {
    uint32_t x_range;      // X轴范围
    uint32_t y_range;      // Y轴范围
    uint32_t x_offset;     // X轴偏移
    uint32_t y_offset;     // Y轴偏移
    uint32_t is_high_voltage; // 高电压模式
    uint32_t is_high_sample_rate; // 高采样率模式
    uint32_t is_triggered; // 是否触发
};

extern WaveSettings wave_settings;

void taskTftInit();
void taskWaveRefresh(void* arg);
void taskInfoRefresh(void* arg);
void taskWaveSet(void* arg);

void drawWave(uint8_t* data, size_t len, int y_range, int x_offset, int y_offset);