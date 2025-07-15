#include "main.h"
#include "data_controller.h"
#include "TFT_eSPI.h"

#define WAVE_X_RANGE_MIN 1
#define WAVE_X_RANGE_MAX RECV_ADC_DATA_SIZE
#define WAVE_Y_RANGE_MIN 16
#define WAVE_Y_RANGE_MAX 1024
#define WAVE_X_OFFSET_MIN 0
#define WAVE_X_OFFSET_MAX RECV_ADC_DATA_SIZE - 960
#define WAVE_Y_OFFSET_MIN 0
#define WAVE_Y_OFFSET_MAX 1024

extern TFT_eSPI tft;
extern SemaphoreHandle_t tft_mutex;
extern SemaphoreHandle_t wave_set_mutex;

void taskTftInit();
void taskWaveRefresh(void* arg);
void taskPhaseDiffCalc(void* arg);
void taskWaveSet(void* arg);

void tftDashboardRefresh(WaveInfo wave_info, WaveSettings wave_settings);