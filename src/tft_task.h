#include "Arduino.h"
#include "TFT_eSPI.h"

extern TFT_eSPI tft;
extern SemaphoreHandle_t tft_mutex;
extern SemaphoreHandle_t freq_mutex;
extern SemaphoreHandle_t wave_set_mutex;

void taskTftInit();
void taskWaveRefresh(void* arg);
void taskInfoRefresh(void* arg);
void taskWaveSet(void* arg);