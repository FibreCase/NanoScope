#include "Arduino.h"

#define CHANNELS 2

#define DEV_DEBUG_FLAG 1
#define LED_PIN 2

#define TASK_TFT_WAVE_EVENT_BIT (1 << 0)
#define TASK_TFT_PHASE_EVENT_BIT (1 << 1)
#define TASK_WAVE_SET_EVENT_BIT (1 << 2)
#define TASK_TFT_PHASE_READY_EVENT_BIT (1 << 3)

extern EventGroupHandle_t data_ready_event_group;
