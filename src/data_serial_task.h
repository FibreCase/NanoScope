#include "Arduino.h"

#define SERIAL_RX_PIN 16
#define SERIAL_TX_PIN 17
#define SERIAL_BAUD_RATE 921600

#define SERIAL_FSM_IDLE 1
#define SERIAL_FSM_HEADER 2
#define SERIAL_FSM_FREQ_DATA 3
#define SERIAL_FSM_ADC_DATA 4

#define RECV_FREQ_DATA_SIZE 2 * 4
#define RECV_ADC_DATA_SIZE 4000 * 2
#define TIMEOUT_MS 1000 // 超时时间1秒

extern uint8_t status;
extern uint8_t recv_freq_buf[RECV_FREQ_DATA_SIZE];
extern uint8_t recv_adc_buf[RECV_ADC_DATA_SIZE];
extern uint8_t recv_freq_index;
extern uint16_t recv_adc_index;
extern uint8_t recv_count;
extern unsigned long last_recv_time;
extern uint8_t error_count;

extern SemaphoreHandle_t serial_fsm_mutex;
extern HardwareSerial dataSerial;

void taskDataSerialRecvInit();
void taskDataSerialRecv(void* arg);
