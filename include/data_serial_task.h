#include <Arduino.h>

#define SERIAL_RX_PIN 16
#define SERIAL_TX_PIN 17
#define SERIAL_BAUD_RATE 2250000
#define SERIAL_MODE SERIAL_8N1

#define SERIAL_FSM_IDLE 1
#define SERIAL_FSM_HEADER 2
#define SERIAL_FSM_FREQ_DATA 3
#define SERIAL_FSM_ADC_DATA 4

#define RECV_FREQ_DATA_SIZE (2 * 4)
#define RECV_ADC_DATA_SIZE (4000 * 2) // 4000个采样点，每个点2字节，+2字节用于标记数据结束
#define RECV_ADC_DATA_ENDER_SIZE 3 // 结束标记的字节数
#define TIMEOUT_MS 2000 // 超时时间1秒

#define DAC_0_PIN 25
#define DAC_1_PIN 26

#define STM32_CMD_SET_SR 0x01
#define STM32_CMD_SET_TRIG 0x02

extern uint8_t data_serial_status;
extern uint8_t recv_freq_buf[RECV_FREQ_DATA_SIZE];
extern uint8_t recv_adc_buf[2][RECV_ADC_DATA_SIZE + RECV_ADC_DATA_ENDER_SIZE]; // 双缓冲区用于接收ADC数据
extern uint8_t recv_freq_index;
extern uint16_t recv_adc_index;
extern uint8_t recv_count;
extern unsigned long last_recv_time;
extern uint8_t error_count;

extern uint8_t* recv_adc_buf_write_ptr; // 指向当前接收的ADC数据缓冲区
extern uint8_t* recv_adc_buf_read_ptr; // 指向上次接收的ADC数据缓冲区

extern SemaphoreHandle_t serial_fsm_mutex;
extern SemaphoreHandle_t adc_recv_mutex;
extern SemaphoreHandle_t freq_recv_mutex;
extern HardwareSerial dataSerial;

void taskDataSerialRecvInit();
void taskDataSerialRecv(void* arg);
void sendCommandToSTM32(uint8_t command, uint8_t* data);
void setCompareVoltage(uint8_t v_1, uint8_t v_2);
