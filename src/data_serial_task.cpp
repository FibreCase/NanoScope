#include "data_serial_task.h"
#include "main.h"

HardwareSerial dataSerial(1);

SemaphoreHandle_t serial_fsm_mutex;

static uint8_t cur_buf[1000];
uint8_t status = SERIAL_FSM_IDLE;
uint8_t recv_freq_buf[RECV_FREQ_DATA_SIZE];
uint32_t recv_freq_num[2] = { 0, 0 };
uint8_t recv_freq_index = 0;
uint8_t recv_adc_buf[RECV_ADC_DATA_SIZE];
uint16_t recv_adc_index = 0;
uint8_t recv_count = 0;
unsigned long last_recv_time = 0;
uint8_t error_count = 0;

/**
 * 串口接收状态机
 */

void taskDataSerialRecvInit()
{
    serial_fsm_mutex = xSemaphoreCreateMutex();
    dataSerial.setRxBufferSize(8000);
    dataSerial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
    dataSerial.flush(false);
    xTaskCreate(taskDataSerialRecv, "DataSerialRecv", 4096, NULL, 1, NULL);
}

void taskDataSerialRecv(void* arg)
{
    while (1) {
        if (status != SERIAL_FSM_IDLE && millis() - last_recv_time > TIMEOUT_MS) {
            Serial.println("Timeout - Reset to IDLE");
            status = SERIAL_FSM_IDLE;
            recv_adc_index = 0;
            error_count = 0;
        }
        last_recv_time = millis();

        size_t bytesRead = dataSerial.readBytes(cur_buf, sizeof(cur_buf));
        for (size_t i = 0; i < bytesRead; i++) {
            if (xSemaphoreTake(serial_fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                serialFsm(cur_buf[i]);
                xSemaphoreGive(serial_fsm_mutex);
            }
        }
    }
}

void serialFsm(uint8_t buf)
{
    switch (status) {
    case SERIAL_FSM_IDLE:
        if (buf == 0xFF) {
            status = SERIAL_FSM_HEADER;
            error_count = 0; // 重置错误计数
        } else {
            error_count++;
            if (error_count >= 100) { // 每100个错误字节输出一次
                if (DEV_DEBUG_FLAG)
                    Serial.printf("Header Not Found ( 0x%02X, %d times )\n", buf, error_count);
                error_count = 0;
            }
        }
        break;

    case SERIAL_FSM_HEADER:
        if (buf == 0xFE) {
            recv_freq_index = 0;
            status = SERIAL_FSM_FREQ_DATA;
            error_count = 0;
        } else {
            if (DEV_DEBUG_FLAG)
                Serial.printf("Header Error ( 0x%02X )\n", buf);
            status = SERIAL_FSM_IDLE;
            error_count++;
        }
        break;

    case SERIAL_FSM_FREQ_DATA:
        recv_freq_buf[recv_freq_index++] = buf;
        if (recv_freq_index >= RECV_FREQ_DATA_SIZE) {
            status = SERIAL_FSM_ADC_DATA;
            recv_adc_index = 0;
            error_count = 0;
        }
        break;

    case SERIAL_FSM_ADC_DATA:
        recv_adc_buf[recv_adc_index++] = buf;
        if (recv_adc_index >= RECV_ADC_DATA_SIZE) {
            xEventGroupSetBits(data_ready_event_group, TASK_TFT_WAVE_EVENT_BIT | TASK_TFT_INFO_EVENT_BIT);
            status = SERIAL_FSM_IDLE;
            error_count = 0;
        }
        break;

    default:
        if (DEV_DEBUG_FLAG)
            Serial.println("FSM Error - Reset to IDLE");
        status = SERIAL_FSM_IDLE;
        recv_adc_index = 0;
        recv_freq_index = 0;
        error_count = 0;
        break;
    }
}