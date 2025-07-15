#include "data_serial_task.h"
#include "main.h"

HardwareSerial dataSerial(1);

SemaphoreHandle_t serial_fsm_mutex;
SemaphoreHandle_t adc_recv_mutex;
SemaphoreHandle_t freq_recv_mutex;

static uint8_t cur_buf[8020];
uint8_t data_serial_status = SERIAL_FSM_IDLE;
uint8_t recv_freq_buf[RECV_FREQ_DATA_SIZE];
uint8_t recv_freq_index = 0;
uint8_t recv_adc_buf[2][RECV_ADC_DATA_SIZE + RECV_ADC_DATA_ENDER_SIZE]; // 双缓冲区用于接收ADC数据
uint16_t recv_adc_index = 0;
uint8_t* recv_adc_buf_write_ptr = recv_adc_buf[0]; // 指向当前接收的ADC数据缓冲区
uint8_t* recv_adc_buf_read_ptr = recv_adc_buf[1]; // 指向上次接收的ADC数据缓冲区
uint8_t recv_count = 0;
unsigned long last_recv_time = 0;
uint8_t error_count = 0;

/**
 * 串口接收状态机
 */

void taskDataSerialRecvInit()
{
    serial_fsm_mutex = xSemaphoreCreateMutex();
    adc_recv_mutex = xSemaphoreCreateMutex();
    freq_recv_mutex = xSemaphoreCreateMutex();
    dataSerial.setRxBufferSize(16384);

    dataSerial.begin(SERIAL_BAUD_RATE, SERIAL_MODE, SERIAL_RX_PIN, SERIAL_TX_PIN);
    dataSerial.flush(false);
    xTaskCreatePinnedToCore(taskDataSerialRecv, "DataSerialRecv", 4096, NULL, 5, NULL, 1);
}

void dataSerialFsm(uint8_t buf)
{
    switch (data_serial_status) {
    case SERIAL_FSM_IDLE:
        if (buf == 0xFF) {
            data_serial_status = SERIAL_FSM_HEADER;
            error_count = 0; // 重置错误计数
        } else {
            error_count++;
            if (error_count >= 100) { // 每100个错误字节输出一次
                // if (DEV_DEBUG_FLAG)
                //     Serial.printf("Header Not Found ( 0x%02X, %d times )\n", buf, error_count);
                error_count = 0;
            }
        }
        break;

    case SERIAL_FSM_HEADER:
        if (buf == 0xFE) {
            recv_freq_index = 0;
            data_serial_status = SERIAL_FSM_FREQ_DATA;
            error_count = 0;
        } else {
            if (DEV_DEBUG_FLAG)
                Serial.printf("Header Error ( 0x%02X )\n", buf);
            data_serial_status = SERIAL_FSM_IDLE;
            error_count++;
        }
        break;

    case SERIAL_FSM_FREQ_DATA:
        recv_freq_buf[recv_freq_index++] = buf;
        if (recv_freq_index >= RECV_FREQ_DATA_SIZE) {
            data_serial_status = SERIAL_FSM_ADC_DATA;
            recv_adc_index = 0;
            error_count = 0;
        }
        break;

    case SERIAL_FSM_ADC_DATA:
        recv_adc_buf_write_ptr[recv_adc_index++] = buf;
        if (recv_adc_index >= RECV_ADC_DATA_SIZE + RECV_ADC_DATA_ENDER_SIZE) {
            // 交换读写缓冲区
            if (recv_adc_buf_write_ptr[RECV_ADC_DATA_SIZE + RECV_ADC_DATA_ENDER_SIZE - 3] == 0xDE && recv_adc_buf_write_ptr[RECV_ADC_DATA_SIZE + RECV_ADC_DATA_ENDER_SIZE - 2] == 0xAD && recv_adc_buf_write_ptr[RECV_ADC_DATA_SIZE + RECV_ADC_DATA_ENDER_SIZE - 1] == 0xEF) {
                if (xSemaphoreTake(adc_recv_mutex, portMAX_DELAY) == pdTRUE) {
                    {
                        uint8_t* temp = recv_adc_buf_write_ptr;
                        recv_adc_buf_write_ptr = recv_adc_buf_read_ptr;
                        recv_adc_buf_read_ptr = temp;
                    }
                    xSemaphoreGive(adc_recv_mutex);
                }
                xEventGroupSetBits(data_ready_event_group, TASK_TFT_WAVE_EVENT_BIT);
            } else {
                if (DEV_DEBUG_FLAG)
                    Serial.println("ADC Data Error - Invalid Ender");
            }
            last_recv_time = millis();
            data_serial_status = SERIAL_FSM_IDLE;
            error_count = 0;
        }
        break;

    default:
        if (DEV_DEBUG_FLAG)
            Serial.println("FSM Error - Reset to IDLE");
        data_serial_status = SERIAL_FSM_IDLE;
        recv_adc_index = 0;
        recv_freq_index = 0;
        error_count = 0;
        break;
    }
}

void taskDataSerialRecv(void* arg)
{
    while (1) {
        if (millis() - last_recv_time > TIMEOUT_MS) {
            if (data_serial_status != SERIAL_FSM_IDLE) {
                Serial.println("Timeout - Reset to IDLE");
                data_serial_status = SERIAL_FSM_IDLE;
                recv_adc_index = 0;
                error_count = 0;
            }
            last_recv_time = millis();
        }

        size_t bytesRead = dataSerial.readBytes(cur_buf, sizeof(cur_buf));
        if (xSemaphoreTake(serial_fsm_mutex, portMAX_DELAY) == pdTRUE) {
            for (size_t i = 0; i < bytesRead; i++) {
                dataSerialFsm(cur_buf[i]);
            }
            xSemaphoreGive(serial_fsm_mutex);
        }
    }
}

/**
 * 串口发送 - 控制STM32
 */
void sendCommandToSTM32(uint8_t command, uint8_t* data)
{
    dataSerial.write(0xFA); // Header
    dataSerial.write(0xFE); // Start byte
    dataSerial.write(command); // Command ID
    if (data != nullptr) {
        for (int i = 0; i < 4; i++) {
            dataSerial.write(data[i]); // Data
        }
    } else {
        for (int i = 0; i < 4; i++) {
            dataSerial.write(0); // Fill with zeros if no data
        }
    }
}

void setCompareVoltage(uint8_t v_1, uint8_t v_2)
{
    dacWrite(DAC_0_PIN, constrain(v_1, 0, 255)); // 限制电压值在0-255范围内
    dacWrite(DAC_1_PIN, constrain(v_2, 0, 255)); // 限制电压值在0-255范围内
}