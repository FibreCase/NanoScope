#include "Arduino.h"
#include "BluetoothSerial.h"
#include "TFT_eSPI.h"

#define DEV_DEBUG_FLAG 1

#define CHANNELS 2

struct ChannelData {
    uint16_t vpp[CHANNELS];
    uint16_t vmean[CHANNELS];
    float frequency[CHANNELS];
};

HardwareSerial dataSerial(1);
BluetoothSerial SerialBT;
TFT_eSPI tft = TFT_eSPI();

SemaphoreHandle_t serial_fsm_mutex;
SemaphoreHandle_t bt_fsm_mutex;
SemaphoreHandle_t tft_mutex;
SemaphoreHandle_t freq_mutex;
SemaphoreHandle_t wave_set_mutex;
EventGroupHandle_t data_ready_event_group;

#define TASK_TFT_WAVE_EVENT_BIT (1 << 0)
#define TASK_TFT_INFO_EVENT_BIT (1 << 1)
#define TASK_WAVE_SET_EVENT_BIT (1 << 2)

void taskWaveRefresh(void* arg);
void taskInfoRefresh(void* arg);
void taskWaveSet(void* arg);
void serialRecvTask(void* arg);
void taskBluetoothRecv(void* arg);

void setup()
{
    serial_fsm_mutex = xSemaphoreCreateMutex();
    bt_fsm_mutex = xSemaphoreCreateMutex();
    tft_mutex = xSemaphoreCreateMutex();
    freq_mutex = xSemaphoreCreateMutex();
    wave_set_mutex = xSemaphoreCreateMutex();
    data_ready_event_group = xEventGroupCreate();

    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);

    Serial.setRxBufferSize(8000);
    dataSerial.setRxBufferSize(8000);

    Serial.begin(921600);
    dataSerial.begin(921600, SERIAL_8N1, 16, 17);
    SerialBT.begin("Fibre_ESP32_OSC");

    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(4);

    tft.setCursor(0, 130);
    tft.println("Simple Oscilloscope");

    digitalWrite(2, LOW);
    delay(2000);

    dataSerial.flush(false);
    Serial.flush(false);

    xTaskCreate(taskWaveRefresh, "TftRefresh", 4096, NULL, 3, NULL);
    xTaskCreate(taskInfoRefresh, "InfoRefresh", 4096, NULL, 3, NULL);
    xTaskCreate(taskWaveSet, "WaveSet", 4096, NULL, 3, NULL);
    xTaskCreate(serialRecvTask, "SerialRecv", 4096, NULL, 1, NULL);
    xTaskCreate(taskBluetoothRecv, "BluetoothRecv", 4096, NULL, 2, NULL);

    Serial.println("ESP32 Started - Ready to receive data");
    digitalWrite(2, HIGH);
}

void loop()
{
    SerialBT.write(0xF0);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * 串口接收状态机
 */

static uint8_t cur_buf[1000];
void serial_fsm(uint8_t buf);

#define SERIAL_FSM_IDLE 1
#define SERIAL_FSM_HEADER 2
#define SERIAL_FSM_FREQ_DATA 3
#define SERIAL_FSM_ADC_DATA 4

#define RECV_FREQ_DATA_SIZE 2 * 4
#define RECV_ADC_DATA_SIZE 4000 * 2
#define TIMEOUT_MS 1000 // 超时时间1秒

uint8_t status = SERIAL_FSM_IDLE;

uint8_t recv_freq_buf[RECV_FREQ_DATA_SIZE];
uint32_t recv_freq_num[2] = { 0, 0 };
uint8_t recv_freq_index = 0;

uint8_t recv_adc_buf[RECV_ADC_DATA_SIZE];
uint16_t recv_adc_index = 0;

uint8_t recv_count = 0;
unsigned long last_recv_time = 0;
uint8_t error_count = 0;

void serialRecvTask(void* arg)
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
                serial_fsm(cur_buf[i]);
                xSemaphoreGive(serial_fsm_mutex);
            }
        }
    }
}

void serial_fsm(uint8_t buf)
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

/**
 * 蓝牙接收状态机
 */

void bt_fsm(uint8_t buf);

void taskBluetoothRecv(void* arg)
{
    uint8_t buf[64];
    while (1) {
        int len = SerialBT.available();
        if (len > 0) {
            int readLen = SerialBT.readBytes(buf, sizeof(buf));
            for (int i = 0; i < readLen; ++i) {
                if (xSemaphoreTake(bt_fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    bt_fsm(buf[i]);
                    xSemaphoreGive(bt_fsm_mutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#define BT_RECV_ID_X_RANGE 0x01
#define BT_RECV_ID_Y_RANGE 0x02
#define BT_RECV_ID_X_OFFSET 0x03
#define BT_RECV_ID_Y_OFFSET 0x04

#define BT_FSM_IDLE 1
#define BT_FSM_HEADER 2
#define BT_FSM_ID 3
#define BT_FSM_DATA 4

uint8_t bt_recv_id = 0;
uint8_t bt_recv_count = 0;
uint8_t bt_recv_buf[4] = { 0 };
uint8_t bt_status = BT_FSM_IDLE;

void bt_fsm(uint8_t buf)
{
    switch (bt_status) {
    case BT_FSM_IDLE:
        if (buf == 0xFF) {
            bt_status = BT_FSM_HEADER;
        } else {
            if (DEV_DEBUG_FLAG)
                Serial.printf("BT Header Not Found ( 0x%02X )\n", buf);
        }
        break;

    case BT_FSM_HEADER:
        if (buf == 0xFE) {
            bt_status = BT_FSM_ID;
        } else {
            if (DEV_DEBUG_FLAG)
                Serial.printf("BT Header Error ( 0x%02X )\n", buf);
            bt_status = BT_FSM_IDLE;
        }
        break;

    case BT_FSM_ID:
        if (buf == BT_RECV_ID_X_RANGE || buf == BT_RECV_ID_Y_RANGE || buf == BT_RECV_ID_X_OFFSET || buf == BT_RECV_ID_Y_OFFSET) {
            bt_recv_id = buf;
            bt_recv_count = 0;
            bt_status = BT_FSM_DATA;
        } else {
            if (DEV_DEBUG_FLAG)
                Serial.printf("BT ID Error ( 0x%02X )\n", buf);
            bt_status = BT_FSM_IDLE;
        }
        break;

    case BT_FSM_DATA:
        bt_recv_buf[bt_recv_count++] = buf;
        if (bt_recv_count >= 4) {
            bt_recv_count = 0;
            bt_status = BT_FSM_IDLE;
            xEventGroupSetBits(data_ready_event_group, TASK_WAVE_SET_EVENT_BIT);
        }
        break;
    }
}

/**
 * 绘制波形
 */

uint32_t wave_x_range = 8000; // 960 - 8000
uint32_t wave_y_range = 128; // 16 - 1024
uint32_t wave_x_offset = 0;
uint32_t wave_y_offset = 0;
uint32_t is_high_voltage = 0; // 是否高电压

void drawWave(uint8_t* data, size_t len, int y_range, int x_offset, int y_offset);

void taskWaveRefresh(void* arg)
{
    for (;;) {
        xEventGroupWaitBits(data_ready_event_group, TASK_TFT_WAVE_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (xSemaphoreTake(wave_set_mutex, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(tft_mutex, portMAX_DELAY) == pdTRUE) {
                {
                    drawWave(recv_adc_buf, (size_t)wave_x_range, (int)wave_y_range, (int)wave_x_offset, (int)wave_y_offset);
                }
                xSemaphoreGive(tft_mutex);
            }
        }
        xSemaphoreGive(wave_set_mutex);
    }
}

const int screen_w = 240;
const int screen_h = 128;
const int adc_max = 4095;

void drawWave(uint8_t* data, size_t len, int y_range, int x_offset, int y_offset)
{

    const int ch_samples = len / 4; // 每通道样本数
    float bin_size = (float)ch_samples / screen_w;

    int prev_y_ch1 = -1;
    int prev_y_ch2 = -1;

    for (int x = 0; x < screen_w; x++) {
        // 1. 垂直擦除整列
        tft.drawFastVLine(x, 0, screen_h, x % 40 && x != 239 ? TFT_BLACK : TFT_DARKGREY);

        // 2. 区间数据平均
        int start_idx = round(x * bin_size);
        int end_idx = round((x + 1) * bin_size);
        if (end_idx > ch_samples)
            end_idx = ch_samples;

        uint32_t sum_ch1 = 0, sum_ch2 = 0;
        int count = 0;

        for (int i = start_idx; i < end_idx; i++) {
            uint16_t ch1 = data[(i + x_offset) * 4] | (data[(i + x_offset) * 4 + 1] << 8);
            uint16_t ch2 = data[(i + x_offset) * 4 + 2] | (data[(i + x_offset) * 4 + 3] << 8);
            sum_ch1 += ch1;
            sum_ch2 += ch2;
            count++;
        }

        if (count == 0)
            continue;

        int avg_ch1 = sum_ch1 / count;
        int avg_ch2 = sum_ch2 / count;

        int y_ch1 = screen_h - (avg_ch1 * y_range / adc_max + y_offset);
        int y_ch2 = screen_h - (avg_ch2 * y_range / adc_max + y_offset);

        y_ch1 = constrain(y_ch1, 0, screen_h - 1);
        y_ch2 = constrain(y_ch2, 0, screen_h - 1);

        // 3. 连线绘制（通道1: 绿，通道2: 红）
        if (x > 0 && prev_y_ch1 >= 0 && prev_y_ch2 >= 0) {
            tft.drawLine(x - 1, prev_y_ch1, x, y_ch1, TFT_GREEN);
            tft.drawLine(x - 1, prev_y_ch2, x, y_ch2, TFT_RED);
        }

        prev_y_ch1 = y_ch1;
        prev_y_ch2 = y_ch2;
    }

    for (uint8_t i = 0; i < 5; i++)
    {
        tft.drawFastHLine(0, i * 32, screen_w, TFT_DARKGREY);
    }
    
}

/**
 * 信息刷新
 */

#define DAV_A_PIN 26
#define DAC_B_PIN 25

ChannelData calculateResults = { { 0, 0 }, { 0, 0 }, { 0.0f, 0.0f } };
ChannelData calculateInfo(uint8_t* data);
void showInfo(ChannelData results);

void taskInfoRefresh(void* arg)
{
    for (;;) {
        xEventGroupWaitBits(data_ready_event_group, TASK_TFT_INFO_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        calculateResults = calculateInfo(recv_adc_buf);
        calculateResults.frequency[0] = ((uint32_t*)recv_freq_buf)[0] / 1000.0f;
        calculateResults.frequency[1] = ((uint32_t*)recv_freq_buf)[1] / 1000.0f;
        dacWrite(DAV_A_PIN, calculateResults.vpp[0] > 250 ? (calculateResults.vmean[0] / 16) : 255); // 剔除Vpp小于0.2V的信号频率
        dacWrite(DAC_B_PIN, calculateResults.vpp[1] > 250 ? (calculateResults.vmean[1] / 16) : 255);
        Serial.printf("Frame %3d | CH1: %d , %2.2f , %d | CH2: %d , %2.2f , %d\n",
            recv_count++, calculateResults.vpp[0], calculateResults.vmean[0] * 3.3 / 4096, ((uint32_t*)recv_freq_buf)[0],
            calculateResults.vpp[1], calculateResults.vmean[1] * 3.3 / 4096, ((uint32_t*)recv_freq_buf)[1]);
        if (xSemaphoreTake(wave_set_mutex, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(tft_mutex, portMAX_DELAY) == pdTRUE) {
                {
                    showInfo(calculateResults);
                }
                xSemaphoreGive(tft_mutex);
            }
            xSemaphoreGive(wave_set_mutex);
        }
    }
}

void showInfo(ChannelData results)
{
    tft.setCursor(0, 156);
    tft.setTextFont(2);

    if (is_high_voltage) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.printf("CH1 %4.2f Vpp | %4.2f Vmean \n    %8.3f kHz \n", calculateResults.vpp[0] * 10 / 4096 - 2.5, calculateResults.vmean[0] * 10 / 4096 - 2.5, calculateResults.frequency[0]);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.printf("CH2 %4.2f Vpp | %4.2f Vmean \n    %8.3f kHz \n", calculateResults.vpp[1] * 10 / 4096 - 2.5, calculateResults.vmean[1] * 10 / 4096 - 2.5, calculateResults.frequency[1]);

    } else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.printf("CH1 %4.2f Vpp | %4.2f Vmean \n    %8.3f kHz \n", calculateResults.vpp[0] * 3.3 / 4096, calculateResults.vmean[0] * 3.3 / 4096, calculateResults.frequency[0]);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.printf("CH2 %4.2f Vpp | %4.2f Vmean \n    %8.3f kHz \n", calculateResults.vpp[1] * 3.3 / 4096, calculateResults.vmean[1] * 3.3 / 4096, calculateResults.frequency[1]);
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("%c %3d %4d  %4d %4d %4d \n", is_high_voltage ? 'H' : 'L', recv_count, wave_x_range, wave_y_range, wave_x_offset, wave_y_offset);
}

ChannelData calculateInfo(uint8_t* data)
{

    ChannelData results = { { 0, 0 }, { 0, 0 }, { 0.0f, 0.0f } };

    // 计算每个通道的 Vpp
    uint16_t max_ch1 = 0;
    uint16_t min_ch1 = 4095;
    uint16_t max_ch2 = 0;
    uint16_t min_ch2 = 4095;

    for (int i = 0; i < RECV_ADC_DATA_SIZE / 4; i++) {
        uint16_t ch1 = data[i * 4] | (data[i * 4 + 1] << 8);
        uint16_t ch2 = data[i * 4 + 2] | (data[i * 4 + 3] << 8);

        if (ch1 > max_ch1)
            max_ch1 = ch1;
        if (ch1 < min_ch1)
            min_ch1 = ch1;
        if (ch2 > max_ch2)
            max_ch2 = ch2;
        if (ch2 < min_ch2)
            min_ch2 = ch2;
    }

    results.vpp[0] = (max_ch1 - min_ch1) < 0 ? 0 : (max_ch1 - min_ch1);
    results.vpp[1] = (max_ch2 - min_ch2) < 0 ? 0 : (max_ch2 - min_ch2);

    // 计算每个通道的 Vmean
    results.vmean[0] = (max_ch1 + min_ch1) / 2;
    results.vmean[1] = (max_ch2 + min_ch2) / 2;

    return results;
}

/**
 * 更新波形范围和偏移量
 */

void updateWaveRange(uint8_t id, uint32_t value);

void taskWaveSet(void* arg)
{
    for (;;) {
        xEventGroupWaitBits(data_ready_event_group, TASK_WAVE_SET_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        SerialBT.write(0xFA);
        if (xSemaphoreTake(wave_set_mutex, portMAX_DELAY) == pdTRUE) {
            updateWaveRange(bt_recv_id, ((uint32_t*)bt_recv_buf)[0]);
            xSemaphoreGive(wave_set_mutex);
        }
    }
}

#define WAVE_X_RANGE_MIN 960
#define WAVE_X_RANGE_MAX 8000
#define WAVE_Y_RANGE_MIN 16
#define WAVE_Y_RANGE_MAX 1024
#define WAVE_X_OFFSET_MIN 0
#define WAVE_X_OFFSET_MAX 8000 - 960
#define WAVE_Y_OFFSET_MIN 0
#define WAVE_Y_OFFSET_MAX 1024

void updateWaveRange(uint8_t id, uint32_t value)
{
    switch (id) {
    case BT_RECV_ID_X_RANGE:
        wave_x_range = constrain(value, WAVE_X_RANGE_MIN, WAVE_X_RANGE_MAX);
        break;
    case BT_RECV_ID_Y_RANGE:
        wave_y_range = constrain(value, WAVE_Y_RANGE_MIN, WAVE_Y_RANGE_MAX);
        break;
    case BT_RECV_ID_X_OFFSET:
        wave_x_offset = constrain(value, WAVE_X_OFFSET_MIN, WAVE_X_OFFSET_MAX);
        break;
    case BT_RECV_ID_Y_OFFSET:
        wave_y_offset = constrain(value, WAVE_Y_OFFSET_MIN, WAVE_Y_OFFSET_MAX);
        break;
    default:
        if (DEV_DEBUG_FLAG)
            Serial.printf("[ERROR] Wave Set Unknown ID: %d\n", id);
        break;
    }
}
