#include "Arduino.h"
#include "TFT_eSPI.h"

#define DEV_DEBUG_FLAG 1

#define DAC_B_PIN 25
#define DAV_A_PIN 26

#define CHANNELS 2

struct ChannelData {
    uint16_t vpp[CHANNELS];
    uint16_t vmean[CHANNELS];
    float frequency[CHANNELS];
};

ChannelData calulateResults = { { 0, 0 }, { 0, 0 }, { 0.0f, 0.0f } };

void showInfo(ChannelData results);
ChannelData calculateInfo(uint8_t* data);

HardwareSerial dataSerial(1);
TFT_eSPI tft = TFT_eSPI();

SemaphoreHandle_t fsm_mutex;
SemaphoreHandle_t tft_mutex;
SemaphoreHandle_t freq_mutex;
EventGroupHandle_t data_ready_event_group;

#define TASK1_EVENT_BIT (1 << 0)
#define TASK2_EVENT_BIT (1 << 0)

void drawWave(uint8_t* data, size_t len, int y_range, int x_offset, int y_offset);

void taskWaveRefresh(void* arg);
void taskInfoRefresh(void* arg);
void serialRecvTask(void* arg);

void setup()
{
    fsm_mutex = xSemaphoreCreateMutex();
    tft_mutex = xSemaphoreCreateMutex();
    freq_mutex = xSemaphoreCreateMutex();
    data_ready_event_group = xEventGroupCreate();

    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);

    Serial.setRxBufferSize(8000);
    dataSerial.setRxBufferSize(8000);

    Serial.begin(921600);
    dataSerial.begin(921600, SERIAL_8N1, 16, 17);

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

    xTaskCreate(taskWaveRefresh, "TftRefresh", 4096, NULL, 2, NULL);
    xTaskCreate(taskInfoRefresh, "InfoRefresh", 4096, NULL, 2, NULL);
    xTaskCreate(serialRecvTask, "SerialRecv", 4096, NULL, 1, NULL);

    Serial.println("ESP32 Started - Ready to receive data");
    digitalWrite(2, HIGH);
}

void loop()
{
}

static uint8_t cur_buf[1000];
void fsm(uint8_t buf);

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
            if (xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                fsm(cur_buf[i]);
                xSemaphoreGive(fsm_mutex);
            }
        }
    }
}

void fsm(uint8_t buf)
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
            if (DEV_DEBUG_FLAG)
                // Serial.printf("Frame %d | %d | %d | %d | %d\n", recv_count++, ((uint32_t*)recv_freq_buf)[0], ((uint32_t*)recv_freq_buf)[1], ((uint16_t*)recv_adc_buf)[0], ((uint16_t*)recv_adc_buf)[1]);
                xEventGroupSetBits(data_ready_event_group, TASK1_EVENT_BIT | TASK2_EVENT_BIT);
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

int waveXRange = 4000; // 200 - 8000 4000测249khz
int waveYRange = 128; // 16 - 1024
int waveXOffset = 0; // 0 - 8000 / waveXRange
int waveYOffset = 0; // 

void taskWaveRefresh(void* arg)
{
    for (;;) {
        xEventGroupWaitBits(data_ready_event_group, TASK1_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (xSemaphoreTake(tft_mutex, portMAX_DELAY) == pdTRUE) {
            {
                drawWave(recv_adc_buf, waveXRange, waveYRange, waveXOffset, waveYOffset);
            }
            xSemaphoreGive(tft_mutex);
        }
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
        tft.drawFastVLine(x, 0, screen_h, TFT_BLACK);

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
}

void taskInfoRefresh(void* arg)
{
    for (;;) {
        xEventGroupWaitBits(data_ready_event_group, TASK2_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        calulateResults = calculateInfo(recv_adc_buf);
        calulateResults.frequency[0] = ((uint32_t*)recv_freq_buf)[0] / 1000.0f;
        calulateResults.frequency[1] = ((uint32_t*)recv_freq_buf)[1] / 1000.0f;
        // dacWrite(DAV_A_PIN, (calulateResults.vmean[0] + calulateResults.vpp[0] * 0.4) / 16);
        // dacWrite(DAC_B_PIN, (calulateResults.vmean[1] + calulateResults.vpp[1] * 0.4) / 16);
        dacWrite(DAV_A_PIN, (calulateResults.vmean[0] + calulateResults.vpp[0] * 0.4) / 16);
        dacWrite(DAC_B_PIN, (calulateResults.vmean[1] + calulateResults.vpp[1] * 0.4) / 16);
        Serial.printf("Frame %d | CH1: %d , %2.2f , %d | CH2: %d , %2.2f , %d\n",
            recv_count++, calulateResults.vpp[0], calulateResults.vmean[0] * 3.3 / 4096, ((uint32_t*)recv_freq_buf)[0],
            calulateResults.vpp[1], calulateResults.vmean[1] * 3.3 / 4096, ((uint32_t*)recv_freq_buf)[1]);
        if (xSemaphoreTake(tft_mutex, portMAX_DELAY) == pdTRUE) {
            {
                showInfo(calulateResults);
            }
            xSemaphoreGive(tft_mutex);
        }
    }
}

void showInfo(ChannelData results)
{
    tft.setCursor(0, 156);
    tft.setTextFont(2);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.printf("CH1 %4.1f Vpp, %8.2f kHz   \n", calulateResults.vpp[0] * 3.3 / 4096, calulateResults.frequency[0]);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.printf("CH2 %4.1f Vpp, %8.2f kHz   \n", calulateResults.vpp[1] * 3.3 / 4096, calulateResults.frequency[1]);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Frame %d \n", recv_count);
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
