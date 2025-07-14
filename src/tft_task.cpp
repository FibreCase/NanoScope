#include "tft_task.h"
#include "bt_serial_task.h"
#include "data_serial_task.h"
#include "main.h"

TFT_eSPI tft = TFT_eSPI();

SemaphoreHandle_t tft_mutex;
SemaphoreHandle_t freq_mutex;
SemaphoreHandle_t wave_set_mutex;

uint32_t wave_x_range = 8000; // 960 - 8000
uint32_t wave_y_range = 128; // 16 - 1024
uint32_t wave_x_offset = 0;
uint32_t wave_y_offset = 0;
uint32_t is_high_voltage = 1; // 是否高电压
uint32_t is_high_sample_rate = 0;

/**
 * 绘制波形
 */
void taskTftInit()
{
    tft_mutex = xSemaphoreCreateMutex();
    freq_mutex = xSemaphoreCreateMutex();
    wave_set_mutex = xSemaphoreCreateMutex();

    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextWrap(false);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(4);

    tft.setCursor(0, 122);
    tft.println("FibreCase 2025");
    tft.println("Simple Oscilloscope");
    tft.println("Loading...");
    delay(1000);
    // 清理屏幕 y 0 - 146, 176 - 240
    tft.fillRect(0, 0, 240, 146, TFT_BLACK);
    tft.fillRect(0, 174, 240, 66, TFT_BLACK);

    xTaskCreate(taskWaveRefresh, "TftRefresh", 4096, NULL, 3, NULL);
    xTaskCreate(taskInfoRefresh, "InfoRefresh", 4096, NULL, 3, NULL);
    xTaskCreate(taskWaveSet, "WaveSet", 4096, NULL, 3, NULL);
}

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

        int y_ch1 = screen_h - (avg_ch1 * y_range / adc_max - y_offset);
        int y_ch2 = screen_h - (avg_ch2 * y_range / adc_max - y_offset);

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

    for (uint8_t i = 0; i < 5; i++) {
        tft.drawFastHLine(0, i * 32, screen_w, TFT_DARKGREY);
    }
}

/**
 * 信息刷新
 */

#define DAV_A_PIN 25
#define DAC_B_PIN 26

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
        dacWrite(DAV_A_PIN, calculateResults.vpp[0] > 250 ? (calculateResults.vmean[0] / 16) : 200); // 剔除Vpp小于0.2V的信号频率
        dacWrite(DAC_B_PIN, calculateResults.vpp[1] > 250 ? (calculateResults.vmean[1] / 16) : 200);
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

String axis_info[2];

void showInfo(ChannelData results)
{
    tft.setTextFont(2);

    // 显示波形范围和偏移量 130 - 146
    axis_info[0] = String(is_high_voltage ? wave_y_offset * 7.5 / wave_y_range - 2.5 : wave_y_offset * 3.3 / wave_y_range, 2) + " V";
    axis_info[1] = is_high_sample_rate ? String(wave_x_range / (250.0 * 6), 2) + " ms/div | " + String(is_high_voltage ? 7.5 * 128 / (wave_y_range * 4) : 3.3 * 128 / (wave_y_range * 4), 2) + " V/div\n" : String(wave_x_range / (125.0 * 6), 2) + " ms/div | " + String(is_high_voltage ? 7.5 * 128 / (wave_y_range * 4) : 3.3 * 128 / (wave_y_range * 4), 2) + " V/div\n";

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(0, 130);
    tft.print(axis_info[0]);
    tft.setCursor(239 - tft.textWidth(axis_info[1]), 130);
    tft.print(axis_info[1]);

    // 显示名称 148 - 174
    // 显示通道信息 174 - 222
    tft.setCursor(0, 174);
    if (is_high_voltage) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.printf("%4.2f Vpp | %5.2f Vmean | %7.3f kHz  \n", calculateResults.vpp[0] * 7.5 / 4096, calculateResults.vmean[0] * 7.5 / 4096 - 2.5, calculateResults.frequency[0]);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.printf("%4.2f Vpp | %5.2f Vmean | %7.3f kHz  \n", calculateResults.vpp[1] * 7.5 / 4096, calculateResults.vmean[1] * 7.5 / 4096 - 2.5, calculateResults.frequency[1]);

    } else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.printf("%4.2f Vpp | %5.2f Vmean | %7.3f kHz  \n", calculateResults.vpp[0] * 3.3 / 4096, calculateResults.vmean[0] * 3.3 / 4096, calculateResults.frequency[0]);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.printf("%4.2f Vpp | %5.2f Vmean | %7.3f kHz  \n", calculateResults.vpp[1] * 3.3 / 4096, calculateResults.vmean[1] * 3.3 / 4096, calculateResults.frequency[1]);
    }
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.printf("Phase Diff: 179.3° \n");

    // 显示其他信息
    tft.setCursor(0, 224);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.printf("SR %c | Mode %c | Frame %3d \n", is_high_sample_rate ? 'H' : 'L', is_high_voltage ? 'H' : 'L', recv_count);
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
        if (xSemaphoreTake(wave_set_mutex, portMAX_DELAY) == pdTRUE) {
            updateWaveRange(bt_recv_id, ((uint32_t*)bt_recv_buf)[0]);
            xSemaphoreGive(wave_set_mutex);
        }
    }
}

#define WAVE_X_RANGE_MIN 1
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
    case BT_RECV_ID_VOLTAGE:
        is_high_voltage = (value == 1) ? 1 : 0;
        break;
    case BT_RECV_ID_SAMPLE_RATE:
        is_high_sample_rate = (value == 1) ? 1 : 0;
        break;
    default:
        if (DEV_DEBUG_FLAG)
            Serial.printf("[ERROR] Wave Set Unknown ID: %d\n", id);
        break;
    }
}