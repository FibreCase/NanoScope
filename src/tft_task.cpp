#include "tft_task.h"
#include "bluetooth_task.h"
#include "data_controller.h"
#include "data_serial_task.h"
#include "main.h"

TFT_eSPI tft = TFT_eSPI();

SemaphoreHandle_t tft_mutex;
SemaphoreHandle_t wave_set_mutex;

void taskTftInit()
{
    tft_mutex = xSemaphoreCreateMutex();
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

    xTaskCreatePinnedToCore(taskWaveSet, "WaveSet", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(taskWaveRefresh, "TftRefresh", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(taskPhaseDiffCalc, "PhaseDiffCalc", 8192 * 2, NULL, 5, NULL, 0);
}

/**
 * 绘制波形
 */

const int screen_w = 240;
const int screen_h = 128;
const int adc_max = 4095;

void tftWaveRefresh(uint8_t* recv_adc_buf, size_t len, int y_range, int x_offset, int y_offset)
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
            uint16_t ch1 = recv_adc_buf[(i + x_offset) * 4] | (recv_adc_buf[(i + x_offset) * 4 + 1] << 8);
            uint16_t ch2 = recv_adc_buf[(i + x_offset) * 4 + 2] | (recv_adc_buf[(i + x_offset) * 4 + 3] << 8);
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

void tftInfoRefresh(WaveInfo wave_info, WaveSettings wave_settings, float phase_diff)
{
    // 显示波形范围和偏移量 130 - 146
    tft.setTextFont(2);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(0, 130);
    tft.print(wave_info.axis_info[0]);

    // 显示通道信息 174 - 222
    tft.setTextFont(2);
    tft.setCursor(0, 174);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println(wave_info.channel_info[0]);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println(wave_info.channel_info[1]);

    // 显示相位差信息
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (phase_diff == 270.0) {
        tft.printf("Phase Diff: No       \n");
    } else {
        tft.printf("Phase Diff: %6.1f° \n", phase_diff);
    }

    // 显示其他信息 222 - 240
    tft.setCursor(0, 224);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.println(wave_info.mode_info);

    // // 显示名称 148 - 174
    // tft.setTextFont(4);
    // tft.setTextColor(TFT_WHITE, TFT_BLACK);
    // tft.setCursor(0, 148);
    // tft.println("Simple Oscilloscope");
}

uint8_t recv_adc_buf_read_ptr_copy[RECV_ADC_DATA_SIZE];
float phase_diff = 270.0f;
bool is_waiting_for_phase_diff = false;

void taskWaveRefresh(void* arg)
{
    for (;;) {
        xEventGroupWaitBits(data_ready_event_group, TASK_TFT_WAVE_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        WaveSettings wave_settings_copy;

        if (xSemaphoreTake(wave_set_mutex, portMAX_DELAY) == pdTRUE) {
            {
                wave_settings_copy = wave_settings;
            }
            xSemaphoreGive(wave_set_mutex);
        }

        if (xSemaphoreTake(adc_recv_mutex, portMAX_DELAY) == pdTRUE) {
            {
                memcpy(recv_adc_buf_read_ptr_copy, recv_adc_buf_read_ptr, RECV_ADC_DATA_SIZE);
            }
            xSemaphoreGive(adc_recv_mutex);
        }

        calculate_results = calculateInfo(recv_adc_buf_read_ptr_copy, recv_freq_buf, calculate_results);
        wave_info = generateWaveInfo(calculate_results, wave_info, wave_settings, recv_count);

        if (abs(calculate_results.frequency[0] - calculate_results.frequency[1]) < 0.002f && calculate_results.frequency[0] > 0.05f) {
            xEventGroupSetBits(data_ready_event_group, TASK_TFT_PHASE_EVENT_BIT);
            is_waiting_for_phase_diff = true;
        } else {
            phase_diff = 270.0f;
        }

        if (wave_settings.is_triggered) {
            setCompareVoltage(wave_settings.trigger_voltage, wave_settings.trigger_voltage);
        } else {
            setCompareVoltage(calculate_results.vpp[0] > 273 ? (calculate_results.vmean[0] / 16) : 196,
                calculate_results.vpp[1] > 273 ? (calculate_results.vmean[1] / 16) : 196);
        }

        if (DEV_DEBUG_FLAG)
            Serial.printf("Frame %3d | CH1: %d , %2.2f , %d | CH2: %d , %2.2f , %d\n",
                recv_count++, calculate_results.vpp[0], calculate_results.vmean[0] * 7.5 / 4096 - 2.5, ((uint32_t*)recv_freq_buf)[0],
                calculate_results.vpp[1], calculate_results.vmean[1] * 7.5 / 4096 - 2.5, ((uint32_t*)recv_freq_buf)[1]);

        if (xSemaphoreTake(tft_mutex, portMAX_DELAY) == pdTRUE) {
            {
                tftWaveRefresh(recv_adc_buf_read_ptr_copy, wave_settings_copy.x_range, wave_settings_copy.y_range, wave_settings_copy.x_offset, wave_settings_copy.y_offset);
                if (is_waiting_for_phase_diff) {
                    xEventGroupWaitBits(data_ready_event_group, TASK_TFT_PHASE_READY_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
                    is_waiting_for_phase_diff = false;
                }
                tftInfoRefresh(wave_info, wave_settings, phase_diff);
            }
            xSemaphoreGive(tft_mutex);
        }
    }
}

void taskPhaseDiffCalc(void* arg)
{
    for (;;) {
        xEventGroupWaitBits(data_ready_event_group, TASK_TFT_PHASE_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        phase_diff = calculatePhaseDiff(recv_adc_buf_read_ptr_copy, wave_settings.is_high_sample_rate ? 250000 : 125000, calculate_results.frequency[0]);
        xEventGroupSetBits(data_ready_event_group, TASK_TFT_PHASE_READY_EVENT_BIT);
    }
}
