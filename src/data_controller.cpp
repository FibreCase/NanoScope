#include "data_controller.h"
#include "bluetooth_task.h"
#include "data_serial_task.h"
#include "tft_task.h"

ChannelData calculate_results = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0.0f, 0.0f } };

WaveInfo wave_info = { { "", "" }, { "", "" }, "" };

WaveSettings wave_settings = {
    .x_range = RECV_ADC_DATA_SIZE,
    .y_range = 128,
    .x_offset = 0,
    .y_offset = 0,
    .is_high_voltage = 0,
    .is_high_sample_rate = 0,
    .is_triggered = 0,
    .trigger_voltage = 0,
    .is_channel_inverted = 0
};

ChannelData calculateInfo(uint8_t* recv_adc_buf, uint8_t* recv_freq_buf, ChannelData results)
{
    // 计算每个通道的 Vpp
    uint16_t max_ch1 = 0;
    uint16_t min_ch1 = 4095;
    uint16_t max_ch2 = 0;
    uint16_t min_ch2 = 4095;

    for (int i = 0; i < RECV_ADC_DATA_SIZE / 4; i++) {
        uint16_t ch1 = recv_adc_buf[i * 4] | (recv_adc_buf[i * 4 + 1] << 8);
        uint16_t ch2 = recv_adc_buf[i * 4 + 2] | (recv_adc_buf[i * 4 + 3] << 8);

        if (ch1 > max_ch1)
            max_ch1 = ch1;
        if (ch1 < min_ch1)
            min_ch1 = ch1;
        if (ch2 > max_ch2)
            max_ch2 = ch2;
        if (ch2 < min_ch2)
            min_ch2 = ch2;
    }

    results.vpp_last[0] = results.vpp[0];
    results.vpp_last[1] = results.vpp[1];

    results.vpp[0] = (max_ch1 - min_ch1) < 0 ? 0 : (max_ch1 - min_ch1);
    results.vpp[1] = (max_ch2 - min_ch2) < 0 ? 0 : (max_ch2 - min_ch2);

    results.vpp[0] = LOW_PASS_FILTER_RATE * results.vpp[0] + (1 - LOW_PASS_FILTER_RATE) * results.vpp_last[0];
    results.vpp[1] = LOW_PASS_FILTER_RATE * results.vpp[1] + (1 - LOW_PASS_FILTER_RATE) * results.vpp_last[1];

    // 计算每个通道的 Vmean
    results.vmean[0] = (max_ch1 + min_ch1) / 2;
    results.vmean[1] = (max_ch2 + min_ch2) / 2;

    // 计算每个通道的频率
    results.frequency[0] = ((uint32_t*)recv_freq_buf)[0] / 1000.0f;
    results.frequency[1] = ((uint32_t*)recv_freq_buf)[1] / 1000.0f;

    return results;
}

WaveInfo generateWaveInfo(ChannelData calculate_results, WaveInfo wave_info, WaveSettings wave_settings, uint16_t recv_count)
{
    wave_info.axis_info[0] = String(wave_settings.is_high_voltage ? wave_settings.y_offset * 7.5 / wave_settings.y_range - 2.5 : wave_settings.y_offset * 3.3 / wave_settings.y_range, 2) + " V | " + String(wave_settings.is_high_sample_rate ? wave_settings.x_range / 250 / 12.0 : wave_settings.x_range / 125 / 12.0, 2) + " ms/div | " + String(wave_settings.is_high_voltage ? 7.5 * 128 / 4 / wave_settings.y_range : 3.3 * 128 / 4 / wave_settings.y_range, 2) + " V/div  ";

    char ch0_info[37];
    snprintf(
        ch0_info, sizeof(ch0_info),
        "%4.2f Vpp | %5.2f Vmean | %7.3f kHz ",
        calculate_results.vpp[0] * (wave_settings.is_high_voltage ? 7.5f : 3.3f) / 4096.0f,
        calculate_results.vmean[0] * (wave_settings.is_high_voltage ? 7.5f : 3.3f) / 4096.0f - (wave_settings.is_high_voltage ? 2.5f : 0.0f),
        calculate_results.frequency[0]);
    wave_info.channel_info[0] = String(ch0_info);

    char ch1_info[37];
    snprintf(
        ch1_info, sizeof(ch1_info),
        "%4.2f Vpp | %5.2f Vmean | %7.3f kHz ",
        calculate_results.vpp[1] * (wave_settings.is_high_voltage ? 7.5f : 3.3f) / 4096.0f,
        calculate_results.vmean[1] * (wave_settings.is_high_voltage ? 7.5f : 3.3f) / 4096.0f - (wave_settings.is_high_voltage ? 2.5f : 0.0f),
        calculate_results.frequency[1]);
    wave_info.channel_info[1] = String(ch1_info);

    wave_info.mode_info = "SR " + String(wave_settings.is_high_sample_rate ? "H" : "L") + " | Mode " + String(wave_settings.is_high_voltage ? "H" : "L") + " | TR " + (wave_settings.is_triggered ? "O" : "N") + " | Frame " + String(recv_count) + "   ";

    return wave_info;
}

/**
 * 更新波形范围和偏移量
 */
WaveSettings updateWaveSettings(uint8_t id, uint32_t value, WaveSettings wave_settings)
{
    switch (id) {
    case BT_RECV_ID_X_RANGE:
        wave_settings.x_range = constrain(value, WAVE_X_RANGE_MIN, WAVE_X_RANGE_MAX);
        break;
    case BT_RECV_ID_Y_RANGE:
        wave_settings.y_range = constrain(value, WAVE_Y_RANGE_MIN, WAVE_Y_RANGE_MAX);
        break;
    case BT_RECV_ID_X_OFFSET:
        wave_settings.x_offset = constrain(value, WAVE_X_OFFSET_MIN, WAVE_X_OFFSET_MAX);
        break;
    case BT_RECV_ID_Y_OFFSET:
        wave_settings.y_offset = constrain(value, WAVE_Y_OFFSET_MIN, WAVE_Y_OFFSET_MAX);
        break;
    case BT_RECV_ID_VOLTAGE:
        wave_settings.is_high_voltage = (value == 1) ? 1 : 0;
        break;
    case BT_RECV_ID_SAMPLE_RATE:
        wave_settings.is_high_sample_rate = (value == 1) ? 1 : 0;
        sendCommandToSTM32(STM32_CMD_SET_SR, (uint8_t*)&wave_settings.is_high_sample_rate);
        break;
    case BT_RECV_ID_TRIGGER:
        wave_settings.is_triggered = (value == 1) ? 1 : 0;
        sendCommandToSTM32(STM32_CMD_SET_TRIG, (uint8_t*)&wave_settings.is_triggered);
        break;
    case BT_RECV_ID_TRIGGER_VOLTAGE:
        wave_settings.trigger_voltage = constrain(value, 0, 255);
    default:
        if (DEV_DEBUG_FLAG)
            Serial.printf("[ERROR] Wave Set Unknown ID: %d\n", id);
        break;
    }

    return wave_settings;
}

// #include <math.h>
// #include <stdint.h>
// #include <stdlib.h>

// #define CHANNEL_SAMPLE_COUNT 2000 // 每个通道的采样点数
// #define MAX_LAG 200 // 最大允许的互相关滞后

// int16_t ch1[CHANNEL_SAMPLE_COUNT];
// int16_t ch2[CHANNEL_SAMPLE_COUNT];

// float calculatePhaseDiff(uint8_t* data, uint32_t sample_rate, float freq)
// {
//     const uint16_t* adc16 = (const uint16_t*)data;

//     // 分离两个通道，并去直流分量（假设你已解决内存分配，使用全局/静态变量等）
//     int16_t ch1[CHANNEL_SAMPLE_COUNT];
//     int16_t ch2[CHANNEL_SAMPLE_COUNT];
//     int32_t sum1 = 0, sum2 = 0;

//     for (int i = 0; i < CHANNEL_SAMPLE_COUNT; i++) {
//         ch1[i] = adc16[2 * i]; // 通道 1 在偶数索引
//         ch2[i] = adc16[2 * i + 1]; // 通道 2 在奇数索引
//         sum1 += ch1[i];
//         sum2 += ch2[i];
//     }

//     int16_t mean1 = sum1 / CHANNEL_SAMPLE_COUNT;
//     int16_t mean2 = sum2 / CHANNEL_SAMPLE_COUNT;

//     for (int i = 0; i < CHANNEL_SAMPLE_COUNT; i++) {
//         ch1[i] -= mean1;
//         ch2[i] -= mean2;
//     }

//     // 互相关计算
//     int best_lag = 0;
//     int64_t max_corr = INT64_MIN;

//     for (int lag = -MAX_LAG; lag <= MAX_LAG; lag++) {
//         int64_t corr = 0;
//         for (int i = 0; i < CHANNEL_SAMPLE_COUNT; i++) {
//             int j = i + lag;
//             if (j >= 0 && j < CHANNEL_SAMPLE_COUNT) {
//                 corr += (int32_t)ch1[i] * (int32_t)ch2[j];
//             }
//         }

//         if (corr > max_corr) {
//             max_corr = corr;
//             best_lag = lag;
//         }

//         vTaskDelay(0);
//     }

//     // 将 kHz 转换为 Hz，计算一个周期内的采样点数
//     float samples_per_period = (float)sample_rate / (freq * 1000.0f);

//     // 相位差（单位：度）
//     float phase_deg = ((float)best_lag / samples_per_period) * 360.0f;

//     // 归一化到 [-180°, 180°]
//     if (phase_deg < 0) {
//         phase_deg += 180.0f;
//     }
//     if (phase_deg > 180.0f) {
//         phase_deg = phase_deg - 360.0f;
//     }

//     return phase_deg;
// }

volatile uint32_t _ch1_time = 0;
volatile uint32_t _ch2_time = 0;
volatile bool _ch1_captured = false;
volatile bool _ch2_captured = false;

void IRAM_ATTR _isr_ch1()
{
    _ch1_time = micros();
    _ch1_captured = true;
    detachInterrupt(digitalPinToInterrupt(CH1_PHASE_DIFF_PIN)); // 禁止再次触发
}

void IRAM_ATTR _isr_ch2()
{
    _ch2_time = micros();
    _ch2_captured = true;
    detachInterrupt(digitalPinToInterrupt(CH2_PHASE_DIFF_PIN)); // 禁止再次触发
}

float last_phase_diff = 0.0f; // 用于存储上次计算的相位差

float calculatePhaseDiff(float frequency_hz)
{

    _ch1_captured = false;
    _ch2_captured = false;

    pinMode(CH1_PHASE_DIFF_PIN, INPUT);
    pinMode(CH2_PHASE_DIFF_PIN, INPUT);

    // 步骤 1：等待 CH1
    attachInterrupt(digitalPinToInterrupt(CH1_PHASE_DIFF_PIN), _isr_ch1, RISING);

    uint32_t start_time = micros();
    while (!_ch1_captured) {
        if (micros() - start_time > 200000) { // 超时
            detachInterrupt(digitalPinToInterrupt(CH1_PHASE_DIFF_PIN));
            return NAN;
        }
        yield();
    }

    // 步骤 2：等待 CH2
    attachInterrupt(digitalPinToInterrupt(CH2_PHASE_DIFF_PIN), _isr_ch2, RISING);
    start_time = micros();
    while (!_ch2_captured) {
        if (micros() - start_time > 200000) {
            detachInterrupt(digitalPinToInterrupt(CH2_PHASE_DIFF_PIN));
            return NAN;
        }
        yield();
    }

    // 计算时间差（保留正负符号）
    int32_t dt = (int32_t)(_ch2_time - _ch1_time);
    if (dt < -500000)
        dt += 1000000;
    if (dt > 500000)
        dt -= 1000000;

    float period_us = 1e6 / frequency_hz;
    float phase = (dt / period_us) * 360.0;

    phase -= 5.0f; // 调整相位差，假设有5us的延迟

    // 将相位归一到 [-180, +180)
    if (phase >= 180.0)
        phase -= 360.0;
    if (phase < -180.0)
        phase += 360.0;

    return phase;
}

void taskWaveSet(void* arg)
{
    for (;;) {
        xEventGroupWaitBits(data_ready_event_group, TASK_WAVE_SET_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (xSemaphoreTake(wave_set_mutex, portMAX_DELAY) == pdTRUE) {
            {
                wave_settings = updateWaveSettings(bt_recv_id, ((uint32_t*)bt_recv_buf)[0], wave_settings);
            }
            xSemaphoreGive(wave_set_mutex);
        }
    }
}
