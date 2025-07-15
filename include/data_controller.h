#pragma once

#include "main.h"

#define LOW_PASS_FILTER_RATE 0.2

struct ChannelData {
    uint16_t vpp[CHANNELS];
    uint16_t vpp_last[CHANNELS];
    uint16_t vmean[CHANNELS];
    float frequency[CHANNELS];
};

struct WaveInfo {
    String axis_info[2];
    String channel_info[CHANNELS];
    String mode_info;
};

struct WaveSettings {
    uint32_t x_range;      // X轴范围
    uint32_t y_range;      // Y轴范围
    uint32_t x_offset;     // X轴偏移
    uint32_t y_offset;     // Y轴偏移
    uint32_t is_high_voltage; // 高电压模式
    uint32_t is_high_sample_rate; // 高采样率模式
    uint32_t is_triggered; // 是否触发
    uint32_t trigger_voltage; // 触发电压
    uint32_t is_channel_inverted; // 是否通道反向
};


extern ChannelData calculate_results;
extern WaveInfo wave_info;
extern WaveSettings wave_settings;

ChannelData calculateInfo(uint8_t* recv_adc_buf, uint8_t* recv_freq_buf, ChannelData results);
WaveInfo generateWaveInfo(ChannelData calculate_results, WaveInfo wave_info, WaveSettings wave_settings, uint16_t recv_count);
float calculatePhaseDiff(uint8_t* data, uint32_t sample_rate, float freq);

