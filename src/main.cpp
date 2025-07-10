#include "Arduino.h"
#include "TFT_eSPI.h"

#define DEV_DEBUG_FLAG 1

HardwareSerial dataSerial(1);
TFT_eSPI tft = TFT_eSPI();

SemaphoreHandle_t fsm_mutex;
SemaphoreHandle_t draw_binary;

void drawWave(uint8_t* data, size_t len, int y_range);

void taskDrawWave(void* arg);
void serialRecvTask(void* arg);

void setup()
{
    fsm_mutex = xSemaphoreCreateMutex();
    draw_binary = xSemaphoreCreateBinary();

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
    delay(1000);

    dataSerial.flush(false);
    Serial.flush(false);

    xTaskCreate(taskDrawWave, "DrawWave", 4096, NULL, 2, NULL);
    xTaskCreate(serialRecvTask, "SerialRecv", 4096, NULL, 1, NULL);

    Serial.println("ESP32 Started - Ready to receive data");
    dataSerial.write(0x01);

    digitalWrite(2, HIGH);
}

void loop()
{
}

static uint8_t cur_buf[1000];
void fsm(uint8_t buf);

#define SERIAL_FSM_IDLE 1
#define SERIAL_FSM_HEADER 2
#define SERIAL_FSM_DATA 3

#define RECV_DATA_SIZE 8000
#define TIMEOUT_MS 1000 // 超时时间1秒

uint8_t status = SERIAL_FSM_IDLE;
uint8_t recv_buf[RECV_DATA_SIZE];
uint16_t* recv_num = (uint16_t*)recv_buf;
uint16_t recv_index = 0;
uint8_t recv_count = 0;
unsigned long last_recv_time = 0;
uint8_t error_count = 0; 

void serialRecvTask(void* arg)
{
    while (1) {
        if (status != SERIAL_FSM_IDLE && millis() - last_recv_time > TIMEOUT_MS) {
            Serial.println("Timeout - Reset to IDLE");
            status = SERIAL_FSM_IDLE;
            recv_index = 0;
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
            recv_index = 0;
            status = SERIAL_FSM_DATA;
            error_count = 0;
        } else {
            if (DEV_DEBUG_FLAG)
                Serial.printf("Header Error ( 0x%02X )\n", buf);
            status = SERIAL_FSM_IDLE;
            error_count++;
        }
        break;

    case SERIAL_FSM_DATA:
        recv_buf[recv_index++] = buf;
        if (recv_index >= RECV_DATA_SIZE) {
            if (DEV_DEBUG_FLAG)
                Serial.printf("Frame %d | %d | %d\n", recv_count++, recv_num[0], recv_num[1]);
            xSemaphoreGive(draw_binary);
            status = SERIAL_FSM_IDLE;
            error_count = 0;
        }
        break;

    default:
        if (DEV_DEBUG_FLAG)
            Serial.println("FSM Error - Reset to IDLE");
        status = SERIAL_FSM_IDLE;
        recv_index = 0;
        error_count = 0;
        break;
    }
}

size_t waveLen = 8000;
int waveYRange = 128;

void taskDrawWave(void* arg)
{
    while (1) {
        if (xSemaphoreTake(draw_binary, portMAX_DELAY) == pdTRUE) {
            drawWave(recv_buf, waveLen, waveYRange);
        }
    }
}

const int screen_w = 240;
const int screen_h = 128;
const int adc_max = 4095;

void drawWave(uint8_t* data, size_t len, int y_range)
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
            uint16_t ch1 = data[i * 4] | (data[i * 4 + 1] << 8);
            uint16_t ch2 = data[i * 4 + 2] | (data[i * 4 + 3] << 8);
            sum_ch1 += ch1;
            sum_ch2 += ch2;
            count++;
        }

        if (count == 0)
            continue;

        int avg_ch1 = sum_ch1 / count;
        int avg_ch2 = sum_ch2 / count;

        int y_ch1 = screen_h - (avg_ch1 * y_range / adc_max);
        int y_ch2 = screen_h - (avg_ch2 * y_range / adc_max);

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
