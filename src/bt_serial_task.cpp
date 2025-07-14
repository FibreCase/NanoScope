#include "bt_serial_task.h"
#include "main.h"

BluetoothSerial SerialBT;
SemaphoreHandle_t bt_fsm_mutex;

uint8_t bt_recv_id = 0;
uint8_t bt_recv_count = 0;
uint8_t bt_recv_buf[4] = { 0 };
uint8_t bt_status = BT_FSM_IDLE;

void bt_fsm(uint8_t buf);

void taskBluetoothRecvInit()
{
    bt_fsm_mutex = xSemaphoreCreateMutex();
    SerialBT.begin("Fibre_ESP32_OSC");
    xTaskCreate(taskBluetoothRecv, "BluetoothRecv", 4096, NULL, 2, NULL);
}

void bluetoothHeartbeat()
{
    SerialBT.write(0xF0);
}

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

void bt_fsm(uint8_t buf)
{
    switch (bt_status) {
    case BT_FSM_IDLE:
        if (buf == 0xFF) {
            bt_status = BT_FSM_HEADER;
        } else {
            if (DEV_DEBUG_FLAG)
                Serial.printf("BT Header Not Found ( 0x%02X )\n", buf); // vsnprintf
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
        if (buf == BT_RECV_ID_X_RANGE || buf == BT_RECV_ID_Y_RANGE || buf == BT_RECV_ID_X_OFFSET || buf == BT_RECV_ID_Y_OFFSET || buf == BT_RECV_ID_VOLTAGE || buf == BT_RECV_ID_SAMPLE_RATE) {
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
            SerialBT.write(0xFA);
            xEventGroupSetBits(data_ready_event_group, TASK_WAVE_SET_EVENT_BIT);
        }
        break;
    }
}