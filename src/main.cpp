#include "main.h"
#include "bt_serial_task.h"
#include "data_serial_task.h"
#include "tft_task.h"

EventGroupHandle_t data_ready_event_group;

void setup()
{
    data_ready_event_group = xEventGroupCreate();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Serial.setRxBufferSize(8000);
    Serial.begin(921600);
    Serial.flush(false);

    digitalWrite(2, LOW);

    taskDataSerialRecvInit();
    taskTftInit();
    taskBluetoothRecvInit();

    Serial.println("ESP32 Started - Ready to receive data");
    digitalWrite(2, HIGH);
}

void loop()
{
    bluetoothHeartbeat();
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * ESP32指令控制STM32
 */

#define STM32_CMD_SET_SR 0x01
#define STM32_CMD_SET_TRIG 0x02
#define STM32_CMD_SET_TRIG_VOLTAGE 0x03

void sendCommandToSTM32(uint8_t command, uint8_t* data)
{
    dataSerial.write(0xFF); // Header
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
