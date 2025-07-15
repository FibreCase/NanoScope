#include "main.h"
#include "bluetooth_task.h"
#include "data_serial_task.h"
#include "tft_task.h"

EventGroupHandle_t data_ready_event_group;

void setup()
{
    data_ready_event_group = xEventGroupCreate();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(CH1_PHASE_DIFF_PIN, INPUT);
    pinMode(CH2_PHASE_DIFF_PIN, INPUT);

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


