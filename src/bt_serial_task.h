#include "BluetoothSerial.h"

#define BT_RECV_ID_X_RANGE 0x01
#define BT_RECV_ID_Y_RANGE 0x02
#define BT_RECV_ID_X_OFFSET 0x03
#define BT_RECV_ID_Y_OFFSET 0x04
#define BT_RECV_ID_VOLTAGE 0x05
#define BT_RECV_ID_SAMPLE_RATE 0x06

#define BT_FSM_IDLE 1
#define BT_FSM_HEADER 2
#define BT_FSM_ID 3
#define BT_FSM_DATA 4

extern BluetoothSerial SerialBT;
extern SemaphoreHandle_t bt_fsm_mutex;

extern uint8_t bt_recv_id;
extern uint8_t bt_recv_count;
extern uint8_t bt_recv_buf[4];
extern uint8_t bt_status;

void taskBluetoothRecvInit();
void bluetoothHeartbeat();
void taskBluetoothRecv(void* arg);