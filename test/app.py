import time
import struct
import serial  # pip install pyserial

# 修改为你的ESP32蓝牙串口端口
BLUETOOTH_PORT = 'COM5'  # Windows下通常为COMx
BAUDRATE = 921600

# 协议常量
HEADER1 = 0xFF
HEADER2 = 0xFE
BT_RECV_ID_X_RANGE = 0x01
BT_RECV_ID_Y_RANGE = 0x02
BT_RECV_ID_X_OFFSET = 0x03
BT_RECV_ID_Y_OFFSET = 0x04
BT_RECV_ID_VOLTAGE = 0x05

def send_param(ser, param_id, value):
    # 组包：FF FE ID [4字节数据]
    packet = struct.pack('<3B I', HEADER1, HEADER2, param_id, value)
    ser.write(packet)
    print(f"Sent: {[hex(b) for b in packet]}")

def main():
    ser = serial.Serial(BLUETOOTH_PORT, BAUDRATE, timeout=1)
    print("Connected to ESP32 Bluetooth serial.")

    while True:
        print("\n1. 设置X范围\n2. 设置Y范围\n3. 设置X偏移\n4. 设置Y偏移\n5. 设置高/低电压\n0. 退出")
        choice = input("选择操作: ").strip()
        if choice == '0':
            break
        elif choice == '1':
            val = int(input("输入X范围 (1~8000): "))
            send_param(ser, BT_RECV_ID_X_RANGE, val)
        elif choice == '2':
            val = int(input("输入Y范围 (16~1024): "))
            send_param(ser, BT_RECV_ID_Y_RANGE, val)
        elif choice == '3':
            val = int(input("输入X偏移 (0~7040): "))
            send_param(ser, BT_RECV_ID_X_OFFSET, val)
        elif choice == '4':
            val = int(input("输入Y偏移 (0~1024): "))
            send_param(ser, BT_RECV_ID_Y_OFFSET, val)
        elif choice == '5':
            val = int(input("输入电压模式 (0=低, 1=高): "))
            send_param(ser, BT_RECV_ID_VOLTAGE, val)
        else:
            print("无效选择")
        time.sleep(0.2)

    ser.close()
    print("已断开连接。")

if __name__ == '__main__':
    main()