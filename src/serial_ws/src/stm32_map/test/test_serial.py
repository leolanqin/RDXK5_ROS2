#!/usr/bin/env python3

import time
import serial
import signal
import sys

# UART 设备路径（视具体设备名称而定，可能是 /dev/ttyS1、ttyUART1、ttyAMA1 等）
UART_DEVICE = "/dev/ttyS1"
BAUDRATE = 115200
# TEST_BYTES = bytes.fromhex("AA555")  # 十六进制发送内容
msg = "125"

# 信号处理：支持 Ctrl+C 退出
def signal_handler(sig, frame):
    print("\n[INFO] 测试结束，退出程序")
    sys.exit(0)

def test_uart():
    try:
        ser = serial.Serial(UART_DEVICE, BAUDRATE, timeout=1)
        print(f"[INFO] 成功打开串口 {UART_DEVICE} @ {BAUDRATE}bps")
    except Exception as e:
        print(f"[ERROR] 无法打开串口: {e}")
        return -1

    while True:
        ser.write(msg.encode('ascii'))
        # print("[TX] 发送:", TEST_BYTES.hex().upper())
        print(f"[TX] Sent: {msg}")

        # rx_data = ser.read(len(TEST_BYTES))
        # print("[RX] 接收:", rx_data.hex().upper() if rx_data else "无响应")

        time.sleep(5)

    ser.close()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    test_uart()
