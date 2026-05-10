#!/usr/bin/env python3
"""
串口调试工具 - 用于连接和测试板子串口通信
"""

import serial
import time
import sys

def test_serial_port(port, baudrate=115200, timeout=2):
    """测试串口连接并发送/接收数据"""
    try:
        print(f"正在连接 {port}，波特率: {baudrate}...")
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"✓ 串口连接成功！")
        print(f"端口信息: {ser}")
        print("-" * 50)
        
        # 发送测试数据
        test_data = "hello\r\n"
        print(f"发送测试数据: {repr(test_data)}")
        ser.write(test_data.encode())
        
        # 等待并读取响应
        print("等待响应...")
        time.sleep(0.5)
        
        response = ser.read_all()
        if response:
            print(f"✓ 收到响应: {response.decode(errors='ignore')}")
        else:
            print("未收到响应")
        
        # 保持连接以进行交互
        print("\n进入交互模式 (按 Ctrl+C 退出):")
        print("-" * 50)
        
        try:
            while True:
                # 读取来自串口的数据
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    print(f"← {data.decode(errors='ignore')}", end='')
                
                # 读取用户输入
                user_input = input()
                if user_input:
                    ser.write((user_input + "\r\n").encode())
                    
        except KeyboardInterrupt:
            print("\n\n退出交互模式")
        
        ser.close()
        print("串口已关闭")
        return True
        
    except serial.SerialException as e:
        print(f"✗ 串口错误: {e}")
        return False
    except Exception as e:
        print(f"✗ 错误: {e}")
        return False

def list_available_ports():
    """列出可用的串口"""
    import os
    ports = []
    
    # 检查 /dev/ttyS* 
    for i in range(8):
        port = f"/dev/ttyS{i}"
        if os.path.exists(port):
            ports.append(port)
    
    # 检查 /dev/ttyUSB*
    for i in range(4):
        port = f"/dev/ttyUSB{i}"
        if os.path.exists(port):
            ports.append(port)
    
    # 检查 /dev/ttyACM*
    for i in range(4):
        port = f"/dev/ttyACM{i}"
        if os.path.exists(port):
            ports.append(port)
    
    return ports

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
        baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    else:
        # 列出可用端口
        ports = list_available_ports()
        print("可用的串口:")
        for p in ports:
            print(f"  {p}")
        
        if not ports:
            print("\n未找到可用的串口设备！")
            print("请检查:")
            print("  1. 板子是否已用串口线连接")
            print("  2. USB转串口设备是否已识别")
            sys.exit(1)
        
        # 使用第一个可用端口
        port = ports[0]
        baudrate = 115200
        print(f"\n使用第一个可用端口: {port}")
    
    test_serial_port(port, baudrate)
