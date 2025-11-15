import socket
import pygame
import time
import xml.etree.ElementTree as ET
import argparse

def generate_xml(axes, buttons):
    root = ET.Element("ControllerState")

    axes_elem = ET.SubElement(root, "Axes")
    for i, value in enumerate(axes):
        axis_elem = ET.SubElement(axes_elem, f"Axis{i}")
        axis_elem.text = str(round(value, 4))  # 保留4位小数

    buttons_elem = ET.SubElement(root, "Buttons")
    for i, state in enumerate(buttons):
        button_elem = ET.SubElement(buttons_elem, f"Button{i}")
        button_elem.text = str(state)

    return ET.tostring(root, encoding="unicode")

def send_controller_data(server_ip, server_port):
    update_interval = 0.1  # 每0.1秒发送一次

    # 初始化pygame手柄
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("未检测到手柄设备！")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"已连接手柄: {joystick.get_name()}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((server_ip, server_port))
            print(f"已连接到服务器 {server_ip}:{server_port}")
            print(f"发送摇杆信息......")

            while True:
                pygame.event.pump()  # 刷新事件队列

                # 获取8个通道的摇杆轴数据
                axes = [-joystick.get_axis(i) for i in range(min(8, joystick.get_numaxes()))]

                # 获取D-Pad（方向键）数据
                hat_x, hat_y = joystick.get_hat(0)  # 通常只有一个Hat，索引0
                axes.append(float(-hat_x))
                axes.append(float(hat_y))
                
                # 获取10个按键状态（0或1）
                buttons = [joystick.get_button(i) for i in range(min(11, joystick.get_numbuttons()))]

                # 生成XML字符串
                xml_data = generate_xml(axes, buttons)

                # 发送
                s.sendall(xml_data.encode('utf-8'))
                # print(f"发送: {xml_data}")

                time.sleep(update_interval)

        except Exception as e:
            print(f"连接错误: {str(e)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="发送手柄数据到TCP服务器")
    parser.add_argument("--server_ip", type=str, default="127.0.0.1", help="服务器IP地址，默认127.0.0.1")
    parser.add_argument("--server_port", type=int, default=12345, help="服务器端口，默认12345")
    args = parser.parse_args()

    send_controller_data(args.server_ip, args.server_port)
