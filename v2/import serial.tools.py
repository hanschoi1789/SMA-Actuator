import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
for port in ports:
    print(f"포트명: {port.device} / 장치 설명: {port.description}")