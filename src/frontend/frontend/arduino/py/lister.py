import serial.tools.list_ports

def find_arduino_port(arduino_vid='2341', arduino_pid='0043'):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid is not None and port.pid is not None:
            if f'{port.vid:04x}' == arduino_vid and f'{port.pid:04x}' == arduino_pid:
                return port.device
    return None

print(find_arduino_port())
