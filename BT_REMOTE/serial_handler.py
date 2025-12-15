import serial

class SerialHandler:
    def __init__(self, port, baudrate=9600, timeout=1):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        except Exception as e:
            print(f"Failed to open serial port {port}: {e}")
            self.ser = None

    def send_command(self, command):
        if self.ser and  self.ser.is_open:
            self.ser.write(command.encode())
            print(f"Sent command to serial device: {command}")
        else:
            print("Serial port is not open.")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")