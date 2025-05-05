import serial
import time

class ESPSerialHandler:
    def __init__(self, port: str, baudrate: int = 115200, verbose: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        
        self._serial = None
        self._latest_sample = None
        self._has_new_data = False
        self.is_open = False

        self.callback = None

    def open(self):
        self._serial = serial.Serial(self.port, self.baudrate, timeout=1)
        self.is_open = True

    def close(self):
        if self._serial:
            self._serial.close()
            self.is_open = False
            self._serial = None
    
    def set_callback(self, callback):
        self.callback = callback
    
    def read_loop(self):
        while True:
            if not self.is_open:
                try:
                    self.open()
                    if self.verbose:
                        print("[INFO] Connected to serial port.")
                except (serial.SerialException, OSError) as e:
                    print(f"[WARN] Could not open serial port: {e}")
                    time.sleep(2)
                    continue  # Retry the whole loop

            try:
                line = self._serial.readline()
                if line:
                    self._latest_sample = line
                    self._has_new_data = True
                    topic, data = self._parse_serial_line(line)
                    if topic and data and self.verbose:
                        print(f"Topic: {topic}, Data: {data}")
                    elif self.verbose:
                        print(f"Failed to parse line: {line}")
                    if self.callback:
                        self.callback(topic, data)
            except (serial.SerialException, OSError) as e:
                print(f"[WARN] Lost connection to ESP: {e}")
                print("[INFO] Retrying in 2 seconds...")
                self.is_open = False
                try:
                    self._serial.close()
                except:
                    pass
                time.sleep(2)
    
    def send_command(self, command: str):
        if not self.is_open:
            raise RuntimeError("Serial port is not open.")
        
        self._serial.write(command.encode('utf-8'))
        self._serial.flush()
