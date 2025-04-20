import serial

class PICOSerialHandler:
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
        if not self.is_open:
            raise RuntimeError("Serial port is not open.")

        print("Reading data from serial port...")
        while self.is_open:
            line = self._serial.readline()
            if line:
                self._latest_sample = line
                self._has_new_data = True
                topic, data = self._parse_serial_line(line)
                if topic and data:
                    print(f"Topic: {topic}, Data: {data}")
                else:
                    print(f"Failed to parse line: {line}")
                if self.callback:
                    self.callback(topic, data)

    def _parse_serial_line(self, line: bytes):
        if not line:
            return None, None

        try:
            raw_topic, raw_data = line.split(b':', 1)
        except ValueError:
            if getattr(self, "verbose", False):
                print(f"[Parse Error] No topic/data separator found: {line}")
            return None, None

        topic = raw_topic.decode().strip()
        data_items = raw_data.strip().split(b',')

        parsed_values = []
        for item in data_items:
            try:
                parsed_values.append(int(item))
            except ValueError:
                try:
                    parsed_values.append(float(item))
                except ValueError:
                    if getattr(self, "verbose", False):
                        print(f"[Parse Warning] Could not convert value: {item}")
                    continue  # Skip unparseable values

        if not parsed_values:
            if getattr(self, "verbose", False):
                print(f"[Parse Error] No valid values parsed from: {line}")
            return None, None

        return topic, parsed_values

    
    def send_command(self, command: str):
        if not self.is_open:
            raise RuntimeError("Serial port is not open.")
        
        self._serial.write(command.encode('utf-8'))
        self._serial.flush()

    def get_latest_sample(self):
        if self._has_new_data:
            self._has_new_data = False
            topic, data = self._parse_serial_line(self._latest_sample)
            return topic, data
        else:
            print("No new data available.")
        return None
    