import serial
import time
import queue
import threading

class ESPSerialHandler:
    def __init__(self, port: str, baudrate: int = 115200, verbose: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        
        self._serial = None
        self._latest_sample = None
        self._has_new_data = False
        self.is_open = False
        
        self._realtime_lock = threading.Lock()
        self.realtime_msg = None
        self.control_queue = queue.Queue()

        self._thread = None
        self._running = False

        self.callback = None

    def open(self):
        self._serial = serial.Serial(self.port, self.baudrate, timeout=1)
        self.is_open = True

    def close(self):
        if self._serial:
            self._serial.close()
            self.is_open = False
            self._serial = None

    def start(self):
        self.open()
        if self._thread is None:
            self._running = True
            self._thread = threading.Thread(target=self.read_write_loop, daemon=True)
            self._thread.start()
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
            self._thread = None
        self.close()
    
    def set_callback(self, callback):
        self.callback = callback

    def set_realtime_msg(self, msg):
        with self._realtime_lock:
            self.realtime_msg = msg

    def send_control_msg(self, msg):
        self.control_queue.put(msg)
            
    def read_write_loop(self):#should be launched in a separate thread
        """
        Continuously reads from and writes to the serial port, managing connection and data flow.

        This method should be run in a separate thread. It attempts to connect to the serial port 
        if not already connected, and retries every 2 seconds in case of connection failure. Once 
        connected, it reads lines from the serial port, parses them, and triggers a callback with 
        the parsed data if available. It also handles writing messages from a control queue to the 
        serial port and sends any real-time messages immediately.

        If the connection is lost, it logs a warning, closes the serial port, and attempts to 
        reconnect.

        Note: This method is blocking and runs indefinitely until the program is interrupted.
        """

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
            
            try:
                while True:
                    msg = self.control_queue.get_nowait()
                    self._serial.write(msg)
            except queue.Empty:
                pass

            with self._realtime_lock:
                msg = self.realtime_msg

            if msg:
                try:
                    self._serial.write(msg)
                    self._serial.flush()
                    with self._realtime_lock:
                        self.realtime_msg = None
                except serial.SerialException as e:
                    print(f"[ERROR] Failed to send real-time msg: {e}")
    
    def _parse_serial_line(self, line):
        line = line.decode('utf-8').strip()
        try:
            topic, data = line.split(':', 1)
            return topic.strip(), data.strip().split(',')
        except ValueError:
            return None, None

def print_topic_data(topic, data):
    print(f"Topic: {topic}, Data: {data}")

if __name__ == "__main__":
    esp_serial = ESPSerialHandler('/dev/ttyUSB0', 115200, True)
    esp_serial.set_callback(print_topic_data)
    
    # Start background thread
    esp_serial.start()

    # Send test real-time message
    esp_serial.set_realtime_msg("<4\n".encode())

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
        esp_serial.stop()
