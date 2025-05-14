import serial
import threading
from collections import deque

class IMUSerialReader:
    def __init__(self, port: str, baudrate: int = 115200, buffer_size: int = 5):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size

        self._ser = serial.Serial(self.port, self.baudrate, timeout=1)
        self._buffer = deque(maxlen=self.buffer_size)
        self._latest_sample = None
        self._new_data_available = False

        self._running = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)

    def start(self):
        """Start the reader thread."""
        if not self._running:
            self._running = True
            self._thread.start()

    def stop(self):
        """Stop the reader thread."""
        self._running = False
        if self._ser.is_open:
            self._ser.close()

    def get_latest(self, mark_as_read=True):
        """Return the most recent sample and optionally mark it as read."""
        sample = self._latest_sample
        if mark_as_read:
            self._new_data_available = False
        return sample
    
    def has_new_data(self):
        """Return True if a new sample has arrived since last check."""
        return self._new_data_available

    def get_buffer(self):
        """Return a copy of the rolling buffer."""
        return list(self._buffer)

    def _reader_loop(self):
        while self._running:
            try:
                line = self._ser.readline().decode('utf-8').strip()
                data = line.split("\t")
                if len(data) != 7:
                    continue  # Skip malformed lines

                t = float(data[0])
                gx, gy, gz = float(data[1]), float(data[2]), float(data[3])
                ax, ay, az = float(data[4]), float(data[5]), float(data[6])

                sample = {
                    "time": t,
                    "gyrox": gx,
                    "gyroy": gy,
                    "gyroz": gz,
                    "accelx": ax,
                    "accely": ay,
                    "accelz": az
                }
                self._buffer.append(sample)
                self._latest_sample = sample
                self._new_data_available = True


            except Exception as e:
                print(f"[IMUSerialReader] Error: {e}")
