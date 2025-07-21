import time
import os
import csv

class Capture:
    def __init__(self, path: str, filename: str = "capture"):
        if not path:
            raise ValueError("Path cannot be empty")
        self.path = os.path.abspath(path)  # Ensure absolute path
        self.startTime = time.time()
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        self.file_path = os.path.join(self.path, f"{filename}_{time.strftime('%H-%M')}.csv")
        self.file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['Timestamp', 'Data'])

    def capture(self, data: str):
        timestamp = time.time() - self.startTime
        self.writer.writerow([round(timestamp, 3), data])
        self.file.flush()

    def close(self):
        self.file.close()