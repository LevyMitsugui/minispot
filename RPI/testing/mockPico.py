import serial
import time
import random

if __name__ == "__main__":

    ser = serial.Serial('/dev/pts/15', 115200)
    while True:
        topic = "test_topic"
        currTime = time.time() #time from code initialization
        value = random.randint(0, 100)  # randomly generate a value between 0 and 100
        float_value = random.uniform(0, 100)  # randomly generate a float between 0 and 100
        mensage = f"{topic}:{currTime},{value},{float_value}\n"
        ser.write(mensage.encode())

        print(mensage)

        topic = "test_topic2"
        currTime = time.time()
        value = random.randint(100, 200)
        float_value = random.uniform(100, 200)
        mensage = f"{topic}:{currTime},{value},{float_value}\n"
        ser.write(mensage.encode())

        print(mensage)

        topic = "pico/log"
        currTime = time.time()
        value = "This is a log example"
        mensage = f"{topic}:{currTime},{value}\n"
        ser.write(mensage.encode())

        print(mensage)
        time.sleep(1)