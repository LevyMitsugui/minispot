import serial
import time

# Adjust to match your ESP32's serial port
SERIAL_PORT = '/dev/ttyUSB0'  # On Windows it might be 'COM4' or similar
BAUD_RATE = 115200

# One test message: Gait flag = 'F', Leg ID = 0 (FL), 3 joints with 19 frames
# Using small test angles (can be all the same for simplicity)
angles = [30] * 19
joints = ",".join(map(str, angles))

# FL message format: F,0,<joint1>,<joint2>,<joint3>
test_message = f"F,0,{joints},{joints},{joints}\n"

# Stop message (noop or acknowledgment)
stop_message = "A\n"

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print("Sending gait test message...")
        ser.write(test_message.encode())

        # Wait a few seconds for it to play
        time.sleep(3)

        print("Sending stop message...")
        ser.write(stop_message.encode())

if __name__ == "__main__":
    main()
