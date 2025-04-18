from IMUSerialReader import IMUSerialReader
import math
import zmq
import msgpack
import os


socket_path = "/tmp/RollPitch.socket"
if os.path.exists(socket_path):
    os.remove(socket_path)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(f"ipc://{socket_path}")

def calibrate_imu_row_pitch(samples = 1000): #TODO make a useful calibration routine, for now the values from the IMU are used as is
    pass

def compute_roll_pitch(ax, ay, az):
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
    return roll, pitch

def compute_robot_roll_pitch(ax, ay, az):
    pitch = math.atan2(ax, math.sqrt(ay**2 + az**2))
    roll = math.atan2(-ay, az)
    return roll, pitch

if __name__ == "__main__":
    import time

    reader = IMUSerialReader(port='/dev/ttyACM0', baudrate=230400, buffer_size=5)
    reader.start()

    try:
        while True:
            if reader.has_new_data():
                sample = reader.get_latest()
                roll, pitch = compute_robot_roll_pitch(sample['accelx'], sample['accely'], sample['accelz'])
                msg = {
                    "roll": roll,
                    "pitch": pitch
                }
                socket.send(msgpack.packb(msg))
                print("Roll: {:.2f}".format(roll*180/math.pi), "Pitch: {:.2f}".format(pitch*180/math.pi), "Sample:", sample)
            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\n[Main] Keyboard interrupt received. Exiting...")

    finally:
        reader.stop()
        print("[Main] Reader stopped.")

