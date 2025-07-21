import ControlHolo
from Capture import Capture
import zmq
import msgpack
import argparse

PUB_IPC_SOCKET_PATH = "/tmp/sub_esp_serial_handler.socket" # Default publisher path for IPC socket
SUB_IPC_SOCKET_PATH = "/tmp/pub_esp_serial_handler.socket" # Default subscriber path for IPC socket

STD_TOPIC = "ESP/CTRL" # Default topic for ESP32 control messages
SPEED_UPDTE_CMD = "25" # Command to update speed in the ESP32	
#CAPTURE_PATH = "/mnt/c/Users/Levy/Documents/GitHub/minispot/RPI/logs" # Default path for Capture logs
CAPTURE_PATH = "/home/Levy/minispot/RPI/logs"

MAXV = 1.0
MAXW = 1.0

def parse_args():
    parser = argparse.ArgumentParser(description="Read and publish data from Pico Serial Handler.")
    parser.add_argument("--pub-socket", type=str, default=PUB_IPC_SOCKET_PATH, help="Path to the IPC output socket")
    parser.add_argument("--sub-socket", type=str, default=SUB_IPC_SOCKET_PATH, help="Path to the IPC or TCP input socket")
    parser.add_argument("--sub-port", type=int, help="Port for the Input Socket")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")

    args = parser.parse_args()

    return args

def setup_zmq_socket(args):
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(f"ipc://{args.pub_socket}")

    sub_socket = context.socket(zmq.SUB)
    sub_socket.setsockopt(zmq.SUBSCRIBE, b"")
    if args.sub_port:
        sub_socket.connect(f"tcp://{args.sub_socket}:{args.sub_port}")
    else:
        sub_socket.connect(f"ipc://{args.sub_socket}")

    return context, pub_socket, sub_socket

def send_speeds(pub_socket, robotV, robotVn, robotW):
    topic = STD_TOPIC
    data  = "<" + SPEED_UPDTE_CMD + f":{robotV},{robotVn},{robotW}\n"
    pub_socket.send_multipart([topic.encode(), msgpack.packb(data)])

def send_command(pub_socket, cmd: int, string = ""):
    topic = STD_TOPIC
    if string :
        data = "<" + str(cmd) + ":" + string
    else:
        data = "<" + str(cmd)
    
    pub_socket.send_multipart([topic.encode(), msgpack.packb(data)])
        

def main():
    args = parse_args()
    cap = Capture(CAPTURE_PATH, "last_test")
    context, pub_socket, sub_socket = setup_zmq_socket(args)
    ctrl = ControlHolo.ControlHolo(pub_socket, MAXV, MAXW, verbose=args.verbose)
    ctrl.update()  # Initial update to set speeds to zero

    while True:
        try:
            topic, data = sub_socket.recv_multipart()
            data = str(msgpack.unpackb(data)).split()
            
            #userInput = input("Enter string\n")
            if data[0] == "quit":
                cap.close()
                break
            elif data[0] == "start":
                if args.verbose: print("[INFO] Starting Walking state machine")
                ctrl.stopRobot()
                send_command(pub_socket, 23)
                data = None
            elif data[0] == "stop" or data[0] == "s":
                if args.verbose: print("[INFO] Stop robot CMD")
                ctrl.stopRobot()
            elif data[0] == "bfw":
                if args.verbose: print("[INFO] Blind Forward CMD")
                ctrl.blindForward(float(data[1]))
            elif data[0] == "bbw":
                if args.verbose: print("[INFO] Blind Backward CMD")
                ctrl.blindBackward(float(data[1])) 
            elif data[0] == "br":
                if args.verbose: print("[INFO] Blind Right CMD")
                ctrl.blindRight(float(data[1]))
            elif data[0] == "bl":
                if args.verbose: print("[INFO] Blind Left CMD")
                ctrl.blindLeft(float(data[1]))
            elif data[0] == "btr":
                if args.verbose: print("[INFO] Blind Turn Right CMD")
                ctrl.blindTurnRight(float(data[1]))
            elif data[0] == "btl":
                if args.verbose: print("[INFO] Blind Turn Left CMD")
                ctrl.blindTurnLeft(float(data[1]))


            if data:
                ctrl.update()

        except KeyboardInterrupt:
            print("Exiting...")
            cap.close()
            break
            



if __name__ == "__main__":
    main()