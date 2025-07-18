import ControlHolo
import zmq
import msgpack
import argparse

PUB_IPC_SOCKET_PATH = "/tmp/sub_esp_serial_handler.socket" # Default publisher path for IPC socket
SUB_IPC_SOCKET_PATH = "/tmp/pub_esp_serial_handler.socket" # Default subscriber path for IPC socket

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

def main():
    args = parse_args()
    context, pub_socket, sub_socket = setup_zmq_socket(args)

    while(True):
        input("Press enter to send")
        topic = "ESP/CTRL"
        data = "<25:0.1,0.1,0.1\n"
        pub_socket.send_multipart([topic.encode(), msgpack.packb(data)])

if __name__ == "__main__":
    main()