import ESPSerialHandler
import serial
import zmq
import msgpack
import os
import sys
import argparse
import socket as pysocket
import time

PUB_IPC_SOCKET_PATH = "/tmp/pub_esp_serial_handler.socket" # Default publisher path for IPC socket
SUB_IPC_SOCKET_PATH = "/tmp/sub_esp_serial_handler.socket" # Default subscriber path for IPC socket

def parse_args():
    parser = argparse.ArgumentParser(description="ESP Serial Handler")
    parser.add_argument("-p", "--port", type=str, required=True, help="Serial port to connect to")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="Baudrate for the serial connection")
    parser.add_argument("--pub-socket", "--socket_path_pub", type=str, default=PUB_IPC_SOCKET_PATH, help="Path to the IPC output socket")
    parser.add_argument("--sub-socket", "--socket_path_sub", type=str, default=SUB_IPC_SOCKET_PATH, help="Path to the IPC input socket")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")
    args = parser.parse_args()

    if not os.path.exists(args.port):
        print(f"Error: The specified port '{args.port}' does not exist.")
        sys.exit(1)

    return args

def setup_zmq_socket(pub_socket_path, sub_socket_path, sub_filters=[""], verbose=False):
    if os.path.exists(pub_socket_path):
        os.remove(pub_socket_path)

    context = zmq.Context()

    # Set up PUB socket (we bind to this one)
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(f"ipc://{pub_socket_path}")

    # Set up SUB socket (we connect to this one)
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(f"ipc://{sub_socket_path}")
    for topic_filter in sub_filters:
        sub_socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)

    if verbose:
        print(f"[INFO] PUB socket bound to ipc://{pub_socket_path}")
        print(f"[INFO] SUB socket connected to ipc://{sub_socket_path} with filters: {sub_filters}")

    return context, pub_socket, sub_socket


def publish_data(socket, verbose=False):
    def _callback(topic: str, data: list):
        try:
            if verbose:
                print(f"Publishing to '{topic}': {data}")
            socket.send_multipart([topic.encode(), msgpack.packb(data)])
        except zmq.ZMQError as e:
            print(f"[ZMQ Error] {e}")
    return _callback


def main():
    try:
        args = parse_args()
        handler = ESPSerialHandler.ESPSerialHandler(args.port, args.baudrate, verbose=args.verbose)
        context, pub_socket, sub_socket = setup_zmq_socket(args.pub_socket, args.sub_socket, sub_filters=["CMD/ESP"], verbose=args.verbose)

        handler.set_callback(publish_data(pub_socket, verbose=args.verbose))
        handler.start()

        #TODO Program sending realtime and control messages from sub_socket


        while True:
            time.sleep(1)



    except serial.SerialException as e:
        print(f"[ERROR] Serial port error: {e}")
        sys.exit(1)

    except zmq.ZMQError as e:
        print(f"[ERROR] ZMQ error: {e}")
        sys.exit(1)

    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        sys.exit(1)
    
    except KeyboardInterrupt:
        print("[INFO] Exiting...")

    finally:
        if handler.is_open:
            handler.stop()
        if context:
            context.term()
        if socket:
            socket.close()
        if os.path.exists(args.socket_path):
            os.remove(args.socket_path)

        if args.verbose:
            print("[INFO] Cleaned up resources.")
        sys.exit(0)


if __name__ == "__main__":
    main()