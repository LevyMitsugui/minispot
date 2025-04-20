import sys
import os
import zmq
import msgpack
from PICOSerialHandler import PICOSerialHandler


def parse_args():
    if len(sys.argv) < 3:
        print("Usage: python main.py <port> <baudrate>")
        sys.exit(1)
    port = sys.argv[1]
    baudrate = int(sys.argv[2])
    if not os.path.exists(port):
        print(f"Error: The specified port '{port}' does not exist.")
        sys.exit(1)
    return port, baudrate


def setup_zmq_socket(socket_path="/tmp/pico_serial_handler.socket"):
    if os.path.exists(socket_path):
        os.remove(socket_path)
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"ipc://{socket_path}")
    return context, socket


def publish_data(socket):
    def _callback(topic: str, data: list):
        try:
            print(f"Publishing to '{topic}': {data}")
            socket.send_multipart([topic.encode(), msgpack.packb(data)])
        except zmq.ZMQError as e:
            print(f"[ZMQ Error] {e}")
    return _callback


def main():
    port, baudrate = parse_args()
    handler = PICOSerialHandler(port, baudrate)
    context, socket = setup_zmq_socket()

    try:
        print("Starting PICOSerialHandler...")
        handler.open()
        handler.set_callback(publish_data(socket))
        handler.read_loop()

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")

    except Exception as e:
        print(f"[Error] {e}")

    finally:
        print("Cleaning up...")
        if handler.is_open:
            handler.close()
        socket.close()
        context.term()
        print("Cleanup complete. Goodbye!")


if __name__ == "__main__":
    main()
