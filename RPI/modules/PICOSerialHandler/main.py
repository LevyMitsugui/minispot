import sys
import os
import zmq
import msgpack
import argparse
from PICOSerialHandler import PICOSerialHandler


def parse_args():
    parser = argparse.ArgumentParser(description="Read and publish data from Pico Serial Handler.")
    parser.add_argument("port", help="Serial port to use (e.g., /dev/ttyACM0)")
    parser.add_argument("baudrate", type=int, help="Baudrate for serial communication")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")

    args = parser.parse_args()

    if not os.path.exists(args.port):
        print(f"Error: The specified port '{args.port}' does not exist.")
        sys.exit(1)

    return args


def setup_zmq_socket(socket_path="/tmp/pico_serial_handler.socket"):
    if os.path.exists(socket_path):
        os.remove(socket_path)
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"ipc://{socket_path}")
    return context, socket


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
    args = parse_args()
    handler = PICOSerialHandler(args.port, args.baudrate, verbose=args.verbose)
    context, socket = setup_zmq_socket()

    try:
        if args.verbose:
            print("Starting PICOSerialHandler...")

        handler.open()
        handler.set_callback(publish_data(socket, verbose=args.verbose))
        handler.read_loop()

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")

    except Exception as e:
        print(f"[Error] {e}")

    finally:
        if args.verbose:
            print("Cleaning up...")
        if handler.is_open:
            handler.close()
        socket.close()
        context.term()
        if args.verbose:
            print("Cleanup complete. Goodbye!")


if __name__ == "__main__":
    main()
