from JoystickGUI import JoystickGUI
import tkinter as tk
import zmq
import msgpack
import argparse

def parse_args():
    parser=argparse.ArgumentParser(description="Static Pose Command GUI")
    parser.add_argument("ip", help="IP address", type=str)
    parser.add_argument("port", help="Port number", type=int)
    parser.add_argument("-oc", "--onchange", action="store_true", help="Only send data when changed")
    args = parser.parse_args()
    return args

def setup_zmq_socket(bind_ip="0.0.0.0", port=5555):
    try:
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.bind(f"tcp://{bind_ip}:{port}")
    except Exception as e:
        print(f"[ZMQ Error] {e}")
        raise
    return context, socket

def publish_data(socket):
    def callback(data):
        try:
            for key, value in data.items():
                topic = f"command/{key}"
                socket.send_multipart([
                    topic.encode("utf-8"),
                    msgpack.packb(value)
                ])
        except zmq.ZMQError as e:
            print(f"[ZMQ Error] {e}")
    return callback

def main():
    """
    Main entry point of the script. Parses command line arguments, sets up a ZMQ socket at the specified IP and port, 
    and starts a Tkinter event loop with a JoystickGUI widget. The widget is configured to only send data when
    changed if the --onchange flag is given. The data is published to the ZMQ socket.

    :return: None
    """
    args = parse_args()
    context, socket = setup_zmq_socket(args.ip, args.port)
    on_change_only = args.onchange

    try:
        root = tk.Tk()
        app = JoystickGUI(root, on_change_only)
        app.set_output_callback(publish_data(socket))
        
        root.mainloop()

    except KeyboardInterrupt:
        print("\n[Main] Keyboard interrupt received. Exiting...")

    except Exception as e:
        print(f"[Main] {e}")

    finally:
        print("[Main] Exiting...")
        socket.close()
        context.term()

if __name__ == "__main__":
    main()
