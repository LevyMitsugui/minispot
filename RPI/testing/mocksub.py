import zmq
import msgpack
import socket as pysocket

sockets_paths = ["/tmp/pico_serial_handler.socket",
                 "/tmp/static_pose_command.socket",
                 "/tmp/pub_esp_serial_handler.socket"]
sockets_tcp = [{'ip':"10.227.157.156", 'port': 5000}]


def is_port_open(host: str, port: int, timeout=2):
    with pysocket.socket(pysocket.AF_INET, pysocket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        try:
            sock.connect((host, port))
            return True
        except (pysocket.timeout, pysocket.error, OSError):
            return False




context = zmq.Context()
socket = context.socket(zmq.SUB)

for socket_path in sockets_paths:
    socket.connect(f"ipc://{socket_path}")

for socket_tcp in sockets_tcp:
    if not is_port_open(socket_tcp['ip'], socket_tcp['port']):
        print(f"Error: Could not connect to {socket_tcp['ip']}:{socket_tcp['port']}, wrong IP or port is not open.")
        continue
    socket.connect(f"tcp://{socket_tcp['ip']}:{socket_tcp['port']}")

socket.setsockopt_string(zmq.SUBSCRIBE, "pico/accel")
socket.setsockopt_string(zmq.SUBSCRIBE, "pico/log")
socket.setsockopt_string(zmq.SUBSCRIBE, "command")
socket.setsockopt_string(zmq.SUBSCRIBE, "ESP/FB/FL")
socket.setsockopt_string(zmq.SUBSCRIBE, "ESP/FB/RL")
try:
    while True:
        topic, data = socket.recv_multipart()
        data = msgpack.unpackb(data, raw=False)
        print(topic.decode(), data)
except KeyboardInterrupt:
    print("Exiting...")

finally:
    socket.close()
    context.term()
    