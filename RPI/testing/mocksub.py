import zmq
import msgpack

sockets_paths = ["/tmp/pico_serial_handler.socket",
                 "/tmp/static_pose_command.socket"]
sockets_tcp = [{'ip':"localhost", 'port': 5555}]

context = zmq.Context()
socket = context.socket(zmq.SUB)
#socket.connect(f"ipc://{socket_path}")
for socket_path in sockets_paths:
    socket.connect(f"ipc://{socket_path}")
# Subscribe to all topics
for socket_tcp in sockets_tcp:
    socket.connect(f"tcp://{socket_tcp['ip']}:{socket_tcp['port']}")

socket.setsockopt_string(zmq.SUBSCRIBE, "pico/accel")
socket.setsockopt_string(zmq.SUBSCRIBE, "pico/log")
socket.setsockopt_string(zmq.SUBSCRIBE, "command")
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
    