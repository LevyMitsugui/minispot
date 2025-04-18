import zmq
import msgpack

socket_path = "/tmp/RollPitch.socket"

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(f"ipc://{socket_path}")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

while True:
    msg = socket.recv()
    print(msgpack.unpackb(msg))