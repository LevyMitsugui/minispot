import zmq
import msgpack

socket_path = "/tmp/pico_serial_handler.socket"

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(f"ipc://{socket_path}")
socket.setsockopt_string(zmq.SUBSCRIBE, "test_topic2")

while True:
    topic, data = socket.recv_multipart()
    data = msgpack.unpackb(data, raw=False)
    print(topic.decode(), data)
    