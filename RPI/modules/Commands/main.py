import os
import socket
import fcntl
import struct
import time
import zmq
import msgpack

STD_TOPIC = "CMD" # Default topic for ESP32 control messages
PORT = 5000

import os

def get_wifi_interface():
    interfaces = os.listdir('/sys/class/net/')
    for iface in interfaces:
        if iface.startswith('wl'):
            return iface
    return None

def get_ip_address(ifname='wlan0'):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15].encode('utf-8'))
    )[20:24])

def setup_zmq_socket(ip, port):
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(f"tcp://{ip}:{port}")

    return context, pub_socket

def send_to_esp(pub_socket, string):
    topic = STD_TOPIC
    data  = str(string)
    pub_socket.send_multipart([topic.encode(), msgpack.packb(data)])

def main():
    try:
        iface = get_wifi_interface()
        if iface:
            IP = get_ip_address(iface)
            print(f"WLAN IP for {iface}: {IP}")
        else:
            print("No Wi-Fi interface found.")

        context , socket = setup_zmq_socket(IP, PORT)

        while True:
            userInput = input("CMD: ")
            send_to_esp(socket, userInput)
    
    except KeyboardInterrupt:
        print("\nExiting")

if __name__ == "__main__":
    main()