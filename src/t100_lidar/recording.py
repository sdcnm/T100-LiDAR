"""Write datagrams to a standard PCAP file with raw IPv4/UDP packets."""

import socket
import struct
import threading
import time


class PcapWriter:
    def __init__(self, path):
        self.output = open(path, "xb")
        self.lock = threading.Lock()
        self.output.write(struct.pack("<IHHIIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 101))

    def write(self, payload, source, destination):
        udp_length = 8 + len(payload)
        ip = bytearray(struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + udp_length, 0, 0, 64, 17, 0, socket.inet_aton(source[0]), socket.inet_aton(destination[0])))
        total = sum(struct.unpack("!10H", ip))
        while total >> 16:
            total = (total & 65535) + (total >> 16)
        struct.pack_into("!H", ip, 10, (~total) & 65535)
        packet = bytes(ip) + struct.pack("!HHHH", source[1], destination[1], udp_length, 0) + payload
        now = time.time()
        seconds = int(now)
        record = struct.pack("<IIII", seconds, int((now - seconds) * 1e6), len(packet), len(packet))
        with self.lock:
            self.output.write(record + packet)
            self.output.flush()

    def close(self):
        with self.lock:
            self.output.close()
