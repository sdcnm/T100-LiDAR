"""A synthetic room and protocol peer, bound to loopback only."""

import math
import socket
import struct
import threading
import time
import zlib

from .parameters import BY_NAME, BY_UID
from .protocol import Frame, ProtocolError, encode_data_header


class Simulator:
    def __init__(self, destinations, control_port=0):
        if any(address[0] != "127.0.0.1" for address in destinations):
            raise ValueError("simulator destinations must use IPv4 loopback")
        self.destinations = list(destinations)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", control_port))
        self.socket.settimeout(0.02)
        self.address = self.socket.getsockname()
        self.stop_event = threading.Event()
        self.thread = None
        self.target = 2
        self.enabled = False
        self.write_mode = False
        self.parameters = {}
        self.started_at = time.monotonic_ns()
        self.packet_sequence = 0
        self.imu_sequence = 0
        self.status_sequence = 0
        for p in BY_UID.values():
            value = "127.0.0.1" if p.fmt == "ip" else 0
            if p.name.endswith("yaw_stop"):
                value = 360
            elif p.name.endswith("pitch_start"):
                value = -7
            elif p.name.endswith("pitch_stop"):
                value = 52
            elif p.name in ("pcl_data_type", "imu_en"):
                value = 1
            elif p.name == "tgt_state":
                value = 2
            elif p.name.endswith("_port"):
                kind = {"host_pcl_port": 0, "host_imu_port": 1, "host_msg_port": 2}[p.name]
                value = destinations[kind][1]
            self.parameters[p.uid] = p.encode(value)

    @property
    def actual_state(self):
        return None if self.write_mode else 1 if self.target == 1 and self.enabled else 2

    def start(self):
        self.thread = threading.Thread(target=self._run, name="t100-simulator", daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        self.socket.close()

    def _run(self):
        next_points = next_status = time.monotonic()
        while not self.stop_event.is_set():
            try:
                data, source = self.socket.recvfrom(65535)
                request = Frame.decode(data)
                if source[0] == "127.0.0.1" and request.flags == 0x40 and (request.sender, request.receiver) == (0x28, 0xAD):
                    response = self.dispatch(request)
                    self.socket.sendto(request.reply(response).encode(), source)
            except socket.timeout:
                pass
            except (ProtocolError, ValueError, KeyError, struct.error):
                pass
            now = time.monotonic()
            if now >= next_status and not self.write_mode:
                self._status()
                next_status = now + 0.2
            if now >= next_points:
                if self.actual_state == 1 and not self.write_mode:
                    for _ in range(8):
                        self.socket.sendto(self._point_packet(), self.destinations[0])
                    self.socket.sendto(self._imu_packet(), self.destinations[1])
                next_points = now + 0.1

    def dispatch(self, request):
        cmd, data = (request.command_set, request.command_id), request.payload
        if self.write_mode:
            return b"\x03"
        if cmd == (0x32, 0x12):
            if len(data) != 8:
                return b"\x07"
            uid, length = struct.unpack("<II", data)
            if uid not in self.parameters or len(self.parameters[uid]) != length:
                return b"\x03"
            value = self.parameters[uid]
            return struct.pack("<BIHH", 0, uid, len(value), 0) + value
        if cmd == (0x32, 0x42):
            count = struct.unpack_from("<H", data)[0]
            if len(data) != 2 + count * 4:
                return b"\x07"
            uids = struct.unpack_from("<" + "I" * count, data, 2)
            if any(uid not in self.parameters for uid in uids):
                return b"\x03"
            return struct.pack("<BIH", 0, 0, count) + b"".join(struct.pack("<IH", uid, len(self.parameters[uid])) + self.parameters[uid] for uid in uids)
        if cmd in ((0x32, 0x13), (0x32, 0x43)):
            batch = cmd[1] == 0x43
            if batch:
                if len(data) < 10 or data[:4] != b"\0\0\x01\0":
                    return struct.pack("<BI", 7, 0)
                uid, length = struct.unpack_from("<IH", data, 4)
                value = data[10:]
            else:
                uid, length = struct.unpack_from("<II", data)
                value = data[8:]
            p = BY_UID.get(uid)
            code = 0 if p and p.writable and len(value) == length == p.size else 3
            if code == 0:
                decoded = p.decode(value)
                if (p.fmt == "ip" and decoded != "127.0.0.1") or (p.name.endswith("_port") and decoded == 0) or (p.name == "roi_en" and decoded > 3):
                    code = 3
                else:
                    self.parameters[uid] = value
                    if p.name.startswith("host_"):
                        index = {"msg": 2, "pcl": 0, "imu": 1}[p.name.split("_")[1]]
                        current = self.destinations[index]
                        self.destinations[index] = (decoded, current[1]) if p.fmt == "ip" else (current[0], decoded)
            return struct.pack("<BI", code, uid if code else 0) if batch else bytes((code,))
        if cmd == (0x32, 0x50):
            if len(data) != 9 or data[:8] != struct.pack("<HIH", 1, 3, 1) or data[8] not in (1, 2):
                return struct.pack("<BI", 7, 3)
            self.target = data[8]
            return b"\0" * 5
        if cmd == (0x32, 0x51):
            if data != struct.pack("<HI", 1, 3):
                return b"\x07"
            return struct.pack("<BIHIHB", 0, 0, 1, 3, 1, self.target)
        if cmd == (0x32, 0x14):
            if data not in (b"\0", b"\x01"):
                return b"\x07"
            self.enabled = bool(data[0])
            return b"\0"
        if cmd == (0, 0x0B) and not data:
            self.target, self.enabled = 2, False
            return b"\0"
        if cmd == (0, 7) and not data:
            self.write_mode = True
            return b"\0" * 3
        if cmd == (0, 0x68) and len(data) == 9:
            # The public document does not define this command's ACK layout.
            return b"\x03"
        return b"\x03"

    def _status(self):
        payload = struct.pack("<HHHHB", 1, 0, 0x8006, 1, self.actual_state)
        frame = Frame(1, 2, payload, self.status_sequence & 65535, 0xAD, 0x28, 0)
        self.status_sequence += 1
        self.socket.sendto(frame.encode(), self.destinations[2])

    def _roi_allows(self, x, y, z):
        enabled = self.parameters[0x10023][0]
        if not enabled:
            return True
        yaw = math.degrees(math.atan2(y, x)) % 360
        pitch = math.degrees(math.atan2(z, math.hypot(x, y)))
        for window in range(2):
            base = 0x10024 + window * 5
            values = [struct.unpack("<i", self.parameters[base + i])[0] for i in range(4)]
            if enabled & (1 << window) and values[0] <= yaw <= values[1] and values[2] <= pitch <= values[3]:
                return True
        return False

    def _point_packet(self):
        number = self.packet_sequence
        timestamp = time.monotonic_ns() - self.started_at
        data = encode_data_header(1, number, timestamp)
        for i in range(96):
            index = number * 96 + i
            # Incommensurate steps spread samples over the same fixed scene.
            angle = (index * 2.399963229728653) % math.tau
            mixed = (index * 2654435761) & 0xFFFFFFFF
            mixed = ((mixed ^ (mixed >> 16)) * 2246822519) & 0xFFFFFFFF
            f = (mixed & 0xFFFFFF) / 0xFFFFFF
            c, s = math.cos(angle), math.sin(angle)
            distance = min(7.5 / max(abs(c), 1e-6), 5.2 / max(abs(s), 1e-6))
            if index % 5 == 0:
                radius = 1.7 + f * 6.0
                x, y, z = c * radius, s * radius, -0.7
            elif index % 7 == 0:
                x, y, z = 2.3 + math.cos(angle) * 0.7, 1.1 + math.sin(angle) * 0.7, -0.6 + f * 3.8
            else:
                x, y, z = c * distance, s * distance, -0.6 + f * 4.8
            if not self._roi_allows(x, y, z):
                x = y = z = 0
            reflectivity = 70 + (index * 13) % 180
            data.extend(struct.pack("<iiiBB", round(x * 1000), round(y * 1000), round(z * 1000), reflectivity, 0))
        self.packet_sequence = (number + 1) & 65535
        return bytes(data)

    def _imu_packet(self):
        elapsed = (time.monotonic_ns() - self.started_at) / 1e9
        data = encode_data_header(0, self.imu_sequence, int(elapsed * 1e9))
        data.extend(struct.pack("<6f", 0.003 * math.sin(elapsed), 0.002 * math.cos(elapsed), 0.001, 0.003, -0.002, 1.0))
        struct.pack_into("<I", data, 24, zlib.crc32(data[28:]))
        self.imu_sequence = (self.imu_sequence + 1) & 65535
        return bytes(data)
