"""Receive point cloud, IMU and status datagrams without optional packages."""

import collections
import select
import socket
import struct
import threading
import time

from .protocol import Frame, ProtocolError, decode_data, decode_status


class SequenceTracker:
    def __init__(self):
        self.last = None
        self.missing = 0
        self.duplicates = 0
        self.out_of_order = 0

    def update(self, value):
        if self.last is not None:
            delta = (value - self.last) & 0xFFFF
            if delta == 0:
                self.duplicates += 1
                return
            if delta >= 32768:
                self.out_of_order += 1
                return
            self.missing += delta - 1
        self.last = value


class StreamReceiver:
    def __init__(self, bind="127.0.0.1", point_port=0, imu_port=0, status_port=0, on_datagram=None, source_ip=None):
        self.sockets = []
        try:
            for port in (point_port, imu_port, status_port):
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sockets.append(sock)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
                sock.bind((bind, port))
        except Exception:
            for sock in self.sockets:
                sock.close()
            raise
        self.addresses = [sock.getsockname() for sock in self.sockets]
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.on_datagram = on_datagram
        self.source_ip = source_ip
        self.points = collections.deque(maxlen=24000)
        self.samples = collections.deque(maxlen=2000)
        self.counters = {"point_packets": 0, "imu_packets": 0, "status_packets": 0, "valid_points": 0, "filtered_points": 0, "bad_packets": 0, "bytes": 0}
        self.trackers = {name: SequenceTracker() for name in ("points", "imu")}
        self.imu = None
        self.status = {}
        self.last_point_at = None
        self.started_at = time.monotonic()

    def start(self):
        self.thread = threading.Thread(target=self._run, name="t100-data", daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        for sock in self.sockets:
            sock.close()

    def _run(self):
        while not self.stop_event.is_set():
            ready, _, _ = select.select(self.sockets, [], [], 0.1)
            for sock in ready:
                data, source = sock.recvfrom(65535)
                if self.source_ip and source[0] != self.source_ip:
                    continue
                destination = sock.getsockname()
                if self.on_datagram:
                    self.on_datagram(data, source, destination)
                self.ingest(data)

    def ingest(self, data):
        now = time.monotonic()
        with self.lock:
            self.counters["bytes"] += len(data)
            try:
                if data and data[0] == 0x55:
                    frame = Frame.decode(data)
                    if (frame.command_set, frame.command_id) != (1, 2):
                        raise ProtocolError("unexpected control frame on data socket")
                    self.status.update(decode_status(frame.payload))
                    self.counters["status_packets"] += 1
                    return
                packet = decode_data(data)
                self.trackers[packet.kind].update(packet.sequence)
                if packet.kind == "points":
                    valid = [point.as_meters() for point in packet.points if point.valid()]
                    self.points.extend((now, point) for point in valid)
                    self.samples.append((now, len(valid)))
                    self.counters["valid_points"] += len(valid)
                    self.counters["filtered_points"] += len(packet.points) - len(valid)
                    self.counters["point_packets"] += 1
                    self.last_point_at = now
                else:
                    self.imu = {"gyro_rad_s": packet.gyro, "acc_g": packet.acceleration, "timestamp_ns": packet.timestamp_ns, "crc32_valid": packet.crc32_checked}
                    self.counters["imu_packets"] += 1
            except (ProtocolError, ValueError, struct.error):
                self.counters["bad_packets"] += 1

    def clear_points(self):
        with self.lock:
            self.points.clear()

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            while self.points and self.points[0][0] < now - 2.0:
                self.points.popleft()
            while self.samples and self.samples[0][0] < now - 1.0:
                self.samples.popleft()
            state = self.status.get(0x8006)
            return {**self.counters,
                    "points_per_second": sum(count for _, count in self.samples),
                    "point_age_ms": None if self.last_point_at is None else round((now - self.last_point_at) * 1000),
                    "uptime_s": round(now - self.started_at, 1),
                    "points": [point for _, point in self.points],
                    "imu": self.imu,
                    "actual_state": state[0] if state and len(state) == 1 else None,
                    "sequence": {key: {"last": value.last, "missing": value.missing, "duplicates": value.duplicates, "out_of_order": value.out_of_order} for key, value in self.trackers.items()}}
