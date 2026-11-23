"""UDP request/response client. Mutations are never retried automatically."""

import ipaddress
import json
import random
import socket
import struct
import threading
import time
from pathlib import Path

from .parameters import BY_UID, ROI_PARAMETERS, parameter
from .protocol import Frame, ProtocolError


class DeviceError(RuntimeError):
    pass


class Client:
    def __init__(self, address, port=60000, timeout=1.0, bind="0.0.0.0", trace=None):
        self.address = str(ipaddress.IPv4Address(address))
        if not 1 <= int(port) <= 65535 or timeout <= 0:
            raise ValueError("invalid control port or timeout")
        self.destination = (self.address, int(port))
        self.timeout = timeout
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((bind, 0))
        self.sequence = random.randrange(65536)
        self.lock = threading.Lock()
        self.trace = trace

    def close(self):
        self.socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def request(self, command_set, command_id, payload=b"", sequence=None):
        with self.lock:
            seq = self.sequence if sequence is None else sequence
            self.sequence = (seq + 1) & 0xFFFF
            request = Frame(command_set, command_id, payload, seq)
            raw = request.encode()
            if self.trace:
                self.trace("TX", request, raw)
            self.socket.sendto(raw, self.destination)
            deadline = time.monotonic() + self.timeout
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError(f"no matching response for {request.command}; request was not retried")
                self.socket.settimeout(left)
                try:
                    raw, source = self.socket.recvfrom(65535)
                except socket.timeout:
                    raise TimeoutError(f"no matching response for {request.command}; request was not retried") from None
                if source != self.destination:
                    continue
                try:
                    reply = Frame.decode(raw)
                except ProtocolError:
                    continue
                if (reply.sequence, reply.command_set, reply.command_id, reply.sender, reply.receiver) != (seq, command_set, command_id, request.receiver, request.sender) or not reply.flags & 0x80:
                    continue
                if self.trace:
                    self.trace("RX", reply, raw)
                return reply.payload

    @staticmethod
    def _success(data, size=None):
        if not data or data[0] != 0:
            raise DeviceError(f"device returned {data.hex(' ') or 'an empty response'}")
        if size is not None and len(data) != size:
            raise ProtocolError(f"unexpected response length {len(data)} (expected {size})")

    def read(self, item):
        p = parameter(item)
        data = self.request(0x32, 0x12, struct.pack("<II", p.uid, p.size))
        self._success(data)
        if len(data) < 9:
            raise ProtocolError("short UID read response")
        uid, size = struct.unpack_from("<IH", data, 1)
        if uid != p.uid or size != p.size or len(data) != 9 + size:
            raise ProtocolError("UID read response does not match request")
        return p.decode(data[9:])

    def read_many(self, items):
        params = [parameter(item) for item in items]
        if not params or len(params) > 64 or len({p.uid for p in params}) != len(params):
            raise ValueError("read 1..64 unique parameters per batch")
        data = self.request(0x32, 0x42, struct.pack("<H", len(params)) + b"".join(struct.pack("<I", p.uid) for p in params))
        self._success(data)
        if len(data) < 7:
            raise ProtocolError("short batch response")
        error_key, count = struct.unpack_from("<IH", data, 1)
        if error_key or count != len(params):
            raise ProtocolError("batch response item count/error key mismatch")
        expected = {p.uid: p for p in params}
        offset, result = 7, {}
        for _ in range(count):
            if offset + 6 > len(data):
                raise ProtocolError("short batch item")
            uid, size = struct.unpack_from("<IH", data, offset)
            offset += 6
            if uid not in expected or expected[uid].name in result or offset + size > len(data):
                raise ProtocolError("unexpected, duplicate or truncated batch UID")
            result[expected[uid].name] = expected[uid].decode(data[offset:offset + size])
            offset += size
        if offset != len(data):
            raise ProtocolError("trailing bytes in batch response")
        return result

    def write(self, item, value):
        p = parameter(item)
        if not p.writable:
            raise ValueError(f"{p.name} is read-only/reserved")
        raw = p.encode(value)
        if p.name == "roi_en" and not 0 <= int(value) <= 3:
            raise ValueError("ROI bitmask must be 0..3")
        if p.name.endswith("_port") and not 1 <= int(value) <= 65535:
            raise ValueError("destination port must be 1..65535")
        self._success(self.request(0x32, 0x13, struct.pack("<II", p.uid, len(raw)) + raw), 1)
        readback = self.read(p)
        expected = p.decode(raw)
        if readback != expected:
            raise DeviceError(f"write readback mismatch for {p.name}: {readback!r} != {expected!r}")
        return readback

    def target_state(self):
        data = self.request(0x32, 0x51, struct.pack("<HI", 1, 3))
        self._success(data, 14)
        if struct.unpack("<BIHIH", data[:13]) != (0, 0, 1, 3, 1):
            raise ProtocolError("unexpected target state response")
        return data[13]

    def set_target(self, value):
        if value not in (1, 2):
            raise ValueError("target state must be 1 (sampling) or 2 (standby)")
        data = self.request(0x32, 0x50, struct.pack("<HIHB", 1, 3, 1, value))
        self._success(data, 5)
        if data != b"\0" * 5:
            raise DeviceError("target state error key is nonzero")

    def sampling_enable(self, enabled):
        self._success(self.request(0x32, 0x14, bytes((int(bool(enabled)),))), 1)

    def start(self):
        self.set_target(1)
        self.sampling_enable(True)
        return self.target_state()

    def stop(self):
        self.sampling_enable(False)
        self.set_target(2)
        return self.target_state()

    def roi(self):
        return self.read_many(ROI_PARAMETERS)

    def set_roi(self, start, end, backup_path=None):
        if not (isinstance(start, int) and isinstance(end, int) and 0 <= start < end <= 360):
            raise ValueError("single-window yaw requires integer 0 <= start < end <= 360")
        previous = self.roi()
        if backup_path is not None:
            with Path(backup_path).open("x", encoding="utf-8") as output:
                json.dump({"device_ip": self.address, "saved_at": time.time(), "parameters": previous}, output, indent=2)
        # Disable first. If a later write fails, leave the enable bit off; callers
        # must inspect readback before attempting a restoration.
        self.write("roi_en", 0)
        self.write("roi_cfg0.yaw_start", start)
        self.write("roi_cfg0.yaw_stop", end)
        self.write("roi_en", 1)
        return {"before": previous, "after": self.roi()}

    def reboot(self):
        self._success(self.request(0, 0x0B), 1)

    def enter_write_mode(self):
        self._success(self.request(0, 7), 3)
        return {"request_accepted": True}
