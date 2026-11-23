"""Wire codecs. Offsets and examples follow docs/protocol.en.md."""

import math
import struct
import zlib
from dataclasses import dataclass
from typing import Dict, Tuple


class ProtocolError(ValueError):
    """A datagram is malformed or fails its integrity checks."""


def crc8(data: bytes) -> int:
    result = 0x77
    for octet in data:
        result ^= octet
        for _ in range(8):
            result = (result >> 1) ^ (0x8C if result & 1 else 0)
    return result


def crc16(data: bytes) -> int:
    result = 0x3692
    for octet in data:
        result ^= octet
        for _ in range(8):
            result = (result >> 1) ^ (0x8408 if result & 1 else 0)
    return result


@dataclass(frozen=True)
class Frame:
    command_set: int
    command_id: int
    payload: bytes = b""
    sequence: int = 0
    sender: int = 0x28
    receiver: int = 0xAD
    flags: int = 0x40

    @property
    def command(self) -> str:
        return f"{self.command_set:02X}/{self.command_id:02X}"

    def encode(self) -> bytes:
        size = 13 + len(self.payload)
        if not 13 <= size <= 1023:
            raise ProtocolError("control frame must contain 13..1023 bytes")
        for value in (self.sender, self.receiver, self.flags, self.command_set, self.command_id):
            if not 0 <= value <= 255:
                raise ProtocolError("control byte out of range")
        if not 0 <= self.sequence <= 65535:
            raise ProtocolError("sequence must fit u16")
        prefix = bytes((0x55, size & 255, 4 | (size >> 8)))
        header = prefix + bytes((crc8(prefix), self.sender, self.receiver))
        body = header + struct.pack("<HBBB", self.sequence, self.flags, self.command_set, self.command_id) + self.payload
        return body + struct.pack("<H", crc16(body))

    @classmethod
    def decode(cls, data: bytes) -> "Frame":
        if len(data) < 13 or data[0] != 0x55:
            raise ProtocolError("invalid control header")
        size = data[1] | ((data[2] & 3) << 8)
        if size != len(data) or data[2] >> 2 != 1:
            raise ProtocolError("control length/version mismatch")
        if crc8(data[:3]) != data[3]:
            raise ProtocolError("control CRC8 mismatch")
        if crc16(data[:-2]) != struct.unpack_from("<H", data, len(data) - 2)[0]:
            raise ProtocolError("control CRC16 mismatch")
        return cls(data[9], data[10], data[11:-2], struct.unpack_from("<H", data, 6)[0], data[4], data[5], data[8])

    def reply(self, payload: bytes) -> "Frame":
        return Frame(self.command_set, self.command_id, payload, self.sequence, self.receiver, self.sender, 0x80)


@dataclass(frozen=True)
class Point:
    x: int
    y: int
    z: int
    reflectivity: int
    tag: int

    def valid(self, max_range_m: float = 150.0) -> bool:
        squared = self.x * self.x + self.y * self.y + self.z * self.z
        return 0 < squared <= (max_range_m * 1000) ** 2

    def as_meters(self):
        return [self.x / 1000, self.y / 1000, self.z / 1000, self.reflectivity, self.tag]


@dataclass(frozen=True)
class DataPacket:
    kind: str
    sequence: int
    timestamp_ns: int
    time_type: int
    points: Tuple[Point, ...] = ()
    gyro: Tuple[float, ...] = ()
    acceleration: Tuple[float, ...] = ()
    crc32_checked: bool = False


def decode_data(data: bytes) -> DataPacket:
    if len(data) < 36 or data[0] != 1:
        raise ProtocolError("invalid data header")
    size = struct.unpack_from("<H", data, 1)[0]
    count, sequence = struct.unpack_from("<HH", data, 5)
    if size != len(data):
        raise ProtocolError("data length mismatch")
    timestamp = struct.unpack_from("<Q", data, 28)[0]
    dtype, time_type = data[10:12]
    if dtype == 1:
        if count != 96 or len(data) != 1380:
            raise ProtocolError("expected 96 XYZ points in a 1380-byte packet")
        points = tuple(Point(*record) for record in struct.iter_unpack("<iiiBB", data[36:]))
        return DataPacket("points", sequence, timestamp, time_type, points)
    if dtype == 0:
        if count != 1 or len(data) != 60:
            raise ProtocolError("expected one IMU sample in a 60-byte packet")
        expected = struct.unpack_from("<I", data, 24)[0]
        if zlib.crc32(data[28:]) != expected:
            raise ProtocolError("IMU CRC32 mismatch")
        values = struct.unpack_from("<6f", data, 36)
        if not all(math.isfinite(v) for v in values):
            raise ProtocolError("non-finite IMU sample")
        return DataPacket("imu", sequence, timestamp, time_type, (), values[:3], values[3:], True)
    raise ProtocolError(f"unsupported data_type {dtype}")


def encode_data_header(kind: int, sequence: int, timestamp_ns: int) -> bytearray:
    data = bytearray(36)
    data[0] = 1
    struct.pack_into("<HHHH", data, 1, 1380 if kind == 1 else 60, 1000, 96 if kind == 1 else 1, sequence & 0xFFFF)
    data[10] = kind
    struct.pack_into("<Q", data, 28, timestamp_ns)
    return data


def decode_status(payload: bytes) -> Dict[int, bytes]:
    if len(payload) < 4:
        raise ProtocolError("short status TLV header")
    count = struct.unpack_from("<H", payload)[0]
    offset, result = 4, {}
    for _ in range(count):
        if offset + 4 > len(payload):
            raise ProtocolError("truncated status item")
        key, size = struct.unpack_from("<HH", payload, offset)
        offset += 4
        if offset + size > len(payload) or key in result:
            raise ProtocolError("truncated or duplicate status value")
        result[key] = payload[offset:offset + size]
        offset += size
    if offset != len(payload):
        raise ProtocolError("unexpected bytes after status TLVs")
    return result


def discovery_info(frame: Frame):
    if (frame.command_set, frame.command_id) != (0x32, 0x21) or len(frame.payload) != 19:
        raise ProtocolError("invalid discovery message")
    return {"serial": frame.payload[:14].rstrip(b"\0").decode("ascii", "replace"), "flag": frame.payload[16]}
