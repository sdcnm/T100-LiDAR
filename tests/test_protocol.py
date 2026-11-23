import re
import struct
import unittest
import zlib
from pathlib import Path

from t100_lidar.parameters import PARAMETERS, parameter
from t100_lidar.protocol import DataPacket, Frame, Point, ProtocolError, decode_data, decode_status
from t100_lidar.stream import SequenceTracker


class FrameTests(unittest.TestCase):
    def test_all_published_frames(self):
        path = Path(__file__).resolve().parents[1] / "docs/protocol.en.md"
        examples = []
        for line in path.read_text().splitlines():
            if re.match(r"^(TX|RX)  ", line):
                raw = bytes.fromhex(line[4:].replace("|", ""))
                with self.subTest(frame=line):
                    frame = Frame.decode(raw)
                    self.assertEqual(frame.encode(), raw)
                examples.append(raw)
        self.assertEqual(len(examples), 20)

    def test_write_mode_entry_golden_vector(self):
        self.assertEqual(Frame(0, 7, sequence=0x4810).encode().hex(), "550d043328ad104840000776f9")

    def test_corruption_truncation_and_version(self):
        raw = Frame(0, 7, sequence=0x4810).encode()
        for index in range(len(raw)):
            corrupt = bytearray(raw)
            corrupt[index] ^= 1
            with self.subTest(index=index), self.assertRaises(ProtocolError):
                Frame.decode(bytes(corrupt))
        for size in range(len(raw)):
            with self.assertRaises(ProtocolError):
                Frame.decode(raw[:size])

    def test_length_boundaries(self):
        for size in (0, 242, 243, 500, 1010):
            frame = Frame(0x32, 0x42, b"a" * size, 65535)
            self.assertEqual(Frame.decode(frame.encode()), frame)
        with self.assertRaises(ProtocolError):
            Frame(0, 7, b"x" * 1011).encode()

    def test_status_tlvs_and_unknown_keys(self):
        payload = bytes.fromhex("020000000680010002999902001122")
        self.assertEqual(decode_status(payload), {0x8006:b"\x02", 0x9999:b"\x11\x22"})
        for corrupt in (payload[:-1], payload + b"0", b"", b"\x02\0\0\0\x06\x80\x01\0\x02\x06\x80\x01\0\x01"):
            with self.assertRaises(ProtocolError):
                decode_status(corrupt)


class DataTests(unittest.TestCase):
    def point_fixture(self):
        raw = bytearray(1380)
        raw[0] = 1
        struct.pack_into("<H",raw,1,1380)
        struct.pack_into("<HH",raw,5,96,65535)
        raw[10] = 1
        struct.pack_into("<Q",raw,28,123456789)
        for i in range(96):
            struct.pack_into("<iiiBB",raw,36+i*14,-1000,2000,3000,60,16)
        return raw

    def test_xyz_decode_units_and_signed_values(self):
        packet = decode_data(bytes(self.point_fixture()))
        self.assertEqual(packet.sequence,65535)
        self.assertEqual(packet.timestamp_ns,123456789)
        self.assertEqual(len(packet.points),96)
        self.assertEqual(packet.points[0].as_meters(),[-1.0,2.0,3.0,60,16])
        self.assertFalse(packet.crc32_checked)

    def test_invalid_point_packets(self):
        good = self.point_fixture()
        cases = [good[:-1],good+b"0"]
        wrong_count = bytearray(good);wrong_count[5]=95;cases.append(wrong_count)
        wrong_kind = bytearray(good);wrong_kind[10]=2;cases.append(wrong_kind)
        for data in cases:
            with self.assertRaises(ProtocolError):
                decode_data(data)

    def test_imu_crc_and_nonfinite(self):
        raw=bytearray(60);raw[0]=1
        struct.pack_into("<H",raw,1,60);struct.pack_into("<H",raw,5,1)
        struct.pack_into("<Q6f",raw,28,100,0.1,-0.2,0.3,0,0,1)
        struct.pack_into("<I",raw,24,zlib.crc32(raw[28:]))
        packet=decode_data(raw)
        self.assertTrue(packet.crc32_checked)
        self.assertAlmostEqual(packet.gyro[1],-.2)
        raw[-1]^=1
        with self.assertRaises(ProtocolError):decode_data(raw)
        struct.pack_into("<f",raw,36,float("nan"));struct.pack_into("<I",raw,24,zlib.crc32(raw[28:]))
        with self.assertRaises(ProtocolError):decode_data(raw)

    def test_filter_uses_geometry_not_reflectivity(self):
        self.assertTrue(Point(1000,0,0,0,0).valid())
        self.assertFalse(Point(0,0,0,60,0).valid())
        self.assertFalse(Point(200000,0,0,255,0).valid())

    def test_sequence_wrap_and_reordering(self):
        tracker=SequenceTracker()
        for value in (65534,65535,0,2,2,1,3):tracker.update(value)
        self.assertEqual((tracker.missing,tracker.duplicates,tracker.out_of_order,tracker.last),(1,1,1,3))


class ParameterTests(unittest.TestCase):
    def test_all_published_uids_and_ip_encoding(self):
        self.assertEqual(len(PARAMETERS),25)
        self.assertEqual(len({p.uid for p in PARAMETERS}),25)
        self.assertEqual(parameter("host_pcl_ip").encode("192.0.2.50"),bytes.fromhex("320200c0"))
        self.assertEqual(parameter("host_pcl_port").encode(60099),bytes.fromhex("c3ea"))
        self.assertFalse(parameter("roi_cfg0.rsvd").writable)
        self.assertEqual(parameter("roi_cfg0.pitch_start").decode(bytes.fromhex("f9ffffff")),-7)


if __name__ == "__main__":unittest.main()
