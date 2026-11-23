"""The 25 published UIDs, with explicit type and write permissions."""

import ipaddress
import struct
from dataclasses import dataclass


@dataclass(frozen=True)
class Parameter:
    uid: int
    name: str
    fmt: str
    writable: bool = False

    @property
    def size(self):
        return 4 if self.fmt == "ip" else struct.calcsize("<" + self.fmt)

    def encode(self, value):
        if self.fmt == "ip":
            return int(ipaddress.IPv4Address(value)).to_bytes(4, "little")
        return struct.pack("<" + self.fmt, int(value))

    def decode(self, data):
        if len(data) != self.size:
            raise ValueError(f"{self.name}: expected {self.size} bytes, received {len(data)}")
        if self.fmt == "ip":
            return str(ipaddress.IPv4Address(int.from_bytes(data, "little")))
        return struct.unpack("<" + self.fmt, data)[0]


PARAMETERS = [Parameter(0x10001, "dev_idx", "B"), Parameter(0x10002, "ip_mode", "B"),
              Parameter(0x10014, "imu_en", "B"), Parameter(0x10015, "coord_type", "B")]
for suffix, kind in ((0x19, "msg"), (0x1B, "pcl"), (0x1D, "imu")):
    PARAMETERS += [Parameter(0x10000 + suffix, f"host_{kind}_ip", "ip", True),
                   Parameter(0x10001 + suffix, f"host_{kind}_port", "H", True)]
PARAMETERS += [Parameter(0x1001F, "sample_mode", "B"), Parameter(0x10020, "pattern_mode", "B"),
               Parameter(0x10021, "pcl_data_type", "B"), Parameter(0x10022, "tgt_state", "B"),
               Parameter(0x10023, "roi_en", "B", True)]
for window in range(2):
    for index, field in enumerate(("yaw_start", "yaw_stop", "pitch_start", "pitch_stop", "rsvd")):
        PARAMETERS.append(Parameter(0x10024 + window * 5 + index, f"roi_cfg{window}.{field}", "i", field != "rsvd"))

BY_UID = {item.uid: item for item in PARAMETERS}
BY_NAME = {item.name: item for item in PARAMETERS}
ROI_PARAMETERS = [p for p in PARAMETERS if p.uid >= 0x10023]


def parameter(value):
    if isinstance(value, Parameter):
        return value
    if value in BY_NAME:
        return BY_NAME[value]
    uid = int(value, 0) if isinstance(value, str) else value
    if uid not in BY_UID:
        raise ValueError(f"UID {uid:#x} is not in the published parameter list")
    return BY_UID[uid]
