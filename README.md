# T100 LiDAR Toolkit — Ethernet / UDP protocol tools

[简体中文](README.zh-CN.md) · [Protocol reference](docs/protocol.en.md)

A new Python implementation of the published T100 LiDAR protocol: control frames, point cloud and IMU decoding, parameter access, sampling control, ROI settings, and the firmware write mode entry command. Includes a local UDP simulator and a browser preview.

The simulator produces synthetic points; it does not demonstrate physical device performance.

## Quick start

Python 3.9 or newer. The application has no third-party runtime dependencies.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e .
.venv/bin/t100 demo --port 18766
```

Open `http://127.0.0.1:18766`. On Windows, use `.venv\Scripts\python` and `.venv\Scripts\t100` instead. On macOS, the included `Start Demo.command` launches the preview directly from source.

The demo includes start/stop, yaw window changes with readback, 3D/top views, IMU data, complete TX/RX frames, CSV/JSON export, and the firmware write mode entry command. Its UDP and HTTP sockets are confined to `127.0.0.1`.

## Command-line examples

Generate the published update-entry example without sending anything:

```sh
t100 frame enter-write-mode --sequence 0x4810
t100 frame read --parameter pcl_data_type --sequence 0x0110
t100 parameters
```

Listen for discovery or data. These commands do not change device settings:

```sh
t100 discover --seconds 5
t100 --ip 192.0.2.10 capture --bind 192.0.2.50 --seconds 10 --pcap capture.pcap
t100 --ip 192.0.2.10 status
t100 --ip 192.0.2.10 read pcl_data_type
```

Addresses above are documentation examples. Configure the actual receiving address and destination ports for your device. `capture` uses point port 60001 and IMU port 60003 by default; both are configurable. `--ip` filters incoming data to that source address.

Explicit control commands send requests to device UDP port 60000:

```sh
t100 --ip 192.0.2.10 --trace start
t100 --ip 192.0.2.10 roi --start 0 --end 180 --backup roi-before.json
t100 --ip 192.0.2.10 roi --disable --backup roi-before-disable.json
t100 --ip 192.0.2.10 stop
t100 --ip 192.0.2.10 enter-write-mode
```

The final command sends `00/07` to enter firmware write mode. The browser's **Enter firmware write mode** button sends this command to the local simulator. **Resume simulated sampling** resets the local model. The CLI command sends the request to the device address you specify.

ROI changes preserve the eleven original ROI values in a new JSON file, disable cropping before writes, and read back each written parameter. On a partial failure, inspect the settings before restoring them. The generic `write` command is lower-level and requires the caller to preserve original values.

Control requests use a single socket with matching endpoint, sequence and command checks. Writes are sent once, without automatic retries. A successful ACK/readback does not by itself establish actual sampling, coverage, motor state, or a successful reboot.

## Protocol and implementation scope

| Component | Included |
|---|---|
| Python library | CRC8/CRC16, frame codec, 25 UIDs, single/batch reads, single writes, target control, sampling, ROI, reboot and update entry |
| Data decoder | 96-point XYZ packets, IMU with CRC32, status TLVs, sequence accounting |
| Simulator | Local UDP control peer, deterministic synthetic room, ROI cropping, point/IMU/status streams |
| Recording | PCAP with reconstructed IPv4/UDP headers, plus browser CSV and JSON snapshots |
| C header | Portable control-frame CRC/build/check helpers |
| Wireshark Lua | Control/data field dissection; CRC fields are displayed, not verified |

External time synchronization can be encoded with `t100 frame time-sync`; the public reference does not define its ACK or complete synchronization conditions. The simulator does not claim to reproduce undocumented device behavior. No ROS/ROS2 integration is included.

PCAP files preserve received UDP payloads, timing and socket endpoints. Link-layer information and original IP headers are not captured. Browser CSV exports contain valid simulated points from the rolling two-second buffer. Sequence gaps are observable counters, not a measurement of network loss by themselves.

## Development

```sh
.venv/bin/python -m unittest discover -s tests -v
cc -std=c99 -Wall -Wextra -Werror -Iextras tests/test_header.c -o /tmp/t100-header-test
/tmp/t100-header-test
```

Wire examples in the public protocol are used as fixed compatibility fixtures. Other tests cover malformed packets, CRC failures, response matching, real loopback UDP exchanges, ROI/readback, HTTP controls, PCAP output, and the firmware write mode entry.

See [the Chinese quick start](README.zh-CN.md), [the published protocol snapshot](docs/protocol.en.md), [the protocol source record](docs/SOURCE.md).
