# Unlocking Point Cloud Restrictions on the T100 LiDAR

[简体中文](README.md)

This document covers Ethernet access, sampling control, point cloud and IMU decoding, configuration access, and scan angle settings. It also lists the command for entering firmware update mode. The control interface uses a `0x55` frame header and UDP port `60000` on the device. Behavior depends on the firmware and runtime configuration. During integration, check command responses, configuration readback, and the actual data streams together.

## 1 Connections and Data Reception

Use a computer or a Linux development board to connect to the LiDAR over Ethernet. Assign the receiving device an IPv4 address that can reach the LiDAR, and allow the required UDP traffic.

| Channel | Direction | Device Port | Receiving Endpoint |
|---|---|---|---|
| Device discovery | Device → Broadcast | 60010 | Listen on `50000` |
| Control requests and responses | Bidirectional | 60000 | Send requests and receive responses through the same socket |
| Status push | Device → Client | 60000 | Check the actual destination. Messages may arrive at the control socket's source port |
| Point cloud | Device → Client | 60001 | Start by checking destination port `60001`, then confirm the actual destination |
| IMU | Device → Client | 60003 | Start by checking destination port `60003`, then confirm the actual destination |

Discovery messages use command `32/21` and are broadcast to `255.255.255.255:50000`. Obtain the LiDAR's IPv4 address from the UDP source address, then send a query to its control port.

The destination stored in the configuration may differ from the current streaming destination. For example, the configured ports may read back as `56201/56301/56401`, while point cloud and IMU traffic goes to `60001/60003`. When troubleshooting reception, check the receiving address, actual destination port, inbound traffic rules, and application socket.

A reference connection sequence follows.

1. Receive a discovery broadcast, identify the device address, and open a control socket.
2. Read and save the existing network and ROI settings, then prepare receiving sockets for point cloud and IMU data.
3. If the streaming destination needs to change, write the IPv4 address and port as described in Section 6. Read them back and check reception.
4. Send the startup commands in Section 2. Check the target state, actual state, and both data streams separately.
5. Check the data type, packet length, and checksum fields as described in Section 3.

## 2 Starting and Stopping Sampling

### Start Sampling

The reference startup sequence uses two write commands followed by a query.

| Step | Command | Request Contents | Purpose |
|---|---|---|---|
| 1 | `32/50` | `count=1, key=3, length=1, value=1` | Set the target state to sampling |
| 2 | `32/14` | `enable=1` | Enable sampling |
| 3 | `32/51` | `count=1, key=3` | Read back the target state |

Receive status messages `01/02` and check whether key `0x8006` reports the sampling state. Then check whether point cloud and IMU packets arrive. Record the target state, actual state, and data streams separately.

The complete startup frames are shown below. `TX` denotes a request sent by the client, and `RX` denotes the device response. Vertical bars are included for readability. The examples include frame lengths and CRCs. Recalculate the CRCs whenever the sequence number, endpoints, or payload changes.

First, set the target state to sampling.

```text
TX  55 16 04 FC | 28 AD | 20 01 | 40 | 32 50 | 01 00 03 00 00 00 01 00 01 | 54 A3
RX  55 12 04 C7 | AD 28 | 20 01 | 80 | 32 50 | 00 00 00 00 00 | 98 F1
```

Next, enable sampling.

```text
TX  55 0E 04 66 | 28 AD | 21 01 | 40 | 32 14 | 01 | CF 4D
RX  55 0E 04 66 | AD 28 | 21 01 | 80 | 32 14 | 00 | A2 4D
```

Finally, query the target state. The last byte of the example response is `01`.

```text
TX  55 13 04 03 | 28 AD | 22 01 | 40 | 32 51 | 01 00 03 00 00 00 | D5 6C
RX  55 1B 04 75 | AD 28 | 22 01 | 80 | 32 51 | 00 00 00 00 00 01 00 03 00 00 00 01 00 01 | 06 D9
```

### Disable Sampling and Select Standby

Send `32/14` with `enable=0`, then use `32/50` to set the target state to `2`. Query the target and actual states afterward.

```text
TX  55 0E 04 66 | 28 AD | 24 01 | 40 | 32 14 | 00 | C1 48
RX  55 0E 04 66 | AD 28 | 24 01 | 80 | 32 14 | 00 | 25 59
```

```text
TX  55 16 04 FC | 28 AD | 25 01 | 40 | 32 50 | 01 00 03 00 00 00 01 00 02 | 9C 1C
RX  55 12 04 C7 | AD 28 | 25 01 | 80 | 32 50 | 00 00 00 00 00 | 23 6D
```

Data may still stream in standby. Check point cloud output, IMU output, and motor operation separately. The target state or a single ACK alone is insufficient to determine whether these have stopped.

### State Control Fields

The offsets below are relative to the start of the command payload. Multibyte fields use little-endian encoding.

| Request Field | `32/50` Set Target | `32/51` Read Target |
|---|---|---|
| count | Offset 0, u16, set to 1 | Offset 0, u16, set to 1 |
| key | Offset 2, u32, set to 3 | Offset 2, u32, set to 3 |
| length | Offset 6, u16, set to 1 | — |
| value | Offset 8, u8, 1 for sampling and 2 for standby | — |

A successful `32/50` response contains `ret_code u8=0, error_key u32=0`. A `32/14` request contains one byte, `01` to enable or `00` to disable. Its successful response is `00`.

A successful `32/51` response has a 14-byte payload with the following layout.

| Field | Offset | Type | Meaning |
|---|---|---|---|
| ret_code | 0 | u8 | 0 on success |
| error_key | 1 | u32 | 0 on success |
| item_count | 5 | u16 | 1 for a single-item query |
| key | 7 | u32 | Echoes 3 |
| length | 11 | u16 | State value length, 1 byte |
| value | 13 | u8 | Target state |

Reference values for status push key `0x8006` are listed below.

| Value | Operating State |
|---|---|
| `0x01` | Sampling |
| `0x02` | Standby |
| `0x04` | Error |
| `0x05` | Self-check |
| `0x06` | Motor startup |
| `0x09` | Ready |

## 3 Point Cloud and IMU Packets

### Common Header

Offsets are measured from the first byte of the UDP payload. The format below uses a 36-byte header. Integer and floating-point fields are little-endian.

| Field | Offset | Bytes | Description |
|---|---|---|---|
| version | 0 | 1 | Packet version `0x01` for the data format described here |
| length | 1 | 2 | Total UDP payload length |
| time_interval | 3 | 2 | Intra-packet point cloud sampling time field, in units of 0.1 µs |
| dot_num | 5 | 2 | 96 for the point cloud format described here, 1 for IMU |
| udp_cnt | 7 | 2 | Packet sequence number. Track each stream separately and handle wraparound |
| frame_cnt | 9 | 1 | Frame counter. Ignore for non-repetitive scanning |
| data_type | 10 | 1 | 0 for IMU, 1 for Cartesian point cloud |
| time_type | 11 | 1 | 0 for time since startup, 1 for PTP, 2 for GPS |
| reserved | 12 | 12 | Reserved. Ignore on reception. These bytes may be nonzero |
| crc32 | 24 | 4 | May be zero in point cloud packets. For IMU, check from offset 28 to the packet end |
| timestamp | 28 | 8 | u64 timestamp in ns |
| data | 36 | Variable | Decode according to data_type |

When `time_type=0`, compare timestamps within the same boot session. Establish new sequence and time baselines after a reset. Check the synchronization source and time origin before comparing absolute time or timestamps from different devices.

### Cartesian Point Cloud

The reference packet length for `data_type=1` is 1380 bytes, containing 96 point records. Each record is 14 bytes. Offsets in the table below are relative to the start of that record.

| Field | Offset | Type | Unit or Meaning |
|---|---|---|---|
| x | 0 | int32 | mm |
| y | 4 | int32 | mm |
| z | 8 | int32 | mm |
| reflectivity | 12 | uint8 | Raw reflectivity value |
| tag | 13 | uint8 | Point attribute tag |

In Python, use `<iiiBB` to decode a point. Divide XYZ by 1000 to convert coordinates to meters before plotting.

Filter using the coordinates and the distance range required by the application. Packets may contain `x=y=z=0` placeholders and no-return sentinel points with a distance of approximately 200 m. Placeholders may have a reflectivity of 60, while sentinel points may have nonzero reflectivity or different tags. Filtering by reflectivity alone or a fixed tag may miss invalid points.

### IMU

The reference packet length for `data_type=0` is 60 bytes, with one sample per packet. The payload contains six IEEE 754 little-endian float32 values.

| Field | UDP Payload Offset | Unit |
|---|---|---|
| gyro_x / gyro_y / gyro_z | 36 / 40 / 44 | rad/s |
| acc_x / acc_y / acc_z | 48 / 52 / 56 | g |

Multiply acceleration by 9.80665 to convert it to m/s². Compute standard CRC32 over the 32 bytes beginning at offset 28, and compare it with the little-endian u32 at offset 24.

## 4 Building Control Frames

### Frame Fields

Commands are written as `command set/command ID`. For example, `32/12` represents the two bytes `0x32` and `0x12`. The offsets below are relative to the first byte of the control frame.

| Field | Offset | Bytes | Encoding |
|---|---|---|---|
| sof | 0 | 1 | `0x55` |
| length_lo | 1 | 1 | Low 8 bits of the complete frame length |
| version_length_hi | 2 | 1 | Low 2 bits contain the high length bits. The other 6 bits contain protocol version 1 |
| crc8 | 3 | 1 | Checksum of the first three frame bytes |
| sender | 4 | 1 | Use `0x28` in client requests |
| receiver | 5 | 1 | Use `0xAD` in client requests |
| seq_num | 6 | 2 | u16 request sequence number, echoed in the response |
| flags | 8 | 1 | Use `0x40` for requests. Response bit7 is set to 1 |
| cmd_set | 9 | 1 | Command set |
| cmd_id | 10 | 1 | Command ID |
| payload | 11 | n | Command payload |
| crc16 | 11+n | 2 | Checksum from sof through the payload end, stored little-endian |

The total length is `13 + n`. Decode the length using `byte[1] + ((byte[2] & 0x03) << 8)` and the protocol version using `byte[2] >> 2`. The field width can encode complete frames from 13 to 1023 bytes, with a maximum payload of 1010 bytes. Individual commands are also constrained by their own length and buffer requirements.

Use an incrementing request sequence number and handle u16 wraparound. On reception, check the source, frame length, and CRCs first. Then match the swapped endpoints, sequence number, command set, command ID, and response flag. Parse unsolicited messages separately.

### CRC

| Checksum | Initial Value | Normal Polynomial | Reflected Polynomial for Right Shifts | Final XOR | Coverage |
|---|---|---|---|---|---|
| Control CRC8 | `0x77` | `0x31` | `0x8C` | `0x00` | First three control frame bytes |
| Control CRC16 | `0x3692` | `0x1021` | `0x8408` | `0x0000` | From sof through the payload end |
| IMU CRC32 | `0xFFFFFFFF` | `0x04C11DB7` | `0xEDB88320` | `0xFFFFFFFF` | UDP payload offset 28 through the packet end |

For CRC8 and CRC16, use the following right-shift algorithm with the corresponding initial value and reflected polynomial.

```python
def reflected_crc(data, initial, polynomial):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            if value & 1:
                value = (value >> 1) ^ polynomial
            else:
                value >>= 1
    return value
```

For IMU CRC32, use `zlib.crc32(udp_payload[28:])`. Also check the packet length and data type.

### Command Reference

| Command | Direction | Purpose |
|---|---|---|
| `32/21` | Device → Broadcast | Device discovery |
| `01/02` | Device → Client | Status and configuration push |
| `32/12` | Client → Device | Read one configuration parameter |
| `32/42` | Client → Device | Read configuration parameters in a batch |
| `32/13` | Client → Device | Write one configuration parameter |
| `32/43` | Client → Device | Write configuration using the batch-format command |
| `32/50` | Client → Device | Set the target operating state |
| `32/51` | Client → Device | Read the target operating state |
| `32/14` | Client → Device | Set sampling enable |
| `00/0B` | Client → Device | Software reboot |
| `00/68` | Client → Device | External time synchronization reference command |
| `00/07` | Client → Device | Enter Loader update mode |

## 5 Reading and Writing Configuration

All offsets in this section are relative to the start of the command payload. UIDs are little-endian u32 values. Lengths must match the parameter tables in Sections 6 and 7.

### Read One Parameter with `32/12`

The request is 8 bytes long.

| Field | Offset | Type |
|---|---|---|
| uid | 0 | u32 |
| expected_len | 4 | u32 |

A successful response uses the following layout. Check ret_code and the actual payload length before reading the value.

| Field | Offset | Type |
|---|---|---|
| ret_code | 0 | u8, 0 on success |
| uid | 1 | u32, echoes the requested UID |
| length | 5 | u16 |
| reserved | 7 | u16 |
| value | 9 | length bytes |

The following complete example reads point cloud type `0x00010021` with an expected length of 1.

```text
TX  55 15 04 A9 | 28 AD | 10 01 | 40 | 32 12 | 21 00 01 00 01 00 00 00 | 2B D7
RX  55 17 04 38 | AD 28 | 10 01 | 80 | 32 12 | 00 21 00 01 00 01 00 00 00 01 | 06 5B
```

### Batch Read with `32/42`

The request contains `count u16`, followed by count `uid u32` values. The response header is 7 bytes long.

| Field | Offset | Type |
|---|---|---|
| ret_code | 0 | u8, 0 on success |
| error_key | 1 | u32, 0 on success |
| item_count | 5 | u16 |
| item_list | 7 | Each item contains `uid u32, length u16, value[length]` |

Parse each item using its own UID and length. Split long lists into smaller batches. Use `32/42` for batch configuration queries.

### Write One Parameter with `32/13`

| Field | Offset | Type |
|---|---|---|
| uid | 0 | u32 |
| length | 4 | u32 |
| value | 8 | length bytes |

The successful ACK payload is `00`. Write only parameters marked read/write in the tables. Read them back afterward and check the actual output at the receiving endpoint.

### Batch-Format Write with `32/43`

The single-item request format is shown below. For multiple parameters, use `32/13` to set and read back each parameter individually.

| Field | Offset | Type and Value |
|---|---|---|
| reserved | 0 | u16, set to 0 |
| item_count | 2 | u16, set to 1 for a single item |
| uid | 4 | u32 |
| length | 8 | u16 |
| value | 10 | length bytes |

The response payload contains `ret_code u8, error_key u32`. Five zero bytes indicate success. Reference return codes include `0x03` for a write failure and `0x07` for a format error. Read parameters back after an error or timeout, because some writes may already have taken effect.

The following example sets the yaw end angle of window 0 to 180°. Save the original angles and disable ROI before applying it.

```text
TX  55 1B 04 75 | 28 AD | 12 01 | 40 | 32 43 | 00 00 01 00 25 00 01 00 04 00 B4 00 00 00 | 39 60
RX  55 12 04 C7 | AD 28 | 12 01 | 80 | 32 43 | 00 00 00 00 00 | FC 99
```

## 6 Network Destinations and Runtime Parameters

### Streaming Destinations

| UID | Name | Length | Access | Meaning |
|---|---|---|---|---|
| `0x00010019` | host_msg_ip | 4 B | Read/write | Status push destination IPv4 |
| `0x0001001A` | host_msg_port | 2 B | Read/write | Status push destination port |
| `0x0001001B` | host_pcl_ip | 4 B | Read/write | Point cloud destination IPv4 |
| `0x0001001C` | host_pcl_port | 2 B | Read/write | Point cloud destination port |
| `0x0001001D` | host_imu_ip | 4 B | Read/write | IMU destination IPv4 |
| `0x0001001E` | host_imu_port | 2 B | Read/write | IMU destination port |

An IPv4 UID represents the dotted-decimal address as a u32 and stores it little-endian. For example, the documentation address `192.0.2.50` is encoded as `32 02 00 C0`. Port 60099 is encoded as the little-endian bytes `C3 EA`. IPv4 addresses in status push TLVs use dotted-decimal byte order, so the same address appears as `C0 00 02 32`.

Before setting a new destination, save the existing IP and port, then prepare a receiving socket bound to the new address. Write and read back each value individually. The intermediate configuration between the IP and port writes may temporarily interrupt reception.

### Runtime Parameters

| UID | Name | Length | Access | Meaning |
|---|---|---|---|---|
| `0x00010001` | dev_idx | 1 B | Read-only | Device index |
| `0x00010002` | ip_mode | 1 B | Read-only | IP address acquisition mode |
| `0x00010014` | imu_en | 1 B | Read-only | 0 disabled, 1 enabled |
| `0x00010015` | coord_type | 1 B | Read-only | Coordinate system type |
| `0x0001001F` | sample_mode | 1 B | Read-only | Sampling mode configuration |
| `0x00010020` | pattern_mode | 1 B | Read-only | Scan pattern, 0 for non-repetitive scanning |
| `0x00010021` | pcl_data_type | 1 B | Read-only | 1 for 32-bit Cartesian point cloud |
| `0x00010022` | tgt_state | 1 B | Read-only | Target state configuration parameter |

Read the runtime target state using `32/51 key=3`, and check status push key `0x8006` for the actual state. The configuration parameter `tgt_state` may differ from the runtime state.

## 7 Scan Windows and Angle Settings

ROI consists of two windows and a one-byte enable bitmask. Angle fields are little-endian int32 values in degrees. Yaw follows the device coordinate system, with +X at 0° and +Y at 90°.

### Parameters

| UID | Name | Length | Access | Meaning |
|---|---|---|---|---|
| `0x00010023` | roi_en | 1 B | Read/write | bit0 enables window 0, bit1 enables window 1 |
| `0x00010024` | roi_cfg0.yaw_start | 4 B | Read/write | Window 0 yaw start |
| `0x00010025` | roi_cfg0.yaw_stop | 4 B | Read/write | Window 0 yaw end |
| `0x00010026` | roi_cfg0.pitch_start | 4 B | Read/write | Window 0 pitch start |
| `0x00010027` | roi_cfg0.pitch_stop | 4 B | Read/write | Window 0 pitch end |
| `0x00010028` | roi_cfg0.rsvd | 4 B | Reserved | Save the readback and retain the original value |
| `0x00010029` | roi_cfg1.yaw_start | 4 B | Read/write | Window 1 yaw start |
| `0x0001002A` | roi_cfg1.yaw_stop | 4 B | Read/write | Window 1 yaw end |
| `0x0001002B` | roi_cfg1.pitch_start | 4 B | Read/write | Window 1 pitch start |
| `0x0001002C` | roi_cfg1.pitch_stop | 4 B | Read/write | Window 1 pitch end |
| `0x0001002D` | roi_cfg1.rsvd | 4 B | Reserved | Save the readback and retain the original value |

The single-window example uses `0 ≤ yaw_start < yaw_stop ≤ 360`. The pitch start angle is less than the end angle. The table uses an example pitch range of −7° to 52°. Read the current settings from the device before making changes.

| Setting | Window 0 Yaw | Window 1 Yaw | Pitch | roi_en |
|---|---|---|---|---|
| Disable software cropping | Retain original values | Retain original values | Retain original values | `0x00` |
| Horizontal range 0°–180° | 0° to 180° | Disabled | Window 0 from −7° to 52° | `0x01` |
| Across zero, 220°→170° | 220° to 360° | 0° to 170° | Both windows from −7° to 52° | `0x03` |

### Configuration Sequence

1. Read and save the original values of all 11 ROI parameters.
2. Use `32/13` to set `roi_en` to 0 and temporarily disable the software windows.
3. Write the yaw and pitch angles for the required windows, reading back each value.
4. Write the enable bitmask last. Use `1` for window 0, `2` for window 1, or `3` for both.
5. Filter placeholders and no-return sentinel points. Check valid returns inside and outside the windows, allowing angular tolerance near the boundaries.

For example, to set window 0 to a horizontal range of 0°–180° while retaining its pitch settings, write `0x00010023=0`, `0x00010024=0`, `0x00010025=180`, and `0x00010023=1` in that order. The enable bitmask is 1 byte, and each angle is 4 bytes.

The following complete example reads ROI enable with an expected length of 1.

```text
TX  55 15 04 A9 | 28 AD | 11 01 | 40 | 32 12 | 23 00 01 00 01 00 00 00 | E9 D9
RX  55 17 04 38 | AD 28 | 11 01 | 80 | 32 12 | 00 23 00 01 00 01 00 00 00 00 | 5F 99
```

Setting `roi_en=0` disables the software window restriction. Actual coverage also depends on the startup configuration, installation obstructions, and scene. To attempt a rollback, disable the windows, write back the saved angles, then restore the original enable bitmask. Check both readback and the actual point cloud.

## 8 Discovery and Status Messages

### Discovery `32/21`

Reference broadcast endpoints are `sender=0xAD, receiver=0x0A`, with flags set to `0x20`. The payload is 19 bytes long.

| Field | Payload Offset | Type |
|---|---|---|
| sn | 0 | uint8[14], device serial number |
| reserved | 14 | uint8[2] |
| flag | 16 | uint8, reference value `0x1A` |
| tail | 17 | uint16, reserved and may vary |

### Status Push `01/02`

The payload begins with `count u16` and two reserved bytes. Starting at offset 4, items are arranged as `key u16, length u16, value[length]`. Advance through the payload using each item's length, and skip unknown keys. The reference frame flags are `0x00`.

| key | Length | Type and Meaning |
|---|---|---|
| `0x0000` | 1 B | Point cloud type, 1 for Cartesian |
| `0x0001` | 1 B | Scan pattern, 0 for non-repetitive scanning |
| `0x0004` | 12 B | IP address, subnet mask, and gateway, four bytes each in dotted-decimal byte order |
| `0x0005` | 8 B | Configured status push destination |
| `0x0006` | 8 B | Configured point cloud destination |
| `0x0007` | 8 B | Configured IMU destination |
| `0x0015` | 20 B | Window 0, four int32 angles and four reserved bytes |
| `0x0016` | 20 B | Window 1, same layout |
| `0x0017` | 1 B | ROI enable bitmask |
| `0x001A` | 1 B | Target state configuration parameter |
| `0x001C` | 1 B | IMU enable |
| `0x8000` | 16 B | Serial number |
| `0x8001` | 64 B | Product information string |
| `0x8002` | 4 B | Application firmware version bytes |
| `0x8003` | 4 B | Loader firmware version bytes |
| `0x8004` | 4 B | Hardware version bytes |
| `0x8005` | 6 B | MAC address |
| `0x8006` | 1 B | Actual operating state |
| `0x8007` | 4 B | int32 temperature in units of 0.01 ℃ |
| `0x8008` | 4 B | uint32 power-up count |
| `0x8009` | 8 B | uint64 local time in ns |
| `0x800A` | 8 B | uint64 last synchronization time in ns |
| `0x800B` | 8 B | int64 time offset in ns |
| `0x800C` | 1 B | Time synchronization type |
| `0x800E` | 2 B | Module fault severity |
| `0x8010` | 1 B | Firmware type |
| `0x8011` | 32 B | Eight little-endian uint32 diagnostic codes. Interpret nonzero entries using the diagnostic documentation |

Each of the three 8-byte destination entries contains a four-byte IPv4 address, a two-byte little-endian port, and two reserved bytes. Status push keys and configuration UIDs use separate numbering systems. Configuration TLVs may also contain stale values. Redact device identity and network addresses according to the authorized disclosure scope before sharing logs.

The following push frame contains a single item, `0x8006=2`.

```text
RX  55 16 04 FC | AD 4D | 04 01 | 00 | 01 02 | 01 00 00 00 06 80 01 00 02 | 53 F3
```

## 9 Software Reboot and Time Synchronization

The `00/0B` request has an empty payload, and its successful response payload is `00`. After sending it, monitor the control interface, status messages, and data streams separately as they resume.

```text
TX  55 0D 04 33 | 28 AD | 03 01 | 40 | 00 0B | 42 75
RX  55 0E 04 66 | AD 28 | 03 01 | 80 | 00 0B | 00 | DB E5
```

After a reboot, recheck the network destinations, ROI, and state. Configuration readback may differ from actual runtime behavior. Check the output again when reapplying settings. Whether parameters persist across a power cycle depends on the device.

The reference payload for `00/68` is `channel u8, timestamp u64`, with the timestamp in µs. External time synchronization also depends on the channel, time origin, and pulse conditions. Select values according to the device configuration, and assess synchronization using timestamps and synchronization status.

## 10 Firmware Write Entry

Use `00/07` to enter write mode, with command set `0x00` and command ID `0x07`. Send an empty-payload request to device UDP port `60000`. A response payload of `00 00 00` indicates that the request has been accepted.

The complete request and response below use sequence number `0x4810`.

```text
TX  55 0D 04 33 | 28 AD | 10 48 | 40 | 00 07 | 76 F9
RX  55 10 04 56 | AD 28 | 10 48 | 80 | 00 07 | 00 00 00 | 0A 32
```
