"""Command-line tools for the documented Ethernet interface."""

import argparse
import json
import socket
import struct
import sys
import time

from . import __version__
from .client import Client, DeviceError
from .parameters import PARAMETERS, parameter
from .protocol import Frame, ProtocolError, discovery_info
from .recording import PcapWriter
from .stream import StreamReceiver


def integer(text):
    return int(text, 0)


def output(value):
    print(json.dumps(value, indent=2, ensure_ascii=False))


def parser():
    root = argparse.ArgumentParser(prog="t100", description="T100 LiDAR protocol toolkit; demo mode uses a local simulated device.")
    root.add_argument("--version", action="version", version=__version__)
    root.add_argument("--ip", help="device IPv4 for explicit control commands")
    root.add_argument("--control-port", type=int, default=60000)
    root.add_argument("--timeout", type=float, default=1.0)
    root.add_argument("--trace", action="store_true", help="write full TX/RX hex to stderr")
    subs = root.add_subparsers(dest="command", required=True)
    demo = subs.add_parser("demo", help="run simulator and browser preview using loopback UDP")
    demo.add_argument("--port", type=int, default=18766)
    demo.add_argument("--open", action="store_true", help="open the local preview in a browser")
    subs.add_parser("parameters", help="list all 25 published UIDs")
    discover = subs.add_parser("discover", help="passively listen for discovery broadcasts")
    discover.add_argument("--bind", default="0.0.0.0")
    discover.add_argument("--port", type=int, default=50000)
    discover.add_argument("--seconds", type=float, default=5)
    for name, help_text in (("status", "read runtime target state and ROI configuration"), ("start", "set sampling target and enable sampling"), ("stop", "disable sampling and set standby target"), ("reboot", "request software reboot"), ("enter-write-mode", "send 00/07 to enter firmware write mode")):
        subs.add_parser(name, help=help_text)
    read = subs.add_parser("read", help="read one UID by name or hexadecimal number")
    read.add_argument("parameter")
    write = subs.add_parser("write", help="write one published writable UID, then read it back")
    write.add_argument("parameter")
    write.add_argument("value")
    roi = subs.add_parser("roi", help="read or set a single yaw window, with a before-write backup")
    roi.add_argument("--start", type=int)
    roi.add_argument("--end", type=int)
    roi.add_argument("--disable", action="store_true")
    roi.add_argument("--backup", help="new JSON file for original ROI values; required for changes")
    frame = subs.add_parser("frame", help="generate a complete control frame without sending it")
    frame.add_argument("name", choices=("enter-write-mode", "reboot", "sampling-on", "sampling-off", "target-sampling", "target-standby", "target-query", "read", "time-sync"))
    frame.add_argument("--sequence", type=integer, default=0x4810)
    frame.add_argument("--parameter")
    frame.add_argument("--channel", type=int, default=0)
    frame.add_argument("--timestamp-us", type=integer, default=0)
    capture = subs.add_parser("capture", help="listen for point cloud/IMU and optionally record datagrams as PCAP")
    capture.add_argument("--bind", default="0.0.0.0")
    capture.add_argument("--point-port", type=int, default=60001)
    capture.add_argument("--imu-port", type=int, default=60003)
    capture.add_argument("--status-port", type=int, default=0)
    capture.add_argument("--seconds", type=float, default=10)
    capture.add_argument("--pcap", help="new output file; existing files are never overwritten")
    return root


def generated_frame(args):
    commands = {"enter-write-mode": (0, 7, b""), "reboot": (0, 0x0B, b""),
                "sampling-on": (0x32, 0x14, b"\x01"), "sampling-off": (0x32, 0x14, b"\0"),
                "target-sampling": (0x32, 0x50, struct.pack("<HIHB", 1, 3, 1, 1)),
                "target-standby": (0x32, 0x50, struct.pack("<HIHB", 1, 3, 1, 2)),
                "target-query": (0x32, 0x51, struct.pack("<HI", 1, 3))}
    if args.name == "read":
        if args.parameter is None:
            raise ValueError("frame read requires --parameter")
        p = parameter(args.parameter)
        command = (0x32, 0x12, struct.pack("<II", p.uid, p.size))
    elif args.name == "time-sync":
        command = (0, 0x68, struct.pack("<BQ", args.channel, args.timestamp_us))
    else:
        command = commands[args.name]
    frame = Frame(*command, sequence=args.sequence)
    return {"command": frame.command, "sequence": frame.sequence, "hex": frame.encode().hex(" ").upper(), "sent": False}


def trace(direction, frame, raw):
    print(f"{direction} {frame.command} seq={frame.sequence:#06x} {raw.hex(' ').upper()}", file=sys.stderr)


def listen_discovery(args):
    if args.seconds <= 0:
        raise ValueError("seconds must be positive")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    found = {}
    try:
        sock.bind((args.bind, args.port))
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            sock.settimeout(max(0.001, deadline - time.monotonic()))
            try:
                data, source = sock.recvfrom(65535)
                info = discovery_info(Frame.decode(data))
                found[(source[0], info["serial"])] = {"ip": source[0], "source_port": source[1], **info}
            except socket.timeout:
                break
            except ProtocolError:
                continue
    finally:
        sock.close()
    return list(found.values())


def capture(args):
    if args.seconds <= 0:
        raise ValueError("seconds must be positive")
    writer = PcapWriter(args.pcap) if args.pcap else None
    receiver = None
    try:
        receiver = StreamReceiver(args.bind, args.point_port, args.imu_port, args.status_port,
                                  on_datagram=writer.write if writer else None, source_ip=args.ip).start()
        print("Listening on " + ", ".join(f"{host}:{port}" for host, port in receiver.addresses), file=sys.stderr)
        deadline = time.monotonic() + args.seconds
        try:
            while time.monotonic() < deadline:
                time.sleep(min(0.1, max(0, deadline - time.monotonic())))
        except KeyboardInterrupt:
            pass
        state = receiver.snapshot()
        state.pop("points")
        return state
    finally:
        if receiver:
            receiver.close()
        if writer:
            writer.close()


def main(argv=None):
    root = parser()
    args = root.parse_args(argv)
    try:
        if args.command == "demo":
            from .web import serve_demo
            serve_demo(args.port, args.open)
            return 0
        if args.command == "frame":
            output(generated_frame(args))
            return 0
        if args.command == "parameters":
            output([{"uid": f"0x{p.uid:08X}", "name": p.name, "bytes": p.size, "writable": p.writable} for p in PARAMETERS])
            return 0
        if args.command == "discover":
            output(listen_discovery(args))
            return 0
        if args.command == "capture":
            output(capture(args))
            return 0
        if not args.ip:
            root.error("this control command requires --ip DEVICE_IPV4")
        with Client(args.ip, args.control_port, args.timeout, trace=trace if args.trace else None) as client:
            if args.command == "status":
                result = {"target_state": client.target_state(), "roi": client.roi(), "actual_state": "receive status push key 0x8006 separately"}
            elif args.command == "read":
                result = {parameter(args.parameter).name: client.read(args.parameter)}
            elif args.command == "write":
                result = {parameter(args.parameter).name: client.write(args.parameter, args.value)}
            elif args.command in ("start", "stop"):
                result = {"target_state_readback": getattr(client, args.command)(), "note": "check status push and actual point cloud/IMU separately"}
            elif args.command == "reboot":
                client.reboot()
                result = {"request_accepted": True}
            elif args.command == "enter-write-mode":
                result = client.enter_write_mode()
            elif args.command == "roi":
                changing = args.disable or args.start is not None or args.end is not None
                if not changing:
                    result = client.roi()
                elif not args.backup:
                    raise ValueError("ROI changes require --backup NEW_FILE.json to save original settings")
                elif args.disable:
                    if args.start is not None or args.end is not None:
                        raise ValueError("--disable cannot be combined with --start/--end")
                    from pathlib import Path
                    old = client.roi()
                    with Path(args.backup).open("x", encoding="utf-8") as stream:
                        json.dump({"device_ip": args.ip, "parameters": old}, stream, indent=2)
                    result = {"before": old, "roi_en": client.write("roi_en", 0)}
                elif args.start is None or args.end is None:
                    raise ValueError("specify both --start and --end")
                else:
                    result = client.set_roi(args.start, args.end, args.backup)
            else:
                raise ValueError("unknown command")
            output(result)
        return 0
    except (OSError, ValueError, DeviceError, ProtocolError, struct.error) as exc:
        print(f"t100: {exc}", file=sys.stderr)
        return 1
