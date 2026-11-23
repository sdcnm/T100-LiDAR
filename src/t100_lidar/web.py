"""Loopback-only demonstration server. There is no real-device web endpoint."""

import collections
import csv
import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from urllib.parse import urlsplit

from .client import Client
from .simulator import Simulator
from .stream import StreamReceiver

class Demo:
    def __init__(self):
        self.events = collections.deque(maxlen=100)
        self.event_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.quiet_trace = False
        self.receiver = StreamReceiver().start()
        self.simulator = Simulator(self.receiver.addresses).start()
        self.client = Client(*self.simulator.address, bind="127.0.0.1", trace=self._trace)
        self.started_at = time.monotonic()
        self.client.start()
        self.last_roi = self.client.roi()

    def _trace(self, direction, frame, raw):
        if self.quiet_trace:
            return
        event = {"time": time.strftime("%H:%M:%S"), "direction": direction, "command": frame.command, "sequence": frame.sequence, "hex": raw.hex(" ").upper(), "bytes": len(raw)}
        with self.event_lock:
            self.events.appendleft(event)

    def close(self):
        self.simulator.close()
        self.receiver.close()
        self.client.close()

    def snapshot(self):
        with self.operation_lock:
            self.quiet_trace = True
            try:
                write_mode = self.simulator.write_mode
                if write_mode:
                    roi, target = self.last_roi, None
                else:
                    roi = self.last_roi = self.client.roi()
                    target = self.client.target_state()
            finally:
                self.quiet_trace = False
        state = self.receiver.snapshot()
        if write_mode:
            state["actual_state"] = None
        stride = max(1, math.ceil(len(state["points"]) / 9000))
        state["points"] = state["points"][::stride]
        with self.event_lock:
            events = list(self.events)
        return {**state, "mode": "simulation", "write_mode": write_mode,
                "target_state": target, "roi": roi, "events": events[:24],
                "control_address": f"{self.simulator.address[0]}:{self.simulator.address[1]}"}

    def control(self, action, body):
        with self.operation_lock:
            if action == "enter-write-mode":
                if not self.simulator.write_mode:
                    self.client.enter_write_mode()
                    self.receiver.clear_points()
                result = {"write_mode": True, "simulated": True}
            elif action == "resume-demo":
                # Reset the local model; this sends no device recovery command.
                self.simulator.write_mode = False
                self.simulator.enabled = False
                self.simulator.target = 2
                result = {"target_state": self.client.start(), "simulated": True}
            elif self.simulator.write_mode:
                raise ValueError("simulated device is in firmware write mode; resume the demo first")
            elif action == "start":
                result = {"target_state": self.client.start()}
            elif action == "stop":
                result = {"target_state": self.client.stop()}
            elif action == "roi":
                start, end = body.get("start"), body.get("end")
                if type(start) is not int or type(end) is not int:
                    raise ValueError("angles must be integers")
                result = self.client.set_roi(start, end)
                self.receiver.clear_points()
            elif action == "full":
                result = {"roi_en": self.client.write("roi_en", 0)}
                self.receiver.clear_points()
            elif action == "read":
                result = self.client.roi()
            else:
                raise ValueError("unknown demo action")
        return result

class DemoServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, port=18766):
        super().__init__(("127.0.0.1", port), Handler)
        try:
            self.demo = Demo()
        except Exception:
            self.server_close()
            raise

    def server_close(self):
        if hasattr(self, "demo"):
            self.demo.close()
        super().server_close()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def _reply(self, status, content, content_type="application/json; charset=utf-8", filename=None):
        if not isinstance(content, bytes):
            content = content.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; connect-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; frame-ancestors 'none'")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self):
        path = urlsplit(self.path).path
        try:
            if path == "/api/snapshot":
                self._reply(200, json.dumps(self.server.demo.snapshot(), allow_nan=False))
            elif path == "/export/points.csv":
                snapshot = self.server.demo.receiver.snapshot()
                output = io.StringIO(newline="")
                writer = csv.writer(output)
                writer.writerow(["x_m", "y_m", "z_m", "reflectivity", "tag"])
                writer.writerows(snapshot["points"])
                self._reply(200, output.getvalue(), "text/csv; charset=utf-8", "t100-simulated-points.csv")
            elif path == "/export/session.json":
                self._reply(200, json.dumps(self.server.demo.snapshot(), indent=2), filename="t100-simulated-session.json")
            elif path in ("/", "/app.js", "/style.css"):
                name = "index.html" if path == "/" else path[1:]
                mime = {"index.html": "text/html", "app.js": "text/javascript", "style.css": "text/css"}[name]
                content = files("t100_lidar").joinpath("static", name).read_bytes()
                self._reply(200, content, mime + "; charset=utf-8")
            else:
                self._reply(404, '{"error":"not found"}')
        except (ValueError, OSError, RuntimeError) as exc:
            self._reply(502, json.dumps({"error": str(exc)}))

    def do_POST(self):
        if urlsplit(self.path).path != "/api/control":
            self._reply(404, '{"error":"not found"}')
            return
        port = self.server.server_address[1]
        allowed = {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}
        if self.headers.get("Origin") not in allowed | {None}:
            self._reply(403, '{"error":"cross-origin control requests are not allowed"}')
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 4096:
                raise ValueError("expected a small JSON request")
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError("expected a JSON object")
            result = self.server.demo.control(body.get("action"), body)
            self._reply(200, json.dumps(result, allow_nan=False))
        except (ValueError, TypeError) as exc:
            self._reply(400, json.dumps({"error": str(exc)}))
        except (OSError, RuntimeError) as exc:
            self._reply(502, json.dumps({"error": str(exc)}))


def serve_demo(port=18766, open_browser=False):
    server = DemoServer(port)
    print(f"T100 local demo: http://127.0.0.1:{server.server_address[1]}", flush=True)
    print("SIMULATED DATA · loopback UDP peer · Ctrl+C to stop", flush=True)
    if open_browser:
        import webbrowser
        webbrowser.open(f"http://127.0.0.1:{server.server_address[1]}")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
