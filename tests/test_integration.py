import contextlib
import io
import json
import math
import socket
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from t100_lidar.cli import main
from t100_lidar.client import Client
from t100_lidar.protocol import Frame, ProtocolError
from t100_lidar.recording import PcapWriter
from t100_lidar.simulator import Simulator
from t100_lidar.stream import StreamReceiver
from t100_lidar.web import DemoServer


def wait_for(predicate, timeout=4):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        result=predicate()
        if result:return result
        time.sleep(.03)
    raise AssertionError("condition was not observed before deadline")


class ResponseTests(unittest.TestCase):
    def test_wrong_sequence_and_corrupt_replies_are_ignored(self):
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);server.bind(("127.0.0.1",0));server.settimeout(1)
        def respond():
            raw,address=server.recvfrom(2048);request=Frame.decode(raw)
            server.sendto(b"bad",address)
            wrong=Frame(0,7,b"\0\0\0",(request.sequence+1)&65535,0xAD,0x28,0x80)
            server.sendto(wrong.encode(),address)
            server.sendto(request.reply(b"\0\0\0").encode(),address)
        worker=threading.Thread(target=respond);worker.start()
        try:
            with Client(*server.getsockname(),timeout=1) as client:
                self.assertTrue(client.enter_write_mode()["request_accepted"])
        finally:
            worker.join(2);server.close()

    def test_timeout_sends_once(self):
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);server.bind(("127.0.0.1",0));server.settimeout(.1)
        try:
            with Client(*server.getsockname(),timeout=.08) as client:
                with self.assertRaises(TimeoutError):client.enter_write_mode()
            self.assertEqual(Frame.decode(server.recvfrom(2048)[0]).command,"00/07")
            with self.assertRaises(socket.timeout):server.recvfrom(2048)
        finally:server.close()

    def test_cli_dry_run_and_parameter_listing(self):
        text=io.StringIO()
        with contextlib.redirect_stdout(text):self.assertEqual(main(["frame","enter-write-mode"]),0)
        value=json.loads(text.getvalue())
        self.assertFalse(value["sent"])
        self.assertEqual(value["hex"],"55 0D 04 33 28 AD 10 48 40 00 07 76 F9")
        text=io.StringIO()
        with contextlib.redirect_stdout(text):self.assertEqual(main(["parameters"]),0)
        self.assertEqual(len(json.loads(text.getvalue())),25)


class UDPIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.receiver=StreamReceiver().start()
        self.sim=Simulator(self.receiver.addresses).start()
        self.client=Client(*self.sim.address,timeout=1)
    def tearDown(self):
        self.client.close();self.sim.close();self.receiver.close()

    def test_start_roi_stop_over_real_loopback_udp(self):
        self.assertEqual(self.client.start(),1)
        wait_for(lambda:self.receiver.snapshot()["point_packets"]>=8)
        wait_for(lambda:self.receiver.snapshot()["actual_state"]==1)
        self.assertGreater(self.receiver.snapshot()["valid_points"],0)
        with tempfile.TemporaryDirectory() as directory:
            backup=Path(directory)/"roi.json"
            result=self.client.set_roi(0,90,backup)
            self.assertEqual(result["after"]["roi_en"],1)
            self.assertEqual(json.loads(backup.read_text())["parameters"]["roi_en"],0)
            with self.assertRaises(FileExistsError):self.client.set_roi(0,180,backup)
        self.receiver.clear_points()
        wait_for(lambda:self.receiver.snapshot()["points"])
        time.sleep(.15)
        points=self.receiver.snapshot()["points"]
        self.assertTrue(all(-.2<=math.degrees(math.atan2(p[1],p[0]))<=90.2 for p in points))
        self.assertEqual(self.client.stop(),2)
        wait_for(lambda:self.receiver.snapshot()["actual_state"]==2)
        time.sleep(.2)
        count=self.receiver.snapshot()["point_packets"];time.sleep(.2)
        state=self.receiver.snapshot()
        self.assertEqual(count,state["point_packets"])
        self.assertEqual(state["bad_packets"],0)
        self.assertTrue(state["imu"]["crc32_valid"])

    def test_readonly_and_invalid_roi_do_not_write(self):
        with self.assertRaises(ValueError):self.client.write("pcl_data_type",2)
        with self.assertRaises(ValueError):self.client.set_roi(180,90)
        self.assertEqual(self.client.read("pcl_data_type"),1)
        self.assertEqual(self.client.read("roi_en"),0)

    def test_synthetic_room_has_vertical_wall_coverage(self):
        from t100_lidar.protocol import decode_data
        points=[]
        for _ in range(10):points.extend(decode_data(self.sim._point_packet()).points)
        wall=[p.z for p in points if abs(p.x-7500)<2]
        self.assertGreater(len(wall),40)
        self.assertGreater(max(wall)-min(wall),4000)
        self.assertGreater(len({round(z/100) for z in wall}),20)

    def test_write_mode_entry_is_acknowledged_by_mock_only(self):
        self.assertTrue(self.client.enter_write_mode()["request_accepted"])
        self.assertTrue(self.sim.write_mode)

    def test_mock_destination_writes_are_local_and_route_packets(self):
        from t100_lidar.client import DeviceError
        with self.assertRaises(DeviceError):self.client.write("host_pcl_ip","192.0.2.10")
        self.assertEqual(self.client.read("host_pcl_ip"),"127.0.0.1")
        alternate=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        alternate.bind(("127.0.0.1",0));alternate.settimeout(1)
        try:
            self.client.write("host_pcl_port",alternate.getsockname()[1])
            self.client.start()
            self.assertEqual(len(alternate.recvfrom(2048)[0]),1380)
        finally:alternate.close()

    def test_pcap_records_have_valid_ipv4_header(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"sample.pcap"
            writer=PcapWriter(path)
            writer.write(bytes.fromhex("550d043328ad104840000776f9"),("127.0.0.1",40000),("127.0.0.1",60000))
            writer.close()
            data=path.read_bytes()
            self.assertEqual(struct.unpack_from("<I",data,20)[0],101)
            ip=data[40:60];total=sum(struct.unpack("!10H",ip))
            while total>>16:total=(total&65535)+(total>>16)
            self.assertEqual(total,65535)
            self.assertEqual(data[68:],bytes.fromhex("550d043328ad104840000776f9"))


class HTTPIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server=DemoServer(0)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
        cls.url=f"http://127.0.0.1:{cls.server.server_address[1]}"
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close();cls.thread.join(2)
    def request(self,path="/api/snapshot",body=None,origin=None):
        headers={"Content-Type":"application/json"}
        if origin:headers["Origin"]=origin
        request=Request(self.url+path,data=json.dumps(body).encode() if body is not None else None,headers=headers)
        return urlopen(request,timeout=3)
    def test_snapshot_and_exports(self):
        wait_for(lambda:self.server.demo.receiver.snapshot()["point_packets"]>0)
        with self.request() as response:
            state=json.load(response)
            self.assertEqual(state["mode"],"simulation")
            self.assertGreater(len(state["points"]),0)
        with self.request("/export/points.csv") as response:
            self.assertTrue(response.read().startswith(b"x_m,y_m,z_m,reflectivity,tag"))
    def test_enter_write_mode_and_resume_demo(self):
        try:
            with self.server.demo.event_lock:self.server.demo.events.clear()
            with self.request("/api/control",{"action":"enter-write-mode"}) as response:
                self.assertTrue(json.load(response)["write_mode"])
            self.assertTrue(self.server.demo.simulator.write_mode)
            with self.request() as response:
                snapshot=json.load(response)
                self.assertTrue(snapshot["write_mode"])
                self.assertIsNone(snapshot["target_state"])
                self.assertIsNone(snapshot["actual_state"])
                self.assertEqual([(event["direction"],event["command"]) for event in snapshot["events"]],[("RX","00/07"),("TX","00/07")])
                request=Frame.decode(bytes.fromhex(snapshot["events"][1]["hex"]))
                self.assertEqual(request.payload,b"")
            with self.assertRaises(HTTPError):self.request("/api/control",{"action":"start"})
        finally:
            with self.request("/api/control",{"action":"resume-demo"}) as response:
                self.assertTrue(json.load(response)["simulated"])
        self.assertFalse(self.server.demo.simulator.write_mode)

    def test_unknown_post_is_rejected_before_body_processing(self):
        request=Request(self.url+"/unused",data=b"unused bytes",headers={"Content-Type":"application/octet-stream"})
        with self.assertRaises(HTTPError) as error:urlopen(request,timeout=3)
        self.assertEqual(error.exception.code,404)

    def test_snapshot_has_only_selected_public_fields(self):
        expected={"point_packets","imu_packets","status_packets","valid_points","filtered_points","bad_packets","bytes","points_per_second","point_age_ms","uptime_s","points","imu","actual_state","sequence","mode","write_mode","target_state","roi","events","control_address"}
        with self.request() as response:self.assertEqual(set(json.load(response)),expected)

    def test_invalid_controls_and_cross_origin(self):
        for body in ({"action":"roi","start":180,"end":90},{"action":"roi","start":True,"end":90},{"action":"real-device","ip":"192.0.2.10"}):
            with self.assertRaises(HTTPError) as exc:self.request("/api/control",body)
            self.assertEqual(exc.exception.code,400)
        with self.assertRaises(HTTPError) as exc:self.request("/api/control",{"action":"stop"},"https://example.org")
        self.assertEqual(exc.exception.code,403)
    def test_static_files_and_no_traversal(self):
        with self.request("/") as response:
            self.assertIn(b"T100 LiDAR Toolkit",response.read())
            self.assertIn("frame-ancestors 'none'",response.headers["Content-Security-Policy"])
        with self.assertRaises(HTTPError) as exc:self.request("/../../pyproject.toml")
        self.assertEqual(exc.exception.code,404)


if __name__=="__main__":unittest.main()
