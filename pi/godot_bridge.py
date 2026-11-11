import json
import socket
import time
import numpy as np

import config as C
from drone.flight import (
    set_selected_point,
    clear_selected_point,
    request_launch,
    request_land,
)
from web_gcs import start_web_gcs_server, update_web_telemetry


class GodotBridge:
    def __init__(self, host=None, port=None, cmd_port=None, web_port=8080):
        self.addr = (
            host or getattr(C, "GODOT_HOST", "127.0.0.1"),
            int(port or getattr(C, "GODOT_PORT", 9999)),
        )
        self.cmd_port = int(cmd_port or getattr(C, "GODOT_CMD_PORT", 9998))
        self.web_port = int(web_port) if web_port else 0
        self.web_server = start_web_gcs_server("0.0.0.0", self.web_port) if self.web_port > 0 else None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.cmd_sock.bind(("0.0.0.0", self.cmd_port))
            self.cmd_sock.setblocking(False)
            self._cmd_listening = True
        except OSError:
            self._cmd_listening = False
        self.run_id = int(time.time() * 1000) & 0x7FFFFFFF
        self.seq = 0
        self.obstacles = [[float(v) for v in o] for o in C.OBSTACLES]

    def new_run(self):
        self.run_id = (self.run_id + 1) & 0x7FFFFFFF

    def poll_commands(self):
        if not self._cmd_listening:
            return
        while True:
            try:
                data, _ = self.cmd_sock.recvfrom(4096)
            except (BlockingIOError, OSError):
                break
            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue
            if not isinstance(msg, dict):
                continue
            cmd = msg.get("cmd", "")
            if cmd == "select_point" and "pos" in msg:
                p = msg["pos"]
                rad = msg.get("radius", None)
                mode = msg.get("mode", "loiter")
                set_selected_point(p[0], p[1], p[2] if len(p) > 2 else None, radius=rad, mode=mode)
            elif cmd == "clear_point":
                clear_selected_point()
            elif cmd == "start_flight":
                request_launch()
            elif cmd == "land":
                request_land()

    def send(self, world, collision=False, hit_idx=-1):
        self.poll_commands()
        quad, target = world.quad, world.target
        cmd_target = world.ctrl.cmd_target
        loiter_center = getattr(world, "loiter_center", target)
        loiter_radius = float(getattr(world, "loiter_radius", C.CIRCLE_RADIUS))
        surfaces = getattr(quad, "surfaces", [0.5, 0.0, 0.0, 0.0])
        packet = {
            "run": self.run_id,
            "seq": self.seq,
            "t": round(float(world.t), 4),
            "pos": [float(v) for v in quad.pos],
            "vel": [float(v) for v in quad.vel],
            "att": [float(v) for v in quad.att],
            "rot": [float(v) for v in quad.rotation_matrix().ravel()],
            "target": [float(v) for v in target],
            "cmd_target": [float(v) for v in (target if cmd_target is None else cmd_target)],
            "loiter_center": [float(v) for v in loiter_center],
            "loiter_radius": loiter_radius,
            "airspeed": float(getattr(quad, "airspeed", np.linalg.norm(quad.vel))),
            "alpha_deg": float(np.degrees(getattr(quad, "alpha", 0.0))),
            "beta_deg": float(np.degrees(getattr(quad, "beta", 0.0))),
            "surfaces": [float(v) for v in surfaces],
            "thrust": float(world.thrust_cmd),
            "motors": [float(v) for v in quad.motor_thrust],
            "motor_max": float(C.MOTOR_MAX_THRUST),
            "hover_motor": float(quad.mass * C.GRAVITY / 4.0),
            "saturated": bool(quad.saturated),
            "wind": [float(v) for v in world.wind.value],
            "collision": bool(collision),
            "hit": int(hit_idx if hit_idx is not None else -1),
            "obstacles": [[float(v) for v in o] for o in C.OBSTACLES],
            "drone_radius": float(C.DRONE_RADIUS),
            "mode": str(getattr(C, "FLIGHT_MODE", "loiter")),
            "phase": str(getattr(world, "phase", "FLYING")),
            "disarmed": bool(getattr(world, "disarmed", False)),
            "battery": float(getattr(quad, "battery", 1.0)),
            "home_gps": getattr(quad, "home_gps", None),
        }
        self.seq += 1
        update_web_telemetry(packet)
        try:
            self.sock.sendto(json.dumps(packet).encode("utf-8"), self.addr)
        except OSError:
            pass

    def close(self):
        self.sock.close()
        if self._cmd_listening:
            self.cmd_sock.close()
        if self.web_server is not None:
            self.web_server.shutdown()
