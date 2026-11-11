#!/usr/bin/env python3
import argparse
import math
import time
from types import SimpleNamespace
import numpy as np

import config as C
from drone import FlightController, World
from drone.flight import (
    uav_target_for_state,
    get_loiter_radius,
    consume_launch_request,
    is_land_requested,
    clear_land_request,
)
from godot_bridge import GodotBridge


def _wrap_pi(a):
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def ned_to_sim_vec(north, east, down):
    return np.array([float(east), float(north), -float(down)], dtype=float)


def sim_to_ned_vec(x_east, y_north, z_up):
    return float(y_north), float(x_east), -float(z_up)


def euler_to_quat_ned(roll_sim, pitch_sim, yaw_sim):
    r = float(roll_sim)
    p = -float(pitch_sim)
    y = _wrap_pi(0.5 * math.pi - float(yaw_sim))
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def rpy_to_rotmat(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


class MavlinkHardwareInterface:
    def __init__(self, device, baud=921600):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise SystemExit("pip install pymavlink") from exc

        self.mavutil = mavutil
        self.conn = mavutil.mavlink_connection(device, baud=baud)
        self.conn.wait_heartbeat()

        try:
            self.conn.mav.request_data_stream_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                50,
                1,
            )
        except Exception:
            pass

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.att = np.zeros(3)
        self.rate = np.zeros(3)
        self.airspeed = 0.0
        self.battery = 1.0
        self.surfaces = [0.0, 0.0, 0.0, 0.0]
        self._home_lat_lon_alt = None

    def reset_home_here(self):
        self._home_lat_lon_alt = None

    def poll_state(self):
        while True:
            msg = self.conn.recv_match(
                type=[
                    "LOCAL_POSITION_NED",
                    "GLOBAL_POSITION_INT",
                    "ATTITUDE",
                    "VFR_HUD",
                    "SERVO_OUTPUT_RAW",
                    "SYS_STATUS",
                ],
                blocking=False,
            )
            if msg is None:
                break
            mtype = msg.get_type()

            if mtype == "LOCAL_POSITION_NED" and self._home_lat_lon_alt is None:
                self.pos = ned_to_sim_vec(msg.x, msg.y, msg.z)
                self.vel = ned_to_sim_vec(msg.vx, msg.vy, msg.vz)
            elif mtype == "GLOBAL_POSITION_INT":
                lat = msg.lat * 1e-7
                lon = msg.lon * 1e-7
                alt = msg.relative_alt * 1e-3
                if self._home_lat_lon_alt is None and (lat != 0.0 or lon != 0.0):
                    self._home_lat_lon_alt = (lat, lon, alt)
                if self._home_lat_lon_alt is not None:
                    lat0, lon0, alt0 = self._home_lat_lon_alt
                    north_m = math.radians(lat - lat0) * 6378137.0
                    east_m = math.radians(lon - lon0) * 6378137.0 * math.cos(math.radians(lat0))
                    self.pos = np.array([east_m, north_m, alt - alt0], dtype=float)
                    self.vel = ned_to_sim_vec(msg.vx * 0.01, msg.vy * 0.01, msg.vz * 0.01)
            elif mtype == "ATTITUDE":
                yaw_sim = _wrap_pi(0.5 * math.pi - float(msg.yaw))
                self.att = np.array([float(msg.roll), -float(msg.pitch), yaw_sim], dtype=float)
                self.rate = np.array([float(msg.rollspeed), -float(msg.pitchspeed), -float(msg.yawspeed)], dtype=float)
            elif mtype == "VFR_HUD":
                va = float(msg.airspeed) if msg.airspeed > 0.2 else float(msg.groundspeed)
                self.airspeed = max(va, 0.0)
                self.surfaces[0] = float(np.clip(msg.throttle / 100.0, 0.0, 1.0))
            elif mtype == "SERVO_OUTPUT_RAW":
                self.surfaces[1] = float(np.clip((msg.servo1_raw - 1500.0) / 500.0, -1.0, 1.0))
                self.surfaces[2] = float(np.clip((msg.servo2_raw - 1500.0) / 500.0, -1.0, 1.0))
                self.surfaces[0] = float(np.clip((msg.servo3_raw - 1000.0) / 1000.0, 0.0, 1.0))
                self.surfaces[3] = float(np.clip((msg.servo4_raw - 1500.0) / 500.0, -1.0, 1.0))
            elif mtype == "SYS_STATUS":
                if getattr(msg, "battery_remaining", -1) >= 0:
                    self.battery = float(msg.battery_remaining) / 100.0

        R = rpy_to_rotmat(*self.att)
        vb = R.T @ self.vel
        va = max(float(np.linalg.norm(vb)), self.airspeed, 0.5)
        alpha = math.atan2(-vb[2], max(abs(vb[0]), 0.5))
        beta = math.asin(float(np.clip(vb[1] / va, -0.99, 0.99)))

        return SimpleNamespace(
            pos=self.pos.copy(),
            vel=self.vel.copy(),
            att=self.att.copy(),
            rate=self.rate.copy(),
            airspeed=va,
            alpha=alpha,
            beta=beta,
            alpha_trim=0.045,
            surfaces=list(self.surfaces),
            mass=float(C.MASS),
            inertia=np.array(C.INERTIA, dtype=float),
            motor_thrust=np.array([self.surfaces[0] * C.MAX_THRUST, 0.0, 0.0, 0.0]),
            saturated=bool(self.surfaces[0] >= 0.98),
            battery=self.battery,
            home_gps=(
                [self._home_lat_lon_alt[0], self._home_lat_lon_alt[1]]
                if self._home_lat_lon_alt is not None
                else None
            ),
            rotation_matrix=lambda: R,
        )

    def send_fixed_wing_attitude(self, roll_des, pitch_des, yaw_des, thrust_n):
        throttle = float(np.clip(thrust_n / max(float(C.MAX_THRUST), 1e-3), 0.0, 1.0))
        q = euler_to_quat_ned(roll_des, pitch_des, yaw_des)
        self.surfaces[0] = throttle
        self.conn.mav.set_attitude_target_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.conn.target_system,
            self.conn.target_component,
            0b00000111,
            q,
            0.0,
            0.0,
            0.0,
            throttle,
        )

    def send_fixed_wing_position_target(self, target_sim, airspeed_des=None):
        n, e, d = sim_to_ned_vec(target_sim[0], target_sim[1], target_sim[2])
        self.conn.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.conn.target_system,
            self.conn.target_component,
            self.mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            n,
            e,
            d,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        if airspeed_des is not None:
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                self.mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                0,
                float(airspeed_des),
                -1,
                0,
                0,
                0,
                0,
            )


def run_sim_dry_run(godot_host=None, godot_port=None, web_port=8080, speed=1.0, loop=False):
    C.FLIGHT_MODE = "loiter"
    bridge = GodotBridge(godot_host, godot_port, web_port=web_port)
    print("=" * 64)
    print("  Pi 5 UAV Bridge — SIM MODE")
    print(f"  Godot UDP : udp://{bridge.addr[0]}:{bridge.addr[1]}")
    if bridge.web_server is not None:
        print(f"  Web UI    : http://127.0.0.1:{bridge.web_port}")
    print("=" * 64)

    try:
        frame_dt = C.DT * C.STEPS_PER_FRAME
        auto_launch = bridge.web_server is None
        while True:
            world = World(resolve_obstacles=True, auto_launch=auto_launch)
            bridge.new_run()
            last_phase = None

            while world.t < world.duration or bridge.web_server is not None:
                frame_start = time.perf_counter()
                hit, hit_idx = False, -1
                for _ in range(C.STEPS_PER_FRAME):
                    hit, hit_idx = world.step(C.DT)
                    phase_key = world.phase.split("(")[0].strip()
                    if phase_key != last_phase:
                        last_phase = phase_key
                        p = world.quad.pos
                        print(
                            f"  [t={world.t:5.1f}s] {world.phase:28s} "
                            f"| pos=({p[0]:+5.2f}, {p[1]:+5.2f}, {p[2]:5.2f}) m "
                            f"| Va={world.quad.airspeed:4.2f} m/s"
                        )
                    if hit:
                        break

                bridge.send(world, hit, hit_idx)
                if hit:
                    break

                if speed > 0:
                    elapsed = time.perf_counter() - frame_start
                    sleep_s = (frame_dt / speed) - elapsed
                    if sleep_s > 0:
                        time.sleep(sleep_s)

            if not loop:
                break
            time.sleep(1.5)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()


def run_hardware_mavlink(
    device,
    baud,
    control_mode="attitude",
    rate_hz=50.0,
    godot_host=None,
    godot_port=None,
    web_port=8080,
):
    C.FLIGHT_MODE = "loiter"
    hw = MavlinkHardwareInterface(device, baud=baud)
    ctrl = FlightController()
    bridge = GodotBridge(godot_host, godot_port, web_port=web_port)
    bridge.new_run()
    dt = 1.0 / rate_hz

    origin = hw.poll_state().pos.copy()
    print("=" * 64)
    print("  Pi 5 UAV Bridge — PIXHAWK HARDWARE MODE")
    print(f"  MAVLink : {device} @ {baud}")
    if bridge.web_server is not None:
        print(f"  Web UI  : http://0.0.0.0:{bridge.web_port}")
    print("=" * 64)

    t0 = time.perf_counter()
    prev_t = t0
    launch_t = 0.0
    waiting_for_launch = True
    disarmed = False
    in_flare = False
    last_phase = None

    try:
        while True:
            now = time.perf_counter()
            t = now - t0
            step_dt = max(now - prev_t, 1e-3)
            prev_t = now

            est = hw.poll_state()
            est.pos = est.pos - origin
            r_loiter = get_loiter_radius()

            if consume_launch_request():
                clear_land_request()
                waiting_for_launch = False
                disarmed = False
                in_flare = False
                ctrl.reset()
                origin = hw.pos.copy()
                est.pos = np.zeros(3)
                t0 = now
                t = 0.0
                launch_t = 0.0

            carrot_target, loiter_center, phase = uav_target_for_state(t, est.pos)

            if waiting_for_launch:
                phase = "READY — PRESS START FLIGHT"
                thrust_cmd = 0.0
                hw.send_fixed_wing_attitude(0.0, 0.0, est.att[2], 0.0)
            else:
                low_batt = est.battery <= float(getattr(C, "LOW_BATTERY_THRESHOLD", 0.20))
                landing = low_batt or is_land_requested()
                mode_override = None

                if disarmed:
                    thrust_cmd = 0.0
                    phase = "LANDED (LOW BATT)" if low_batt else "LANDED — PRESS START FLIGHT"
                    hw.send_fixed_wing_attitude(0.0, 0.0, est.att[2], 0.0)
                else:
                    if landing:
                        tag = "LOW BATT" if low_batt else "MANUAL"
                        dist_home = math.hypot(float(est.pos[0]), float(est.pos[1]))
                        if not in_flare and dist_home > 10.0:
                            mode_override = "rtl_cruise"
                            loiter_center = np.array([0.0, 0.0, float(C.HOVER_ALT)], dtype=float)
                            carrot_target = loiter_center.copy()
                            phase = f"RTL TO HOME ({dist_home:.1f}m)"
                        elif not in_flare and float(est.pos[2]) > 0.75 and dist_home < 6.0:
                            r_loiter = 3.2
                            z_g = max(0.45, float(est.pos[2]) - 0.55)
                            dx, dy = float(est.pos[0]), float(est.pos[1])
                            gamma = math.atan2(dy, dx) if dist_home > 1e-3 else 0.0
                            lead = gamma + 0.65
                            loiter_center = np.array([0.0, 0.0, z_g], dtype=float)
                            carrot_target = np.array([
                                r_loiter * math.cos(lead),
                                r_loiter * math.sin(lead),
                                z_g,
                            ], dtype=float)
                            phase = f"DESCENDING AT HOME (z={est.pos[2]:.1f}m)"
                        else:
                            in_flare = True
                            mode_override = "flare_land"
                            z_g = float(np.clip((dist_home - 1.2) / 9.0 * float(C.HOVER_ALT), 0.0, float(C.HOVER_ALT)))
                            loiter_center = np.array([0.0, 0.0, 0.0], dtype=float)
                            carrot_target = np.array([0.0, 0.0, z_g], dtype=float)
                            phase = f"LANDING AT HOME ({tag})"

                        if float(est.pos[2]) <= 0.15 and float(np.linalg.norm(est.vel[:2])) < 0.8:
                            disarmed = True
                            thrust_cmd = 0.0
                            phase = f"LANDED AT HOME ({tag})"
                    elif (t - launch_t) < 2.6 and float(est.pos[2]) < (C.HOVER_ALT - 0.10):
                        mode_override = "climb"
                        yaw_c = float(est.att[2])
                        carrot_target = np.array([
                            float(est.pos[0]) + 8.0 * math.cos(yaw_c),
                            float(est.pos[1]) + 8.0 * math.sin(yaw_c),
                            float(C.HOVER_ALT),
                        ])
                        phase = "HAND-LAUNCH CLIMB"

                    if not disarmed:
                        thrust_cmd, _ = ctrl.update(
                            est,
                            carrot_target,
                            step_dt,
                            obstacles=[],
                            loiter_center=loiter_center,
                            loiter_radius=r_loiter,
                            flight_mode_override=mode_override,
                        )
                        if control_mode == "waypoint":
                            hw.send_fixed_wing_position_target(carrot_target, airspeed_des=C.CRUISE_SPEED)
                        else:
                            roll_des, pitch_des, yaw_des = ctrl.att_des
                            hw.send_fixed_wing_attitude(roll_des, pitch_des, yaw_des, thrust_cmd)

            phase_key = phase.split("(")[0].strip()
            if phase_key != last_phase:
                last_phase = phase_key
                print(
                    f"  [t={t:5.1f}s] {phase:28s} "
                    f"| pos=({est.pos[0]:+5.1f}, {est.pos[1]:+5.1f}, {est.pos[2]:5.1f}) m "
                    f"| Va={est.airspeed:4.1f} m/s"
                )

            hw_world = SimpleNamespace(
                t=t,
                quad=est,
                target=carrot_target,
                loiter_center=loiter_center,
                loiter_radius=r_loiter,
                ctrl=ctrl,
                thrust_cmd=thrust_cmd,
                wind=SimpleNamespace(value=np.zeros(3)),
                phase=phase,
                disarmed=disarmed,
            )
            bridge.send(hw_world, False, -1)

            sleep_s = dt - (time.perf_counter() - now)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--mavlink", default=None)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--control-mode", choices=["attitude", "waypoint"], default="attitude")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--host", "--laptop-ip", dest="host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--web-port", type=int, default=8080)
    args = ap.parse_args()

    if args.mavlink:
        run_hardware_mavlink(
            args.mavlink,
            args.baud,
            control_mode=args.control_mode,
            godot_host=args.host,
            godot_port=args.port,
            web_port=args.web_port,
        )
    else:
        run_sim_dry_run(
            godot_host=args.host,
            godot_port=args.port,
            web_port=args.web_port,
            speed=args.speed,
            loop=args.loop,
        )


if __name__ == "__main__":
    main()
