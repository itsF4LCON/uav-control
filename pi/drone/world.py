import math
import numpy as np
import config as C
from drone.quadrotor import Quadrotor
from drone.controller import FlightController
from drone.environment import Wind, Sensors
from drone.flight import (
    uav_target_for_state,
    check_collision,
    get_obstacles,
    get_loiter_radius,
    mission_duration,
    consume_launch_request,
    is_land_requested,
    is_kill_requested,
    clear_land_request,
)


class World:
    def __init__(self, seed=None, resolve_obstacles=False, auto_launch=False):
        self.rng = np.random.default_rng(C.SEED if seed is None else seed)
        if resolve_obstacles:
            C.OBSTACLES = get_obstacles(self.rng)
        self.quad = Quadrotor(self.rng)
        self.uav = self.quad
        self.ctrl = FlightController()
        self.wind = Wind(self.rng)
        self.sensors = Sensors(self.rng)
        self.t = 0.0
        self._launch_t = 0.0
        self.duration = mission_duration()
        self.loiter_radius = get_loiter_radius()
        self.disarmed = False
        self._in_flare = False
        self.thrust_cmd = 0.0

        if auto_launch:
            self.waiting_for_launch = False
            self.launch()
        else:
            self.waiting_for_launch = True
            self._set_holding_in_hand()

    def _set_holding_in_hand(self):
        self.quad.pos = np.array([0.0, 0.0, 1.2], dtype=float)
        self.quad.vel = np.zeros(3, dtype=float)
        self.quad.R = np.eye(3, dtype=float)
        self.quad.att = np.zeros(3, dtype=float)
        self.quad.rate = np.zeros(3, dtype=float)
        self.quad.airspeed = 0.0
        self.quad.alpha = 0.0
        self.quad.beta = 0.0
        self.quad.surfaces = np.zeros(4, dtype=float)
        self.quad.thrust = 0.0
        self.quad.battery = 1.0
        self.thrust_cmd = 0.0
        self.target, self.loiter_center, _ = uav_target_for_state(0.0, self.quad.pos)
        self.phase = "READY — PRESS START FLIGHT"

    def launch(self):
        clear_land_request()
        self.waiting_for_launch = False
        self.quad.reset_hand_launch()
        self.ctrl.reset()
        self.t = 0.0
        self._launch_t = 0.0
        self.disarmed = False
        self._in_flare = False
        self.phase = "HAND-LAUNCH CLIMB"

    def step(self, dt=C.DT):
        if consume_launch_request():
            self.launch()

        if is_kill_requested():
            self.disarmed = True
            self.thrust_cmd = 0.0
            self.quad.vel[:] = 0.0
            self.quad.rate[:] = 0.0
            self.quad.airspeed = 0.0
            self.quad.surfaces[:] = 0.0
            self.phase = "DISARMED (KILL SWITCH)"
            return False, -1

        self.loiter_radius = get_loiter_radius()

        if self.waiting_for_launch:
            self.target, self.loiter_center, _ = uav_target_for_state(self.t, self.quad.pos)
            self.phase = "READY — PRESS START FLIGHT"
            return False, -1

        wind = self.wind.step(dt)
        est = self.sensors.measure(self.quad, dt)
        self.target, self.loiter_center, self.phase = uav_target_for_state(self.t, est.pos)

        low_batt = self.quad.battery <= float(getattr(C, "LOW_BATTERY_THRESHOLD", 0.20))
        landing = low_batt or is_land_requested()

        if self.disarmed:
            self.thrust_cmd = 0.0
            self.quad.vel[:] = 0.0
            self.quad.rate[:] = 0.0
            self.quad.airspeed = 0.0
            self.quad.surfaces[:] = 0.0
            self.phase = "LANDED (LOW BATT)" if low_batt else "LANDED — PRESS START FLIGHT"
            self.t += dt
            return False, -1

        mode_override = None

        if landing:
            tag = "LOW BATT" if low_batt else "MANUAL"
            dist_home = math.hypot(float(est.pos[0]), float(est.pos[1]))
            if not self._in_flare and dist_home > 10.0:
                mode_override = "rtl_cruise"
                self.loiter_center = np.array([0.0, 0.0, float(C.HOVER_ALT)], dtype=float)
                self.target = self.loiter_center.copy()
                self.phase = f"RTL TO HOME ({dist_home:.1f}m)"
            elif not self._in_flare and float(est.pos[2]) > 0.75 and dist_home < 6.0:
                self.loiter_radius = 3.2
                z_g = max(0.45, float(est.pos[2]) - 0.55)
                dx, dy = float(est.pos[0]), float(est.pos[1])
                gamma = math.atan2(dy, dx) if dist_home > 1e-3 else 0.0
                lead = gamma + 0.65
                self.loiter_center = np.array([0.0, 0.0, z_g], dtype=float)
                self.target = np.array([
                    self.loiter_radius * math.cos(lead),
                    self.loiter_radius * math.sin(lead),
                    z_g,
                ], dtype=float)
                self.phase = f"DESCENDING AT HOME (z={est.pos[2]:.1f}m)"
            else:
                self._in_flare = True
                mode_override = "flare_land"
                z_g = float(np.clip((dist_home - 1.2) / 9.0 * float(C.HOVER_ALT), 0.0, float(C.HOVER_ALT)))
                self.loiter_center = np.array([0.0, 0.0, 0.0], dtype=float)
                self.target = np.array([0.0, 0.0, z_g], dtype=float)
                self.quad.vel[:2] *= math.exp(-0.55 * dt)
                self.phase = f"LANDING AT HOME ({tag})"

            if float(self.quad.pos[2]) <= 0.02 and float(np.linalg.norm(self.quad.vel[:2])) < 0.45:
                self.disarmed = True
                self.thrust_cmd = 0.0
                self.quad.vel[:] = 0.0
                self.quad.airspeed = 0.0
                self.phase = f"LANDED AT HOME ({tag})"
                self.t += dt
                return False, -1

        elif (self.t - self._launch_t) < 2.6 and float(self.quad.pos[2]) < (C.HOVER_ALT - 0.10):
            mode_override = "climb"
            self.target = np.array([float(est.pos[0]) + 8.0, float(est.pos[1]), float(C.HOVER_ALT)])
            self.phase = "HAND-LAUNCH CLIMB"

        self.thrust_cmd, torque = self.ctrl.update(
            est,
            self.target,
            dt,
            loiter_center=self.loiter_center,
            loiter_radius=self.loiter_radius,
            flight_mode_override=mode_override,
        )
        self.quad.step(self.thrust_cmd, torque, dt, wind=wind)
        self.t += dt
        return check_collision(self.quad.pos)
