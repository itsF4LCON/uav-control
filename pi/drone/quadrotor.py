import numpy as np
import config as C


def _skew(w):
    return np.array([
        [0.0, -w[2], w[1]],
        [w[2], 0.0, -w[0]],
        [-w[1], w[0], 0.0],
    ])


def _rodrigues(w, dt):
    th = np.linalg.norm(w) * dt
    if th < 1e-12:
        return np.eye(3)
    K = _skew(w / np.linalg.norm(w))
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


class FixedWingUAV:
    def __init__(self, rng=None):
        self.rng = rng if rng is not None else np.random.default_rng(C.SEED)
        self.mass = float(C.MASS)
        self.inertia = np.array(C.INERTIA, dtype=float)

        self._m = self.mass
        self._I = self.inertia.copy()
        self._thrust_gain = 1.0
        if getattr(C, "RANDOMIZE_PARAMS", False):
            f = float(getattr(C, "RANDOMIZE_FRAC", 0.08))
            self._m *= self.rng.uniform(1.0 - 0.5 * f, 1.0 + 0.5 * f)
            self._I *= self.rng.uniform(1.0 - 0.5 * f, 1.0 + 0.5 * f, 3)
            self._thrust_gain = float(self.rng.uniform(1.0 - 0.5 * f, 1.0 + 0.5 * f))

        v_cr = float(getattr(C, "CRUISE_SPEED", 4.5))
        rho = float(getattr(C, "AIR_DENSITY", 1.225))
        S = float(getattr(C, "WING_AREA", 0.55))
        cl0 = float(getattr(C, "CL0", 0.40))
        cla = float(getattr(C, "CL_ALPHA", 5.5))
        q_cr = 0.5 * rho * v_cr * v_cr * S
        cl_trim = (self.mass * C.GRAVITY) / max(q_cr, 1e-3)
        self.alpha_trim = (cl_trim - cl0) / cla
        self.reset()

    def reset(self):
        v_cr = float(getattr(C, "CRUISE_SPEED", 4.5))
        r0 = float(getattr(C, "CIRCLE_RADIUS", 4.5))
        alt0 = float(getattr(C, "HOVER_ALT", 2.2))
        roll0 = -np.arctan2(v_cr * v_cr, C.GRAVITY * max(r0, 1.5))
        pitch0 = -float(self.alpha_trim) / max(np.cos(roll0), 0.6)
        cr, sr = np.cos(roll0), np.sin(roll0)
        cp, sp = np.cos(pitch0), np.sin(pitch0)
        self.pos = np.array([0.0, -r0, alt0], dtype=float)
        self.vel = np.array([v_cr, 0.0, 0.0], dtype=float)
        self.R = np.array([
            [cp, sp * sr, sp * cr],
            [0.0, cr, -sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=float)
        self.att = self.euler_from_R(self.R)
        self.rate = self.R.T @ np.array([0.0, 0.0, v_cr / max(r0, 1.5)], dtype=float)
        self.airspeed = v_cr
        self.alpha = float(self.alpha_trim)
        self.beta = 0.0
        self.air_vel = self.vel.copy()
        self.surfaces = np.array([0.45, 0.0, 0.0, 0.0], dtype=float)
        self.surfaces_cmd = self.surfaces.copy()
        self.thrust = 0.45 * float(getattr(C, "MAX_THRUST", 5.0))
        self.motor_thrust = np.array([self.thrust, 0.0, 0.0, 0.0], dtype=float)
        self.saturated = False
        self.battery = 1.0

    def reset_hand_launch(self):
        v_throw = max(4.0, float(getattr(C, "CRUISE_SPEED", 4.5)) * 0.92)
        pitch0 = -0.14
        cp, sp = np.cos(pitch0), np.sin(pitch0)
        self.pos = np.array([0.0, 0.0, 1.2], dtype=float)
        self.vel = np.array([v_throw, 0.0, 0.35], dtype=float)
        self.R = np.array([
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ], dtype=float)
        self.att = self.euler_from_R(self.R)
        self.rate = np.zeros(3, dtype=float)
        self.airspeed = v_throw
        self.alpha = float(self.alpha_trim)
        self.beta = 0.0
        self.air_vel = self.vel.copy()
        self.surfaces = np.array([0.85, 0.0, 0.0, 0.0], dtype=float)
        self.surfaces_cmd = self.surfaces.copy()
        self.thrust = 0.85 * float(getattr(C, "MAX_THRUST", 5.0))
        self.motor_thrust = np.array([self.thrust, 0.0, 0.0, 0.0], dtype=float)
        self.saturated = False
        self.battery = 1.0

    def rotation_matrix(self):
        return self.R

    @staticmethod
    def euler_from_R(R):
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        return np.array([roll, pitch, yaw])

    def _lift_coeff(self, alpha):
        cl0 = float(getattr(C, "CL0", 0.40))
        cla = float(getattr(C, "CL_ALPHA", 5.5))
        a_st = float(getattr(C, "STALL_ALPHA", 0.40))
        cl_lin = cl0 + cla * alpha
        if abs(alpha) <= a_st:
            return cl_lin
        ex = abs(alpha) - a_st
        s = np.exp(-3.5 * ex)
        return float(cl_lin * s + 0.8 * np.sin(2.0 * alpha) * (1.0 - s))

    def step(self, thrust_cmd, torque_cmd, dt, wind=None):
        if dt <= 0.0:
            return
        max_t = float(getattr(C, "MAX_THRUST", 5.0))
        tau_m = float(getattr(C, "MOTOR_TAU", 0.04))
        tau_s = float(getattr(C, "SERVO_TAU", 0.02))

        sag = float(getattr(C, "BATTERY_SAG", 0.08))
        self.battery = max(0.0, self.battery - dt / max(float(C.DURATION), 30.0))
        v_scale = 1.0 - sag * (1.0 - self.battery)

        t_req = float(np.clip(thrust_cmd, 0.0, max_t))
        self.saturated = bool(thrust_cmd > max_t or thrust_cmd < 0.0)
        self.thrust += (t_req - self.thrust) * (1.0 - np.exp(-dt / max(tau_m, 1e-4)))
        T_fwd = self.thrust * self._thrust_gain * v_scale

        wind = np.zeros(3) if wind is None else np.asarray(wind, dtype=float)
        self.air_vel = self.vel - wind
        vb = self.R.T @ self.air_vel
        u, v, w = float(vb[0]), float(vb[1]), float(vb[2])
        va = float(max(np.linalg.norm(vb), 0.5))
        self.airspeed = va

        self.alpha = float(np.arctan2(-w, max(u, 0.5)))
        self.beta = float(np.arcsin(np.clip(v / va, -1.0, 1.0)))

        rho = float(getattr(C, "AIR_DENSITY", 1.225))
        S = float(getattr(C, "WING_AREA", 0.55))
        q_dyn = 0.5 * rho * va * va * S

        cl = self._lift_coeff(self.alpha)
        cd = float(getattr(C, "CD0", 0.04)) + float(getattr(C, "CD_INDUCED", 0.05)) * (cl * cl)

        lift = q_dyn * cl
        drag = q_dyn * cd
        side = -q_dyn * 0.65 * self.beta

        ca, sa = np.cos(self.alpha), np.sin(self.alpha)
        fx = -drag * ca + lift * sa
        fz = drag * sa + lift * ca
        f_world = self.R @ np.array([T_fwd + fx, side, fz]) + np.array([0.0, 0.0, -C.GRAVITY * self._m])

        max_tau = np.array([0.8, 1.0, 0.5])
        s_req = np.clip(np.asarray(torque_cmd, dtype=float) / max_tau, -1.0, 1.0)
        self.surfaces[0] = float(np.clip(self.thrust / max(max_t, 1e-3), 0.0, 1.0))
        self.surfaces[1:] += (s_req - self.surfaces[1:]) * (1.0 - np.exp(-dt / max(tau_s, 1e-4)))
        self.surfaces_cmd = np.array([t_req / max(max_t, 1e-3), *s_req])

        auth = float(np.clip((va / float(getattr(C, "CRUISE_SPEED", 4.5))) ** 2, 0.35, 1.8))
        tau_ctrl = self.surfaces[1:] * max_tau * auth
        tau_aero = np.array([
            -0.06 * self.rate[0],
            0.08 * (self.alpha - self.alpha_trim) - 0.06 * self.rate[1],
            0.08 * self.beta - 0.06 * self.rate[2],
        ])
        tau = tau_ctrl + tau_aero

        self.motor_thrust = np.array([
            self.thrust,
            abs(self.surfaces[1]) * max_t,
            abs(self.surfaces[2]) * max_t,
            abs(self.surfaces[3]) * max_t,
        ])

        self.vel += (f_world / self._m) * dt
        self.pos += self.vel * dt

        ang_acc = (tau - np.cross(self.rate, self._I * self.rate)) / self._I
        self.rate = self.rate + ang_acc * dt
        self.R = self.R @ _rodrigues(self.rate, dt)
        u_s, _, vt_s = np.linalg.svd(self.R)
        self.R = u_s @ vt_s
        self.att = self.euler_from_R(self.R)

        if self.pos[2] <= 0.0:
            self.pos[2] = 0.0
            self.vel[2] = max(self.vel[2], 0.0)
            if thrust_cmd <= 0.05:
                self.vel[:2] *= np.exp(-5.5 * dt)
                self.rate *= np.exp(-12.0 * dt)
                self.airspeed = float(np.linalg.norm(self.vel))


Quadrotor = FixedWingUAV
