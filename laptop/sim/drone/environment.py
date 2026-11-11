from collections import deque
from types import SimpleNamespace
import numpy as np
import config as C


class Wind:
    def __init__(self, rng):
        self.rng = rng
        self.mean = np.array(C.WIND_MEAN, dtype=float)
        self.gust = np.zeros(3)
        self.std = C.WIND_GUST_STD * np.array([1.0, 1.0, C.WIND_VERTICAL_RATIO])
        self.value = self.mean.copy() if C.WIND_ENABLED else np.zeros(3)

    def step(self, dt):
        if not C.WIND_ENABLED or dt <= 0:
            self.value = np.zeros(3)
            return self.value
        tau = C.WIND_GUST_TAU
        self.gust += (-self.gust / tau) * dt + self.std * np.sqrt(2.0 * dt / tau) * self.rng.standard_normal(3)
        self.value = self.mean + self.gust
        return self.value


class Sensors:
    def __init__(self, rng):
        self.rng = rng
        self.gyro_bias = rng.normal(0.0, C.GYRO_BIAS_STD, 3) if C.SENSORS_ENABLED else np.zeros(3)
        steps = max(int(round(getattr(C, "POS_LATENCY_S", 0.0) / C.DT)), 1)
        self._pos_buf = deque(maxlen=steps)
        self._vel_buf = deque(maxlen=steps)
        self._held_pos = None
        self._held_vel = None
        self._since = 1e9

    def measure(self, quad, dt=C.DT):
        n = self.rng.normal
        if not C.SENSORS_ENABLED:
            return SimpleNamespace(
                pos=quad.pos.copy(),
                vel=quad.vel.copy(),
                att=quad.att.copy(),
                rate=quad.rate.copy(),
                airspeed=float(getattr(quad, "airspeed", np.linalg.norm(quad.vel))),
                alpha_trim=float(getattr(quad, "alpha_trim", 0.06)),
                mass=quad.mass,
                inertia=quad.inertia,
            )

        self._pos_buf.append(quad.pos.copy())
        self._vel_buf.append(quad.vel.copy())
        d_pos = self._pos_buf[0]
        d_vel = self._vel_buf[0]

        hz = getattr(C, "POS_UPDATE_HZ", 0.0)
        self._since += dt
        if self._held_pos is None or hz <= 0 or self._since >= (1.0 / hz - 1e-9):
            self._held_pos = d_pos + n(0, C.POS_NOISE_STD, 3)
            self._held_vel = d_vel + n(0, C.VEL_NOISE_STD, 3)
            self._since = 0.0

        return SimpleNamespace(
            pos=self._held_pos.copy(),
            vel=self._held_vel.copy(),
            att=quad.att + n(0, C.ATT_NOISE_STD, 3),
            rate=quad.rate + self.gyro_bias + n(0, C.GYRO_NOISE_STD, 3),
            airspeed=max(0.5, float(getattr(quad, "airspeed", np.linalg.norm(quad.vel))) + float(n(0, 0.05))),
            alpha_trim=float(getattr(quad, "alpha_trim", 0.06)),
            mass=quad.mass,
            inertia=quad.inertia,
        )
