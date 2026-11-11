class PID:
    def __init__(self, kp, ki, kd, limit=float("inf")):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self._integral = 0.0
        self._prev_err = None

    def reset(self):
        self._integral = 0.0
        self._prev_err = None

    def update(self, err, dt, deriv=None):
        if dt <= 0.0:
            return 0.0
        self._integral += err * dt
        cap = self.limit / self.ki if self.ki > 0 else self.limit
        self._integral = max(-cap, min(cap, self._integral))
        if deriv is None:
            deriv = 0.0 if self._prev_err is None else (err - self._prev_err) / dt
        self._prev_err = err
        out = self.kp * err + self.ki * self._integral + self.kd * deriv
        return max(-self.limit, min(self.limit, out))
