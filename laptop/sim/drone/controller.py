import math
import numpy as np
import config as C
from drone.pid import PID
from drone.avoidance import deflect_target, repulsion
from drone.flight import get_selected_center, get_loiter_radius


def _wrap_pi(a):
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


class FlightController:
    def __init__(self):
        self.alt_pid = PID(C.ALT_KP, C.ALT_KI, C.ALT_KD, limit=C.ALT_LIMIT)
        self.speed_pid = PID(
            float(getattr(C, "SPEED_KP", 0.65)),
            float(getattr(C, "SPEED_KI", 0.25)),
            float(getattr(C, "SPEED_KD", 0.05)),
            limit=float(getattr(C, "MAX_THRUST", 5.0)),
        )
        self.x_pid = PID(C.POS_KP, C.POS_KI, C.POS_KD, limit=C.POS_LIMIT)
        self.y_pid = PID(C.POS_KP, C.POS_KI, C.POS_KD, limit=C.POS_LIMIT)

        self.kp_att = float(C.ATT_KP)
        self.kd_att = float(C.ATT_KD)
        self.max_bank = math.radians(float(getattr(C, "MAX_BANK_DEG", 40.0)))
        self.max_pitch = math.radians(float(getattr(C, "MAX_PITCH_DEG", 22.0)))
        self.max_tilt = self.max_bank

        self.cmd_target = None
        self.loiter_center = np.array([0.0, 0.0, float(C.HOVER_ALT)])
        self.loiter_radius = float(C.CIRCLE_RADIUS)
        self.att_des = np.zeros(3)
        self._rate_filt = np.zeros(3)
        self._vel_filt = None
        self._roll_cmd_smooth = None
        self._pitch_cmd_smooth = None

    def reset(self):
        self.alt_pid.reset()
        self.speed_pid.reset()
        self.x_pid.reset()
        self.y_pid.reset()
        self._rate_filt = np.zeros(3)
        self._vel_filt = None
        self._roll_cmd_smooth = None
        self._pitch_cmd_smooth = None

    def update(
        self,
        uav,
        target,
        dt,
        yaw_des=None,
        loiter_center=None,
        loiter_radius=None,
        obstacles=None,
        flight_mode_override=None,
    ):
        if dt <= 0.0:
            dt = C.DT
        raw = np.asarray(target, dtype=float)
        cmd = deflect_target(raw) if C.AVOID_ENABLED else raw.copy()
        self.cmd_target = cmd

        if loiter_center is None:
            loiter_center, _ = get_selected_center(0.0)
        self.loiter_center = np.asarray(loiter_center, dtype=float)
        R_orb = float(loiter_radius) if loiter_radius is not None else get_loiter_radius()
        self.loiter_radius = R_orb

        pos = uav.pos
        if self._vel_filt is None:
            self._vel_filt = np.array(uav.vel, dtype=float)
        else:
            a_v = dt / (dt + 1.0 / (2.0 * math.pi * 8.0))
            self._vel_filt += a_v * (np.asarray(uav.vel, dtype=float) - self._vel_filt)
        vel = self._vel_filt

        roll, pitch, yaw = float(uav.att[0]), float(uav.att[1]), float(uav.att[2])
        vg = math.hypot(float(vel[0]), float(vel[1]))
        course = math.atan2(float(vel[1]), float(vel[0])) if vg > 0.8 else yaw
        v_cr = float(getattr(C, "CRUISE_SPEED", 4.5))
        va = float(getattr(uav, "airspeed", max(np.linalg.norm(vel), v_cr)))

        cx, cy = float(self.loiter_center[0]), float(self.loiter_center[1])
        dx, dy = float(pos[0]) - cx, float(pos[1]) - cy
        d_c = math.hypot(dx, dy)
        gamma = math.atan2(dy, dx) if d_c > 1e-3 else -0.5 * math.pi

        cos_t = float(np.clip(R_orb / max(d_c, 0.2), 0.25, 1.0))
        lead = gamma + max(math.acos(cos_t), 0.65)
        carrot = np.array([cx + R_orb * math.cos(lead), cy + R_orb * math.sin(lead)])

        if (
            np.linalg.norm(cmd - raw) > 0.03
            or getattr(C, "FLIGHT_MODE", "loiter").lower() == "figure8"
            or flight_mode_override in ("rtl_cruise", "rtl_descend", "flare_land")
        ):
            carrot = np.array([float(cmd[0]), float(cmd[1])])

        to_c = carrot - pos[:2]
        dist_c = float(np.linalg.norm(to_c))
        l1_min = 2.0 * R_orb * math.sin(0.65 * 0.5)
        L1 = float(np.clip(dist_c, l1_min, 6.0))
        c_des = math.atan2(float(to_c[1]), float(to_c[0]))
        eta = _wrap_pi(c_des - course)

        a_rep = (repulsion(pos, vel) + 0.85 * repulsion(pos + vel * 0.35, vel)) if C.AVOID_ENABLED else np.zeros(3)
        a_lat_rep = -a_rep[0] * math.sin(course) + a_rep[1] * math.cos(course)

        v_turn = max(vg, v_cr)
        if abs(eta) > 0.78 * math.pi:
            s_eta = 1.0
        elif abs(eta) > 0.5 * math.pi:
            s_eta = math.copysign(1.0, eta)
        else:
            s_eta = math.sin(eta)

        d_err = d_c - R_orb
        d_dot = vg * math.cos(course - gamma)
        if flight_mode_override in ("rtl_cruise", "rtl_descend", "flare_land"):
            a_rad = 0.0
        else:
            a_rad = -1.6 * d_err - 1.8 * d_dot if abs(d_err) < 2.5 else 0.0

        a_left = (2.0 * v_turn * v_turn / L1) * s_eta - a_rad + a_lat_rep
        bank_lim = self.max_bank * float(np.clip((va - 2.6) / 1.5, 0.35, 1.0))
        r_raw = float(np.clip(-math.atan2(a_left, C.GRAVITY), -bank_lim, bank_lim))
        if flight_mode_override == "climb":
            r_raw = float(np.clip(r_raw, -math.radians(10.0), math.radians(10.0)))
        elif flight_mode_override == "flare_land":
            r_raw = 0.0 if float(pos[2]) < 0.35 else float(np.clip(r_raw, -math.radians(28.0), math.radians(28.0)))

        if self._roll_cmd_smooth is None:
            self._roll_cmd_smooth = roll
        r_step = math.radians(55.0) * dt
        self._roll_cmd_smooth += float(np.clip(r_raw - self._roll_cmd_smooth, -r_step, r_step))
        roll_des = self._roll_cmd_smooth

        z_des = float(cmd[2]) + 0.20 * float(a_rep[2])
        alt_err = z_des - float(pos[2])
        climb_err = -float(vel[2])

        rho = float(getattr(C, "AIR_DENSITY", 1.225))
        S = float(getattr(C, "WING_AREA", 0.55))
        cl0 = float(getattr(C, "CL0", 0.40))
        cla = float(getattr(C, "CL_ALPHA", 5.5))
        a_st = float(getattr(C, "STALL_ALPHA", 0.40))
        q_dyn = 0.5 * rho * max(va, 2.5) ** 2 * S
        cl_req = (uav.mass * C.GRAVITY) / max(q_dyn * max(math.cos(roll), 0.50), 0.5)
        a_req = float(np.clip((cl_req - cl0) / cla, -0.20, 0.72 * a_st))
        fpa = math.atan2(float(vel[2]), max(vg, 1.5))
        trim_up = fpa + a_req / max(math.cos(roll), 0.65)

        up_cmd = trim_up + self.alt_pid.update(alt_err, dt, climb_err)
        if flight_mode_override == "flare_land":
            up_cmd = math.radians(3.0) if float(pos[2]) < 0.20 else float(np.clip(0.55 * alt_err - 0.18, math.radians(-14.0), math.radians(4.0)))
        p_raw = float(np.clip(-up_cmd, -self.max_pitch, self.max_pitch))

        if self._pitch_cmd_smooth is None:
            self._pitch_cmd_smooth = pitch
        p_step = math.radians(35.0) * dt
        self._pitch_cmd_smooth += float(np.clip(p_raw - self._pitch_cmd_smooth, -p_step, p_step))
        pitch_des = self._pitch_cmd_smooth

        cd0 = float(getattr(C, "CD0", 0.04))
        cdi = float(getattr(C, "CD_INDUCED", 0.05))
        drag = q_dyn * (cd0 + cdi * (cl_req * cl_req))
        v_tgt = 3.2 if flight_mode_override == "rtl_descend" else v_cr
        thrust = drag + uav.mass * C.GRAVITY * math.sin(fpa) + self.speed_pid.update(v_tgt - va, dt)
        max_t = float(getattr(C, "MAX_THRUST", 5.0))
        if flight_mode_override == "flare_land":
            thrust = 0.0
        thrust = float(np.clip(thrust, 0.0, max_t))

        a_gyro = dt / (dt + 1.0 / (2.0 * math.pi * 40.0))
        self._rate_filt += a_gyro * (uav.rate - self._rate_filt)

        self.att_des = np.array([roll_des, pitch_des, c_des if yaw_des is None else float(yaw_des)])
        r_err = _wrap_pi(roll_des - roll)
        p_err = _wrap_pi(pitch_des - pitch)
        s_err = _wrap_pi(course - yaw)

        phi_d = 9.0 * r_err
        th_d = 8.0 * p_err
        psi_d = -(C.GRAVITY / max(va, 3.0)) * math.tan(roll) + 3.5 * s_err

        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        p_des = phi_d - sp * psi_d
        q_des = cr * th_d + cp * sr * psi_d
        r_des = -sr * th_d + cp * cr * psi_d

        kp_r = 18.0
        g_comp = np.cross(self._rate_filt, uav.inertia * self._rate_filt)
        tx = uav.inertia[0] * kp_r * (p_des - self._rate_filt[0]) + g_comp[0]
        ty = uav.inertia[1] * kp_r * (q_des - self._rate_filt[1]) + g_comp[1]
        tz = uav.inertia[2] * kp_r * (r_des - self._rate_filt[2]) + g_comp[2]

        return float(thrust), np.array([tx, ty, tz], dtype=float)
