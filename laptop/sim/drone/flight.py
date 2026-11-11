import math
import numpy as np
import config as C

_CUSTOM_OBSTACLES = list(C.OBSTACLES)
_INTERACTIVE_POINT = None
_INTERACTIVE_RADIUS = None
_INTERACTIVE_MODE = "loiter"
_LAUNCH_REQUESTED = False
_LAND_REQUESTED = False


def request_launch():
    global _LAUNCH_REQUESTED, _LAND_REQUESTED
    _LAUNCH_REQUESTED = True
    _LAND_REQUESTED = False


def request_land():
    global _LAND_REQUESTED
    _LAND_REQUESTED = True


def clear_land_request():
    global _LAND_REQUESTED
    _LAND_REQUESTED = False


def consume_launch_request():
    global _LAUNCH_REQUESTED
    if _LAUNCH_REQUESTED:
        _LAUNCH_REQUESTED = False
        return True
    return False


def is_land_requested():
    return _LAND_REQUESTED


def set_selected_point(x, y, z=None, radius=None, mode="loiter"):
    global _INTERACTIVE_POINT, _INTERACTIVE_RADIUS, _INTERACTIVE_MODE, _LAND_REQUESTED
    alt = float(C.HOVER_ALT if z is None else z)
    _INTERACTIVE_POINT = np.array([float(x), float(y), max(alt, 0.8)], dtype=float)
    if radius is not None:
        _INTERACTIVE_RADIUS = max(float(radius), 1.8)
    _INTERACTIVE_MODE = str(mode or "loiter").lower()
    _LAND_REQUESTED = False


def clear_selected_point():
    global _INTERACTIVE_POINT, _INTERACTIVE_RADIUS, _LAND_REQUESTED
    _INTERACTIVE_POINT = None
    _INTERACTIVE_RADIUS = None
    _LAND_REQUESTED = False


def get_loiter_radius():
    if _INTERACTIVE_RADIUS is not None:
        return _INTERACTIVE_RADIUS
    return float(getattr(C, "CIRCLE_RADIUS", 4.5))


def get_selected_center(t=0.0):
    if _INTERACTIVE_POINT is not None:
        return _INTERACTIVE_POINT.copy(), -1

    mode = getattr(C, "FLIGHT_MODE", "loiter").lower()
    if mode in ("demo", "waypoints"):
        wps = getattr(C, "DEMO_WAYPOINTS", [(0.0, 0.0, C.HOVER_ALT, 90.0)])
        elapsed = 0.0
        for idx, wp in enumerate(wps):
            dur = float(wp[3]) if len(wp) > 3 else 90.0
            if t < elapsed + dur or idx == len(wps) - 1:
                return np.array([float(wp[0]), float(wp[1]), float(wp[2])], dtype=float), idx
            elapsed += dur

    hp = getattr(C, "HOVER_POINT", (0.0, 0.0, C.HOVER_ALT))
    return np.array([float(hp[0]), float(hp[1]), float(hp[2])], dtype=float), 0


def get_obstacles(rng=None):
    preset = getattr(C, "OBSTACLE_PRESET", "custom")
    presets = getattr(C, "OBSTACLE_PRESETS", {})
    obs = list(presets[preset]) if (preset in presets and preset != "custom") else list(_CUSTOM_OBSTACLES)
    n_rand = int(getattr(C, "RANDOM_OBSTACLES", 0))
    if n_rand > 0:
        rng = rng if rng is not None else np.random.default_rng()
        for _ in range(n_rand):
            ang = rng.uniform(0.3, 2.0 * math.pi - 0.3)
            r_p = get_loiter_radius() + rng.uniform(-0.35, 0.35)
            obs.append((
                round(r_p * math.cos(ang), 2),
                round(r_p * math.sin(ang), 2),
                round(C.HOVER_ALT + rng.uniform(-0.25, 0.25), 2),
                round(rng.uniform(0.25, 0.45), 2),
            ))
    return obs


def mission_duration():
    mode = getattr(C, "FLIGHT_MODE", "loiter").lower()
    if mode in ("demo", "waypoints"):
        wps = getattr(C, "DEMO_WAYPOINTS", [])
        if wps:
            return float(sum(wp[3] for wp in wps if len(wp) > 3))
    return float(C.DURATION)


def uav_target_for_state(t, uav_pos=None):
    mode = getattr(C, "FLIGHT_MODE", "loiter").lower()
    r = get_loiter_radius()

    if mode == "figure8" and _INTERACTIVE_POINT is None:
        w = (2.0 * math.pi) / float(C.CIRCLE_PERIOD)
        center = np.array([0.0, 0.0, float(C.HOVER_ALT)])
        pt = np.array([r * math.sin(w * t), r * math.sin(w * t) * math.cos(w * t), float(C.HOVER_ALT)])
        return pt, center, "FIGURE-8"

    center, wp_idx = get_selected_center(t)
    tag = "POINT" if wp_idx < 0 else "HOME"

    if uav_pos is None:
        w = float(getattr(C, "CRUISE_SPEED", 4.5)) / max(r, 1.0)
        ang = -0.5 * math.pi + w * t
        carrot = np.array([center[0] + r * math.cos(ang), center[1] + r * math.sin(ang), center[2]])
        return carrot, center, f"LOITER ({tag})"

    dx = float(uav_pos[0] - center[0])
    dy = float(uav_pos[1] - center[1])
    dist = math.hypot(dx, dy)

    if _INTERACTIVE_POINT is not None and _INTERACTIVE_MODE == "fly_to" and dist > 1.2:
        return center.copy(), center, f"FLYING TO {tag} ({dist:.1f}m)"

    gamma = math.atan2(dy, dx) if dist > 1e-3 else -0.5 * math.pi
    lead = gamma + 0.60
    carrot = np.array([center[0] + r * math.cos(lead), center[1] + r * math.sin(lead), center[2]])

    phase = f"FLYING TO {tag} ({dist:.1f}m)" if abs(dist - r) > 2.2 else f"ORBITING {tag} (r={r:.1f}m)"
    return carrot, center, phase


def flight_phase(t, uav_pos=None):
    return uav_target_for_state(t, uav_pos)[2]


def hover_target(t):
    return get_selected_center(t)[0]


def circle_target(t, uav_pos=None):
    return uav_target_for_state(t, uav_pos)[0]


def check_collision(pos, obstacles=None):
    obs = C.OBSTACLES if obstacles is None else obstacles
    for i, (ox, oy, oz, r) in enumerate(obs):
        if np.linalg.norm(pos - np.array([ox, oy, oz])) < (C.DRONE_RADIUS + r):
            return True, i
    return False, -1
