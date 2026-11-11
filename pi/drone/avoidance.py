import numpy as np
import config as C

_UP = np.array([0.0, 0.0, 1.0])


def _obstacles():
    for ox, oy, oz, r in C.OBSTACLES:
        yield np.array([ox, oy, oz], dtype=float), float(r)


def deflect_target(target):
    cmd = np.array(target, dtype=float)
    for _ in range(3):
        moved = False
        for center, r in _obstacles():
            R = r + C.DRONE_RADIUS + C.AVOID_MARGIN
            d = cmd - center
            dist = np.linalg.norm(d)
            if dist >= R:
                continue
            under = d[2] < 0.0 and (center[2] - R) > C.DRONE_RADIUS + 0.1
            bias = -_UP if under else _UP
            v = d.copy()
            v[2] = (min(d[2], 0.0) - (R - dist)) if under else (max(d[2], 0.0) + (R - dist))
            n = np.linalg.norm(v)
            cmd = center + (v / n if n > 1e-9 else bias) * R
            moved = True
        if not moved:
            break
    if not np.allclose(cmd, target):
        cmd[2] = max(cmd[2], C.DRONE_RADIUS)
    else:
        cmd[2] = max(cmd[2], 0.0)
    return cmd


def repulsion(pos, vel):
    acc = np.zeros(3)
    for center, r in _obstacles():
        d = pos - center
        dist = np.linalg.norm(d)
        if dist < 1e-6:
            continue
        n = d / dist
        gap = dist - (r + C.DRONE_RADIUS)
        if gap >= C.REPULSE_RANGE:
            continue
        g = max(gap, 0.02)
        push = C.REPULSE_GAIN * (1.0 / g - 1.0 / C.REPULSE_RANGE)
        v_in = -np.dot(vel, n)
        damp = C.REPULSE_DAMP * max(v_in, 0.0) * (1.0 - gap / C.REPULSE_RANGE)
        acc += min(push + damp, C.REPULSE_MAX) * n
    return acc
