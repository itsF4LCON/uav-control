import math
import numpy as np
import pybullet as pb
import config as C


def _rpy_to_quat(roll, pitch, yaw):
    return pb.getQuaternionFromEuler([roll, pitch, yaw])


def _rgba(hex_str, a=1.0):
    h = hex_str.lstrip("#")
    r, g, b = (int(h[i : i + 2], 16) / 255.0 for i in (0, 2, 4))
    return [r, g, b, a]


class Visualizer:
    def __init__(self):
        self._client = pb.connect(pb.GUI)

        pb.resetDebugVisualizerCamera(
            cameraDistance=C.CAMERA_DIST,
            cameraYaw=C.CAMERA_AZIM,
            cameraPitch=-C.CAMERA_ELEV,
            cameraTargetPosition=[0, 0, C.HOVER_ALT],
            physicsClientId=self._client,
        )

        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0, physicsClientId=self._client)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 1, physicsClientId=self._client)
        pb.configureDebugVisualizer(pb.COV_ENABLE_RGB_BUFFER_PREVIEW, 0, physicsClientId=self._client)
        pb.configureDebugVisualizer(pb.COV_ENABLE_WIREFRAME, 0, physicsClientId=self._client)
        pb.setGravity(0, 0, 0, physicsClientId=self._client)

        self._ground_id = self._make_ground()
        self._drone_id = self._make_drone()
        self._target_id = self._make_target()
        self._obstacle_ids = self._make_obstacles()
        self._draw_grid()

        self._trail_ids = []
        self._prev_pos = None
        self._hud_id = None

    def _make_ground(self):
        col = pb.createCollisionShape(
            pb.GEOM_BOX,
            halfExtents=[C.GRID_SIZE, C.GRID_SIZE, 0.01],
            physicsClientId=self._client,
        )
        vis = pb.createVisualShape(
            pb.GEOM_BOX,
            halfExtents=[C.GRID_SIZE, C.GRID_SIZE, 0.01],
            rgbaColor=_rgba("#0d1117"),
            physicsClientId=self._client,
        )
        return pb.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[0, 0, -0.01],
            physicsClientId=self._client,
        )

    def _make_drone(self):
        base_col = pb.createCollisionShape(
            pb.GEOM_BOX,
            halfExtents=[0.17, 0.020, 0.018],
            physicsClientId=self._client,
        )
        base_vis = pb.createVisualShape(
            pb.GEOM_BOX,
            halfExtents=[0.17, 0.020, 0.018],
            rgbaColor=_rgba("#c2c9d1"),
            physicsClientId=self._client,
        )

        specs = [
            ([0.055, 0.022, 0.022], [0.11, 0.000, 0.010], (0.0, 0.0, 0.0), _rgba("#c2c9d1")),
            ([0.014, 0.014, 0.014], [0.12, 0.000, -0.022], (0.0, 0.0, 0.0), _rgba("#1e293b")),
            ([0.028, 0.280, 0.004], [-0.01, 0.000, -0.002], (0.0, 0.0, 0.0), _rgba("#bcc4cc")),
            ([0.022, 0.004, 0.055], [-0.14, 0.036, 0.042], (-0.73, 0.0, 0.0), _rgba("#bcc4cc")),
            ([0.022, 0.004, 0.055], [-0.14, -0.036, 0.042], (0.73, 0.0, 0.0), _rgba("#bcc4cc")),
            ([0.022, 0.004, 0.028], [-0.14, 0.000, -0.028], (0.0, 0.0, 0.0), _rgba("#334155")),
            ([0.016, 0.014, 0.014], [-0.18, 0.000, 0.004], (0.0, 0.0, 0.0), _rgba("#e2e8f0")),
        ]

        masses, cols, viss, poss, oris, parents, jtypes, jaxes = [], [], [], [], [], [], [], []
        for hext, lpos, lrpy, col_rgba in specs:
            l_col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=hext, physicsClientId=self._client)
            l_vis = pb.createVisualShape(pb.GEOM_BOX, halfExtents=hext, rgbaColor=col_rgba, physicsClientId=self._client)
            masses.append(0)
            cols.append(l_col)
            viss.append(l_vis)
            poss.append(lpos)
            oris.append(_rpy_to_quat(*lrpy))
            parents.append(0)
            jtypes.append(pb.JOINT_FIXED)
            jaxes.append([0, 0, 1])

        return pb.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=base_col,
            baseVisualShapeIndex=base_vis,
            basePosition=[0, 0, 0],
            linkMasses=masses,
            linkCollisionShapeIndices=cols,
            linkVisualShapeIndices=viss,
            linkPositions=poss,
            linkOrientations=oris,
            linkInertialFramePositions=[[0, 0, 0]] * len(masses),
            linkInertialFrameOrientations=[[0, 0, 0, 1]] * len(masses),
            linkParentIndices=parents,
            linkJointTypes=jtypes,
            linkJointAxis=jaxes,
            physicsClientId=self._client,
        )

    def _make_target(self):
        col = pb.createCollisionShape(pb.GEOM_SPHERE, radius=0.08, physicsClientId=self._client)
        vis = pb.createVisualShape(pb.GEOM_SPHERE, radius=0.08, rgbaColor=_rgba("#22d3a5", 0.85), physicsClientId=self._client)
        return pb.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[0, 0, C.HOVER_ALT],
            physicsClientId=self._client,
        )

    def _make_obstacles(self):
        ids = []
        for ox, oy, oz, orad in C.OBSTACLES:
            col = pb.createCollisionShape(pb.GEOM_SPHERE, radius=orad, physicsClientId=self._client)
            vis = pb.createVisualShape(pb.GEOM_SPHERE, radius=orad, rgbaColor=_rgba("#ef4444", 0.75), physicsClientId=self._client)
            body = pb.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=[ox, oy, oz],
                physicsClientId=self._client,
            )
            ids.append(body)
        return ids

    def _draw_grid(self):
        g = C.GRID_SIZE
        col = [0.1, 0.17, 0.27]
        for i in range(-g, g + 1):
            pb.addUserDebugLine([i, -g, 0.001], [i, g, 0.001], col, lineWidth=0.5, physicsClientId=self._client)
            pb.addUserDebugLine([-g, i, 0.001], [g, i, 0.001], col, lineWidth=0.5, physicsClientId=self._client)

        pb.addUserDebugLine([0, 0, 0.01], [1, 0, 0.01], [0.8, 0.1, 0.1], lineWidth=2, physicsClientId=self._client)
        pb.addUserDebugLine([0, 0, 0.01], [0, 1, 0.01], [0.1, 0.8, 0.1], lineWidth=2, physicsClientId=self._client)
        pb.addUserDebugLine([0, 0, 0.01], [0, 0, 1.01], [0.2, 0.4, 0.9], lineWidth=2, physicsClientId=self._client)

    def update(self, quad, target, sim_time, collision=False, phase=None):
        pos = quad.pos.tolist()
        att = quad.att.tolist()
        quat = _rpy_to_quat(*att)

        pb.resetBasePositionAndOrientation(self._drone_id, pos, quat, physicsClientId=self._client)
        pb.resetBasePositionAndOrientation(
            self._target_id,
            [target[0], target[1], target[2]],
            [0, 0, 0, 1],
            physicsClientId=self._client,
        )

        if C.SHOW_TRAIL and self._prev_pos is not None:
            line_id = pb.addUserDebugLine(
                self._prev_pos,
                pos,
                lineColorRGB=[0.23, 0.51, 0.96],
                lineWidth=1.5,
                physicsClientId=self._client,
            )
            self._trail_ids.append(line_id)
            if len(self._trail_ids) > C.TRAIL_LENGTH:
                pb.removeUserDebugItem(self._trail_ids.pop(0), physicsClientId=self._client)

        self._prev_pos = pos[:]

        if collision:
            pb.changeVisualShape(self._drone_id, -1, rgbaColor=_rgba("#ef4444", 0.9), physicsClientId=self._client)

        if self._hud_id is not None:
            pb.removeUserDebugItem(self._hud_id, physicsClientId=self._client)

        xy_err = math.sqrt((quad.pos[0] - target[0]) ** 2 + (quad.pos[1] - target[1]) ** 2)
        spd = float(np.linalg.norm(quad.vel))
        if collision:
            mode = "COLLISION"
        elif phase:
            mode = phase
        else:
            mode = "CIRCLE" if sim_time > C.TAKEOFF_RAMP else "TAKEOFF"

        hud_str = (
            f"t={sim_time:6.2f}s  z={quad.pos[2]:.2f}m  "
            f"err={xy_err:.2f}m  spd={spd:.2f}m/s  [{mode}]  "
            f"roll={math.degrees(att[0]):+.1f}  pitch={math.degrees(att[1]):+.1f}"
        )

        self._hud_id = pb.addUserDebugText(
            hud_str,
            textPosition=[-C.GRID_SIZE + 0.3, -C.GRID_SIZE + 0.3, 0.3],
            textColorRGB=[0.58, 0.64, 0.74],
            textSize=1.1,
            physicsClientId=self._client,
        )

        pb.stepSimulation(physicsClientId=self._client)

    def close(self):
        print("  Window open — close it to exit.")
        try:
            while pb.isConnected(self._client):
                pb.stepSimulation(physicsClientId=self._client)
        except Exception:
            pass
        if pb.isConnected(self._client):
            pb.disconnect(self._client)
