import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "sim"))
os.chdir(os.path.join(HERE, "sim"))

import config as C
from drone.world import World
from drone.flight import set_selected_point, request_launch, request_land
from godot_bridge import GodotBridge

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/replay.jsonl"
POINTS = [(7.0, 5.0), (-6.0, -4.0)]
EVENTS = [(0.0, "start"), (7.0, 0), (20.0, 1), (33.0, "land")]


class Capture:
    def __init__(self):
        self.lines = []

    def sendto(self, data, addr):
        self.lines.append(data.decode())

    def close(self):
        pass


def main():
    cap = Capture()
    bridge = GodotBridge(web_port=None)
    bridge.sock = cap
    world = World(seed=3, auto_launch=False)
    bridge.new_run()
    frame_t = 0.0
    pending = list(EVENTS)
    landed_for = 0.0
    while True:
        while pending and frame_t >= pending[0][0]:
            ev = pending.pop(0)[1]
            if ev == "start":
                request_launch()
            elif ev == "land":
                request_land()
            else:
                x, y = POINTS[ev]
                set_selected_point(x, y, 2.4, radius=3.0, mode="fly_to")
        for _ in range(C.STEPS_PER_FRAME):
            world.step(C.DT)
        bridge.send(world, False, -1)
        frame_t += C.DT * C.STEPS_PER_FRAME
        if world.disarmed:
            landed_for += C.DT * C.STEPS_PER_FRAME
            if landed_for > 2.0:
                break
        if frame_t > 80.0:
            break
    with open(OUT, "w") as f:
        f.write("\n".join(cap.lines))
    print(len(cap.lines), "frames", round(frame_t, 1), "s", world.phase)


if __name__ == "__main__":
    main()
