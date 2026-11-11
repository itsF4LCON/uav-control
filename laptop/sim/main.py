#!/usr/bin/env python3
import config as C
from drone import World
from sim import Visualizer


def run():
    world = World(resolve_obstacles=True, auto_launch=True)
    quad = world.quad
    viz = Visualizer()

    sim_time = 0.0
    frames = 0
    collision = False
    last_phase = None

    try:
        while sim_time < world.duration:
            for _ in range(C.STEPS_PER_FRAME):
                hit, obs_idx = world.step(C.DT)
                sim_time = world.t
                target = world.target
                phase_key = world.phase.split("(")[0].strip()
                if phase_key != last_phase:
                    last_phase = phase_key
                    p = quad.pos
                    print(f"  [t={sim_time:5.1f}s] {world.phase:28s}  pos=({p[0]:+5.2f}, {p[1]:+5.2f}, {p[2]:5.2f}) m")
                if hit:
                    collision = True
                    break

            viz.update(quad, target, sim_time, collision, phase=world.phase)
            frames += 1
            if collision:
                break

    except KeyboardInterrupt:
        pass

    viz.close()


if __name__ == "__main__":
    run()
