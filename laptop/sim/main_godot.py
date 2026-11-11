#!/usr/bin/env python3
import argparse
import time

import config as C
from drone import World
from godot_bridge import GodotBridge


def simulate_once(bridge, speed):
    auto_launch = bridge.web_server is None
    world = World(resolve_obstacles=True, auto_launch=auto_launch)
    quad = world.quad
    bridge.new_run()

    sim_time, frame_dt = 0.0, C.DT * C.STEPS_PER_FRAME
    collision, hit_idx = False, -1
    last_phase = None

    while sim_time < world.duration or bridge.web_server is not None:
        frame_start = time.perf_counter()
        for _ in range(C.STEPS_PER_FRAME):
            hit, idx = world.step(C.DT)
            sim_time = world.t
            phase_key = world.phase.split("(")[0].strip()
            if phase_key != last_phase:
                last_phase = phase_key
                p = quad.pos
                print(
                    f"  [t={sim_time:5.1f}s] {world.phase:28s}  "
                    f"pos=({p[0]:+5.2f}, {p[1]:+5.2f}, {p[2]:5.2f}) m  "
                    f"Va={quad.airspeed:4.2f} m/s  bank={quad.att[0]*57.2958:+5.1f} deg"
                )
            if hit:
                collision, hit_idx = True, idx
                print(f"  !! COLLISION with obstacle {idx} at t={sim_time:.2f}s pos={quad.pos.round(3)}")
                break

        bridge.send(world, collision, hit_idx)

        if collision:
            break

        if speed > 0:
            elapsed = time.perf_counter() - frame_start
            sleep_s = (frame_dt / speed) - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    if not collision:
        print(f"  Flight complete: {sim_time:.1f}s")

    for _ in range(5):
        bridge.send(world, collision, hit_idx)
        time.sleep(frame_dt)
    return collision


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--web-port", type=int, default=8080)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--pause", type=float, default=2.0)
    ap.add_argument("--mode", choices=["loiter", "waypoints", "hover", "circle", "figure8", "demo"], default=None)
    ap.add_argument("--obstacles", choices=["none", "single", "ring", "gauntlet", "custom"], default=None)
    ap.add_argument("--random-obstacles", type=int, default=None)
    args = ap.parse_args()

    if args.mode is not None:
        C.FLIGHT_MODE = args.mode
    if args.obstacles is not None:
        C.OBSTACLE_PRESET = args.obstacles
    if args.random_obstacles is not None:
        C.RANDOM_OBSTACLES = args.random_obstacles

    bridge = GodotBridge(args.host, args.port, web_port=args.web_port)
    print("=" * 60)
    print(f"  UAV Sim -> Godot  udp://{bridge.addr[0]}:{bridge.addr[1]}")
    if bridge.web_server is not None:
        print(f"  Web Control App   http://127.0.0.1:{bridge.web_port}")
    print("=" * 60)

    try:
        while True:
            simulate_once(bridge, args.speed)
            if not args.loop:
                break
            time.sleep(args.pause)
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
