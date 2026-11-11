import json
import os
import signal
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
GODOT_BIN = os.environ.get("GODOT_BIN", "godot")
W, H = 1280, 720
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "docs", "demo.mp4")
POINTS = [(7.0, 5.0), (-6.0, -4.0)]


def cmd(**kw):
    req = urllib.request.Request(
        "http://127.0.0.1:8080/api/command",
        data=json.dumps(kw).encode(),
        headers={"Content-Type": "application/json"},
    )
    urllib.request.urlopen(req, timeout=3).read()


def window_geometry(pid):
    out = subprocess.run(["hyprctl", "clients", "-j"], capture_output=True, text=True).stdout
    for c in json.loads(out):
        if c.get("pid") == pid:
            return c["at"][0], c["at"][1], c["size"][0], c["size"][1]
    return None


def main():
    os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
    py = os.path.join(HERE, "sim", ".venv", "bin", "python")
    sim = subprocess.Popen([py, "main_godot.py"], cwd=os.path.join(HERE, "sim"))
    godot = subprocess.Popen(
        [GODOT_BIN, "--path", os.path.join(HERE, "godot"),
         "--windowed", "--resolution", f"{W}x{H}"]
    )
    rec = None
    try:
        geo = None
        for _ in range(40):
            time.sleep(1)
            geo = window_geometry(godot.pid)
            if geo:
                break
        time.sleep(4)
        sel = f"pid:{godot.pid}"
        subprocess.run(["hyprctl", "dispatch", "setfloating", sel])
        subprocess.run(["hyprctl", "dispatch", "resizewindowpixel", f"exact {W} {H},{sel}"])
        subprocess.run(["hyprctl", "dispatch", "movewindowpixel", f"exact 100 80,{sel}"])
        time.sleep(3)
        geo = window_geometry(godot.pid) or geo
        print("window", geo, flush=True)
        x, y, w, h = geo
        rec = subprocess.Popen(
            ["wf-recorder", "-g", f"{x},{y} {w}x{h}", "-f", OUT, "-y"]
        )
        time.sleep(3)
        cmd(cmd="start_flight")
        time.sleep(7)
        for px, py_ in POINTS:
            cmd(cmd="select_point", pos=[px, py_, 2.4], radius=3.0, mode="fly_to")
            time.sleep(13)
        cmd(cmd="land")
        time.sleep(22)
    finally:
        if rec is not None:
            rec.send_signal(signal.SIGINT)
            rec.wait(timeout=30)
        godot.terminate()
        sim.terminate()


if __name__ == "__main__":
    main()
