# uav-control

Fixed-wing UAV (miniature MQ-9 Reaper) flight control, simulator, OpenStreetMap/Satellite Web Ground Control Station, and Raspberry Pi 5 + Pixhawk MAVLink companion stack.

## Folder Structure

- `laptop/`
  - `sim/` — 6-DOF fixed-wing aerodynamics & L1/TECS controller, OpenStreetMap/Satellite Web GCS (`http://127.0.0.1:8080`), and UDP telemetry bridge.
  - `godot/` — Godot 4 3D real-time viewer (`drone.gd`, `main.gd`, `udp_receiver.gd`).
- `pi/`
  - `pi5_bridge.py` — Raspberry Pi 5 companion script that connects to the Pixhawk over MAVLink (`/dev/ttyAMA0`), runs the L1/TECS controller, serves the Web UI over Wi-Fi (`http://<pi-ip>:8080`), and streams live telemetry to the laptop.

## Quick Start

### 1. Laptop (Simulation + Web Map + Godot 3D)

```bash
cd laptop/sim
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python main_godot.py --loop
```

- Open **`http://127.0.0.1:8080`** in your browser for the live OpenStreetMap / Satellite Ground Control Station.
- Click **`SET HOME`** (or **`MY GPS`**) to pin `(0, 0)` on the field, click **`START FLIGHT (THROW)`** when launching, click anywhere on the map to command a `LOITER` or `FLY TO` point, and click **`LAND NOW`** to return home and land.
- Optional 3D view: open `laptop/godot/project.godot` in Godot 4 and press **F5**.

### 2. Raspberry Pi 5 (Real Flight with Pixhawk)

Copy the `pi/` folder to the Raspberry Pi 5:

```bash
cd pi
pip install -r requirements.txt
python3 pi5_bridge.py --mavlink /dev/ttyAMA0 --baud 921600 --laptop-ip <LAPTOP_IP>
```
