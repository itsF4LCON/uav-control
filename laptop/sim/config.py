# plane setup
MASS = 0.45              # weight in kg
INERTIA = (0.018, 0.024, 0.035)
GRAVITY = 9.81

CRUISE_SPEED = 4.5       # how fast it cruises normally
MIN_SPEED = 2.2          # stall speed, below this it drops
MAX_SPEED = 8.0          # top speed
MAX_THRUST = 5.0         # max motor push

WING_AREA = 0.55         # wing size in m2
AIR_DENSITY = 1.225
CL0 = 0.40
CL_ALPHA = 5.5
CD0 = 0.04
CD_INDUCED = 0.05
STALL_ALPHA = 0.40       # max angle before wings stall

SERVO_TAU = 0.02         # how fast flaps react
MOTOR_TAU = 0.04         # how fast motor spins up
MOTOR_MAX_THRUST = MAX_THRUST

# pixhawk hardware setup (for pi5_bridge.py)
PIXHAWK_MODE = "GUIDED"  # mode to switch pixhawk into on launch
AUTO_ARM_ON_LAUNCH = True
SERVO_AIL = 1            # aileron channel on pixhawk
SERVO_ELV = 2            # elevator / v-tail channel
SERVO_THR = 3            # throttle ESC channel
SERVO_RUD = 4            # rudder / v-tail channel

# sim timing
DT = 0.005               # physics step (dont touch)
DURATION = 90.0          # how long a battery lasts in seconds
STEPS_PER_FRAME = 4
LOW_BATTERY_THRESHOLD = 0.20  # auto lands at 20% battery

# flight settings
FLIGHT_MODE = "loiter"
HOVER_ALT = 2.2          # default height in meters
HOVER_POINT = (0.0, 0.0, 2.2)
CIRCLE_RADIUS = 4.5      # orbit circle size
CIRCLE_PERIOD = 14.0
TAKEOFF_RAMP = 3.0

DEMO_WAYPOINTS = [
    (0.0, 0.0, 2.2, 90.0),
]

# autopilot & pid gains (dont touch unless tuning)
ORBIT_KP = 0.75
COURSE_KP = 2.0
MAX_BANK_DEG = 40.0      # max tilt in turns
MAX_PITCH_DEG = 22.0     # max nose up/down
MAX_TILT_DEG = 40.0

ATT_KP = 45.0
ATT_KD = 10.0

ALT_KP = 0.45
ALT_KI = 0.08
ALT_KD = 0.25
ALT_LIMIT = 0.48

SPEED_KP = 0.65
SPEED_KI = 0.25
SPEED_KD = 0.05

POS_KP = 1.2
POS_KI = 0.2
POS_KD = 1.0
POS_LIMIT = 6.0

# obstacles (off for open field)
DRONE_RADIUS = 0.15
AVOID_ENABLED = False
AVOID_MARGIN = 0.35
REPULSE_RANGE = 0.6
REPULSE_GAIN = 0.8
REPULSE_DAMP = 6.0
REPULSE_MAX = 12.0

OBSTACLE_PRESET = "none"
RANDOM_OBSTACLES = 0
OBSTACLE_PRESETS = {
    "none": [],
    "single": [(-3.0, 0.0, 2.0, 0.40)],
    "ring": [
        (-3.0, 0.0, 2.0, 0.40),
        (0.0, 3.0, 2.1, 0.35),
        (3.0, 0.0, 1.9, 0.35),
        (0.0, -3.0, 2.0, 0.40),
    ],
    "gauntlet": [
        (-3.0, 0.0, 2.0, 0.40),
        (-2.1, 2.1, 2.2, 0.35),
        (0.0, 3.0, 1.8, 0.40),
        (2.1, 2.1, 2.0, 0.30),
        (3.0, 0.0, 2.1, 0.40),
        (0.0, -3.0, 1.9, 0.45),
    ],
}
OBSTACLES = list(OBSTACLE_PRESETS.get(OBSTACLE_PRESET, []))

# wind & sensors
SEED = None
MOTORS_ENABLED = True
ARM_LENGTH = 0.12
YAW_DRAG_COEFF = 0.016
AERO_ENABLED = True
DRAG_LINEAR = 0.15
DRAG_QUADRATIC = 0.02
GROUND_EFFECT = True
PROP_RADIUS = 0.06

WIND_ENABLED = True
WIND_MEAN = (0.45, 0.25, 0.0)   # base wind speed
WIND_GUST_STD = 0.20            # random wind gusts
WIND_GUST_TAU = 3.5
WIND_VERTICAL_RATIO = 0.2

SENSORS_ENABLED = True
POS_NOISE_STD = 0.02
VEL_NOISE_STD = 0.05
ATT_NOISE_STD = 0.005
GYRO_NOISE_STD = 0.02
GYRO_BIAS_STD = 0.005
POS_UPDATE_HZ = 40.0
POS_LATENCY_S = 0.02

RANDOMIZE_PARAMS = True
RANDOMIZE_FRAC = 0.10
CG_OFFSET_STD = 0.002
BATTERY_SAG = 0.08

# viewer settings
TRAIL_LENGTH = 500
SHOW_TRAIL = True
CAMERA_DIST = 9.0
CAMERA_YAW = 45.0
CAMERA_PITCH = -28.0
CAMERA_AZIM = 45.0
CAMERA_ELEV = 28.0
GRID_SIZE = 12

GODOT_HOST = "127.0.0.1"
GODOT_PORT = 9999
GODOT_CMD_PORT = 9998
