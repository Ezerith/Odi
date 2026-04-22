# @Title: Walk 2 — Sequential Alternating Gait
# @Description:
#   Hips  (2,3,5,7) control UP/DOWN (lifting the leg)
#   Knees (1,4,6,8) control FORWARD/BACK (pushing the robot)
#
#   Gait cycle:
#     1. Pair A knees push back (robot moves forward), hips stay down
#     2. Pair A hips lift up, Pair B hips lower down simultaneously
#     3. Pair A knees return forward while raised
#     4. Pair A hips lower, Pair B hips lift simultaneously
#     5. Repeat with Pair B pushing
#
#   One pair is always on the ground pushing while the other is raised

from pylx16a.lx16a import *
from math import sin, cos
import time

# ---------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------
PORT = "/dev/ttyUSB0"

# ---------------------------------------------------------------
# HIP LIMITS — controls up/down (lifting)
# ---------------------------------------------------------------
HIP_MIN  = 110
HIP_MAX  = 150
HIP_HOME = 140   # neutral resting position
HIP_DOWN = 130   # leg on the ground (low angle = leg down)
HIP_UP   = 150   # leg lifted in the air (high angle = leg up)

# ---------------------------------------------------------------
# KNEE LIMITS — controls forward/back (pushing)
# ---------------------------------------------------------------
KNEE_MIN    = 60
KNEE_MAX    = 135
KNEE_HOME   = 90    # neutral knee position
KNEE_PUSH   = 75    # knee pushed back — drives robot forward
KNEE_REACH  = 105   # knee reached forward — ready for next push

# ---------------------------------------------------------------
# TIMING
# ---------------------------------------------------------------
SLEEP_TIME   = 0.1    # seconds between commands
T_INCREMENT  = 0.05   # sine wave step size — smaller = slower smoother gait
MOVE_TIME_MS = 600    # ms — must be larger than SLEEP_TIME for smooth motion

# Hip sine wave — oscillates between HIP_DOWN and HIP_UP
HIP_MID = (HIP_DOWN + HIP_UP) / 2       # center of hip oscillation
HIP_AMP = (HIP_UP   - HIP_DOWN) / 2     # amplitude of hip oscillation

# Knee sine wave — oscillates between KNEE_PUSH and KNEE_REACH
KNEE_MID = (KNEE_PUSH + KNEE_REACH) / 2  # center of knee oscillation
KNEE_AMP = (KNEE_REACH - KNEE_PUSH) / 2  # amplitude of knee oscillation

# ---------------------------------------------------------------
# SERVO ID MAPPING
#   Pair A: Hip 2 + Knee 1  AND  Hip 5 + Knee 4
#   Pair B: Hip 3 + Knee 6  AND  Hip 7 + Knee 8
#
#   Within each pair, legs are on opposite sides so knees mirror each other
#   Knee[0] pushes to KNEE_PUSH, Knee[1] mirrors to KNEE_HOME-(KNEE_PUSH-KNEE_HOME)
# ---------------------------------------------------------------

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def mirror_knee(angle):
    """Mirror a knee angle around KNEE_HOME for the opposite side leg."""
    return clamp(KNEE_HOME - (angle - KNEE_HOME), KNEE_MIN, KNEE_MAX)


def connect_servos():
    print("Connecting to servos...")
    try:
        hip_a1  = LX16A(2)
        knee_a1 = LX16A(1)
        hip_a2  = LX16A(5)
        knee_a2 = LX16A(4)
        hip_b1  = LX16A(3)
        knee_b1 = LX16A(6)
        hip_b2  = LX16A(7)
        knee_b2 = LX16A(8)
        print("  All 8 servos connected successfully.")
    except ServoTimeoutError as e:
        print(f"  ERROR: Servo ID {e.id_} is not responding.")
        exit()

    return {
        "hip_a1":  hip_a1,  "knee_a1": knee_a1,
        "hip_a2":  hip_a2,  "knee_a2": knee_a2,
        "hip_b1":  hip_b1,  "knee_b1": knee_b1,
        "hip_b2":  hip_b2,  "knee_b2": knee_b2,
    }


def get_walk_start_angles():
    """Start position — Pair A down and reached forward, Pair B lifted."""
    return {
        "hip_a1":  HIP_DOWN,  "knee_a1": KNEE_REACH,
        "hip_a2":  HIP_DOWN,  "knee_a2": mirror_knee(KNEE_REACH),
        "hip_b1":  HIP_UP,    "knee_b1": KNEE_REACH,
        "hip_b2":  HIP_UP,    "knee_b2": mirror_knee(KNEE_REACH),
    }


def smooth_transition(servos, transition_time=3.0, steps=30):
    """Smoothly move from current position to walk start position."""
    print("\nReading current servo positions...")
    current_angles = {}
    for name, servo in servos.items():
        try:
            current_angles[name] = servo.get_physical_angle()
            print(f"  {name}: {current_angles[name]:.1f} degrees")
        except ServoTimeoutError:
            print(f"  [WARNING] Could not read {name}, using default")
            current_angles[name] = HIP_DOWN if "hip" in name else KNEE_HOME

    target_angles = get_walk_start_angles()
    print(f"\nSmoothing to walk start position over {transition_time} seconds...")

    step_time = transition_time / steps
    for step in range(1, steps + 1):
        alpha = step / steps
        try:
            for name, servo in servos.items():
                interpolated = current_angles[name] + alpha * (target_angles[name] - current_angles[name])
                if "hip" in name:
                    angle = clamp(interpolated, HIP_MIN, HIP_MAX)
                else:
                    angle = clamp(interpolated, KNEE_MIN, KNEE_MAX)
                servo.move(angle, time=int(step_time * 1000))
        except ServoTimeoutError as e:
            print(f"  [WARNING] Lost contact with Servo ID {e.id_} during transition")
        time.sleep(step_time)

    print("  Ready — starting walk gait!\n")


# Servo 7 is slightly faulty — always runs 5 degrees lower than servo 3
SERVO_7_OFFSET = -5


def set_hips(servos, hip_keys, angle, move_time):
    """Move a set of hips to an angle — applies offset to servo 7 (hip_b2)."""
    try:
        for key in hip_keys:
            if key == "hip_b2":  # hip_b2 = servo 7
                adjusted = clamp(angle + SERVO_7_OFFSET, HIP_MIN, HIP_MAX)
                servos[key].move(adjusted, time=move_time)
            else:
                servos[key].move(angle, time=move_time)
    except ServoTimeoutError as e:
        print(f"  [WARNING] Lost contact with {e.id_}")


def set_knees(servos, knee_keys, angle, move_time):
    """Move a pair of knees — second knee is mirrored."""
    try:
        servos[knee_keys[0]].move(angle,               time=move_time)
        servos[knee_keys[1]].move(mirror_knee(angle),  time=move_time)
    except ServoTimeoutError as e:
        print(f"  [WARNING] Lost contact with {e.id_}")


def go_home(servos):
    """Return all servos to home position."""
    print("Returning to home position...")
    try:
        for key in ["hip_a1", "hip_a2", "hip_b1"]:
            servos[key].move(HIP_HOME, time=1500)
        # Apply offset to servo 7 (hip_b2)
        servos["hip_b2"].move(clamp(HIP_HOME + SERVO_7_OFFSET, HIP_MIN, HIP_MAX), time=1500)
        for key in [("knee_a1", "knee_a2"), ("knee_b1", "knee_b2")]:
            servos[key[0]].move(KNEE_HOME,              time=1500)
            servos[key[1]].move(mirror_knee(KNEE_HOME), time=1500)
    except ServoTimeoutError as e:
        print(f"  [WARNING] {e.id_}")
    time.sleep(1.8)
    print("  Home position reached.")


def walk(servos):
    """
    Sine wave gait — Pair A and Pair B offset by pi:
      - Hips  follow sin(t): UP during swing, DOWN during stance
      - Knees follow sin(t): REACH during swing, PUSH during stance
      - Pair B is offset by pi so it is always opposite to Pair A
    """
    a_hips  = ["hip_a1",  "hip_a2"]
    a_knees = ["knee_a1", "knee_a2"]
    b_hips  = ["hip_b1",  "hip_b2"]
    b_knees = ["knee_b1", "knee_b2"]

    print("Starting sine wave gait. Press Ctrl+C to stop.\n")

    t = 0
    while True:
        s_a = sin(t)
        s_b = sin(t + 3.14159)

        # Hips — higher angle = leg up, lower = leg down
        hip_a_angle = clamp(HIP_MID + s_a * HIP_AMP,  HIP_MIN, HIP_MAX)
        hip_b_angle = clamp(HIP_MID + s_b * HIP_AMP,  HIP_MIN, HIP_MAX)

        # Knees — higher angle = reach forward, lower = push back
        knee_a_angle = clamp(KNEE_MID + s_a * KNEE_AMP, KNEE_MIN, KNEE_MAX)
        knee_b_angle = clamp(KNEE_MID + s_b * KNEE_AMP, KNEE_MIN, KNEE_MAX)

        set_hips(servos,  a_hips,  hip_a_angle,  MOVE_TIME_MS)
        set_hips(servos,  b_hips,  hip_b_angle,  MOVE_TIME_MS)
        set_knees(servos, a_knees, knee_a_angle, MOVE_TIME_MS)
        set_knees(servos, b_knees, knee_b_angle, MOVE_TIME_MS)

        t += T_INCREMENT
        time.sleep(SLEEP_TIME)


def main():
    print("=" * 50)
    print("     WALK 2 — SEQUENTIAL ALTERNATING GAIT")
    print("=" * 50)
    print(f"\nHips  — mid: {HIP_MID} | amp: {HIP_AMP} | range: {HIP_DOWN}-{HIP_UP}")
    print(f"Knees — mid: {KNEE_MID} | amp: {KNEE_AMP} | range: {KNEE_PUSH}-{KNEE_REACH}")
    print(f"Step: {T_INCREMENT} | Sleep: {SLEEP_TIME}s | Move: {MOVE_TIME_MS}ms")

    try:
        LX16A.initialize(PORT)
        print(f"\nPort {PORT} opened successfully.")
    except Exception as e:
        print(f"ERROR: Could not open port {PORT}: {e}")
        exit()

    servos = connect_servos()
    smooth_transition(servos)

    try:
        walk(servos)
    except KeyboardInterrupt:
        print("\n\nStopped by user.")
        go_home(servos)


if __name__ == '__main__':
    main()
