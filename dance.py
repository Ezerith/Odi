# @Title: Walk
# @Description: Moves the quadruped robot in a walking gait using sine waves.
#               Hip servos oscillate between 110-150 degrees.
#               Knee servos oscillate between 45-135 degrees.
#               Diagonal leg pairs move together for a natural trot gait.

from pylx16a.lx16a import *
from math import sin, cos
import time

# ---------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------
PORT = "/dev/ttyUSB0"

SLEEP_TIME   = 0.6    # must match MOVE_TIME_MS so each move fully completes
T_INCREMENT  = 0.5    # larger steps to sweep the full sine wave range
MOVE_TIME_MS = 600    # servo takes 600ms per move — matches SLEEP_TIME

# ---------------------------------------------------------------
# SERVO ANGLE LIMITS
# ---------------------------------------------------------------
HIP_MIN  = 110
HIP_MAX  = 150
HIP_MID  = (HIP_MAX + HIP_MIN) / 2      # 130 degrees — center of hip range
HIP_AMP  = (HIP_MAX - HIP_MIN) / 2      # 20 degrees  — max swing from center

KNEE_MIN = 60
KNEE_MAX = 135
KNEE_MID = (KNEE_MAX + KNEE_MIN) / 2    # 90 degrees  — center of knee range
KNEE_AMP = (KNEE_MAX - KNEE_MIN) / 2    # 45 degrees  — max swing from center

# ---------------------------------------------------------------
# SERVO ID MAPPING
# Hip  servos: 2, 3, 5, 7
# Knee servos: 1, 4, 6, 8
# ---------------------------------------------------------------
# Leg layout:
#   Front Left  — Hip: 2,  Knee: 1
#   Front Right — Hip: 3,  Knee: 4
#   Back  Left  — Hip: 5,  Knee: 6
#   Back  Right — Hip: 7,  Knee: 8

# ---------------------------------------------------------------

def clamp(value, min_val, max_val):
    """Make sure a value never exceeds the allowed servo range."""
    return max(min_val, min(max_val, value))


def connect_servos():
    """Connect to all 8 servos and return them in a dictionary."""
    print("Connecting to servos...")
    try:
        # Leg pairs:
        #   Leg A1: Hip 2 + Knee 1
        #   Leg A2: Hip 5 + Knee 4  (diagonal opposite of A1)
        #   Leg B1: Hip 3 + Knee 6
        #   Leg B2: Hip 7 + Knee 8  (diagonal opposite of B1)
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
        print(f"  ERROR: Servo ID {e.id_} is not responding. Check connections.")
        exit()

    return {
        "hip_a1":  hip_a1,
        "knee_a1": knee_a1,
        "hip_a2":  hip_a2,
        "knee_a2": knee_a2,
        "hip_b1":  hip_b1,
        "knee_b1": knee_b1,
        "hip_b2":  hip_b2,
        "knee_b2": knee_b2,
    }


def get_walk_start_angles():
    """Returns the servo angles at t=0 of the walk gait (the walk start position)."""
    return {
        "hip_a1":  clamp(HIP_MID  + sin(0)    * HIP_AMP,  HIP_MIN,  HIP_MAX),
        "knee_a1": clamp(KNEE_MID - max(0, sin(0))    * KNEE_AMP, KNEE_MIN, KNEE_MAX),
        "hip_a2":  clamp(HIP_MID  + sin(0)    * HIP_AMP,  HIP_MIN,  HIP_MAX),
        "knee_a2": clamp(KNEE_MID - max(0, sin(0))    * KNEE_AMP, KNEE_MIN, KNEE_MAX),
        "hip_b1":  clamp(HIP_MID  + sin(3.14) * HIP_AMP,  HIP_MIN,  HIP_MAX),
        "knee_b1": clamp(KNEE_MID - max(0, sin(3.14)) * KNEE_AMP, KNEE_MIN, KNEE_MAX),
        "hip_b2":  clamp(HIP_MID  + sin(3.14) * HIP_AMP,  HIP_MIN,  HIP_MAX),
        "knee_b2": clamp(KNEE_MID - max(0, sin(3.14)) * KNEE_AMP, KNEE_MIN, KNEE_MAX),
    }


def smooth_transition(servos, transition_time=3.0, steps=30):
    """
    Smoothly moves all servos from their current positions to the
    walk start position before the gait begins.
    transition_time: total seconds for the transition
    steps: number of incremental moves (more = smoother)
    """
    print("\nReading current servo positions...")

    # Read current angle of each servo
    current_angles = {}
    for name, servo in servos.items():
        try:
            current_angles[name] = servo.get_physical_angle()
            print(f"  {name} current angle: {current_angles[name]:.1f} degrees")
        except ServoTimeoutError:
            print(f"  [WARNING] Could not read {name}, defaulting to mid position")
            if "hip" in name:
                current_angles[name] = HIP_MID
            else:
                current_angles[name] = KNEE_MID

    # Get the target walk start angles
    target_angles = get_walk_start_angles()

    print(f"\nSmoothing to walk start position over {transition_time} seconds...")

    step_time = transition_time / steps

    for step in range(1, steps + 1):
        # Linear interpolation between current and target
        alpha = step / steps  # goes from 0.0 to 1.0
        try:
            for name, servo in servos.items():
                interpolated = current_angles[name] + alpha * (target_angles[name] - current_angles[name])
                if "hip" in name:
                    angle = clamp(interpolated, HIP_MIN, HIP_MAX)
                else:
                    angle = clamp(interpolated, KNEE_MIN, KNEE_MAX)
                servo.move(angle, time=int(step_time * 1000))        except ServoTimeoutError as e:
            print(f"  [WARNING] Lost contact with Servo ID {e.id_} during transition")

        time.sleep(step_time)

    print("  Ready — starting walk gait!\n")


def walk(servos):
    """
    Trot gait using sine waves.
    Diagonal pairs based on actual servo layout:
      - Pair A: Hip 2 + Knee 1  AND  Hip 5 + Knee 4  (in phase)
      - Pair B: Hip 3 + Knee 6  AND  Hip 7 + Knee 8  (offset by pi)
    """
    print("\nStarting walking gait. Press Ctrl+C to stop.\n")

    t = 0
    while True:
        # --- PAIR A: Hip 2+Knee 1 and Hip 5+Knee 4 (in phase) ---
        hip_a1_angle  = clamp(HIP_MID  + sin(t)        * HIP_AMP,  HIP_MIN,  HIP_MAX)
        hip_a2_angle  = clamp(HIP_MID  + sin(t)        * HIP_AMP,  HIP_MIN,  HIP_MAX)
        knee_a1_angle = clamp(KNEE_MID - max(0, sin(t)) * KNEE_AMP, KNEE_MIN, KNEE_MAX)
        knee_a2_angle = clamp(KNEE_MID - max(0, sin(t)) * KNEE_AMP, KNEE_MIN, KNEE_MAX)

        # --- PAIR B: Hip 3+Knee 6 and Hip 7+Knee 8 (opposite phase) ---
        hip_b1_angle  = clamp(HIP_MID  + sin(t + 3.14)        * HIP_AMP,  HIP_MIN,  HIP_MAX)
        hip_b2_angle  = clamp(HIP_MID  + sin(t + 3.14)        * HIP_AMP,  HIP_MIN,  HIP_MAX)
        knee_b1_angle = clamp(KNEE_MID - max(0, sin(t + 3.14)) * KNEE_AMP, KNEE_MIN, KNEE_MAX)
        knee_b2_angle = clamp(KNEE_MID - max(0, sin(t + 3.14)) * KNEE_AMP, KNEE_MIN, KNEE_MAX)

        # --- SEND COMMANDS ---
        try:
            servos["hip_a1"].move(hip_a1_angle,   time=MOVE_TIME_MS)
            servos["hip_a2"].move(hip_a2_angle,   time=MOVE_TIME_MS)
            servos["hip_b1"].move(hip_b1_angle,   time=MOVE_TIME_MS)
            servos["hip_b2"].move(hip_b2_angle,   time=MOVE_TIME_MS)

            servos["knee_a1"].move(knee_a1_angle, time=MOVE_TIME_MS)
            servos["knee_a2"].move(knee_a2_angle, time=MOVE_TIME_MS)
            servos["knee_b1"].move(knee_b1_angle, time=MOVE_TIME_MS)
            servos["knee_b2"].move(knee_b2_angle, time=MOVE_TIME_MS)

        except ServoTimeoutError as e:
            print(f"  [WARNING] Lost contact with Servo ID {e.id_} — retrying...")

        t += T_INCREMENT
        time.sleep(SLEEP_TIME)



def main():
    print("=" * 50)
    print("          QUADRUPED WALKING GAIT")
    print("=" * 50)
    print(f"\nHip  range : {HIP_MIN} - {HIP_MAX} degrees (center {HIP_MID})")
    print(f"Knee range : {KNEE_MIN} - {KNEE_MAX} degrees (center {KNEE_MID})")
    print(f"Command rate: {1/SLEEP_TIME:.0f}Hz")

    # Initialize port
    try:
        LX16A.initialize(PORT)
        print(f"\nPort {PORT} opened successfully.")
    except Exception as e:
        print(f"ERROR: Could not open port {PORT}: {e}")
        print("Try running: ls /dev/ttyUSB* to find the correct port")
        exit()

    # Connect to servos
    servos = connect_servos()

    # Start walking
    try:
        smooth_transition(servos)
        walk(servos)
    except KeyboardInterrupt:
        print("\n\nStopped by user. Run home.py to return to home position.")


if __name__ == '__main__':
    main()
