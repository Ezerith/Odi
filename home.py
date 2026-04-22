# @Title: Home Position
# @Description: Moves all servos to their home/resting positions.
#               Run this before any other movement to ensure the robot
#               starts from a known safe position.

from pylx16a.lx16a import *
import time

# ---------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------
PORT = "/dev/ttyUSB0"

# Time in milliseconds to move to home position (smooth movement)
MOVE_TIME = 2000  # 2 seconds

# Home positions for each servo
# Hip servos — home at 140 degrees
HIP_SERVOS = {
    2: 140,   # Front Left  Hip
    3: 140,   # Front Right Hip
    5: 140,   # Back Left   Hip
    7: 140,   # Back Right  Hip
}

# Knee servos — home at 90 degrees
KNEE_SERVOS = {
    1: 90,    # Front Left  Knee
    4: 90,    # Front Right Knee
    6: 90,    # Back Left   Knee
    8: 90,    # Back Right  Knee
}

# Combine into one dictionary
ALL_SERVOS = {**HIP_SERVOS, **KNEE_SERVOS}

# ---------------------------------------------------------------

def connect_servos():
    """Connect to all servos and return a dictionary of servo objects."""
    servos = {}
    for servo_id, angle in ALL_SERVOS.items():
        try:
            servos[servo_id] = LX16A(servo_id)
            print(f"  Connected to Servo ID {servo_id}")
        except ServoTimeoutError:
            print(f"  [WARNING] Servo ID {servo_id} not responding — skipping.")
    return servos


def go_home(servos):
    """Send all connected servos to their home positions."""
    print("\nSending all servos to home position...")

    for servo_id, servo in servos.items():
        target_angle = ALL_SERVOS[servo_id]
        try:
            servo.move(target_angle, time=MOVE_TIME)
            print(f"  Servo ID {servo_id} moving -> {target_angle} degrees")
        except ServoTimeoutError:
            print(f"  [ERROR] Lost connection to Servo ID {servo_id}")

    # Wait for all servos to finish moving
    time.sleep(MOVE_TIME / 1000 + 0.5)
    print("All servos reached home position!")


def main():
    print("=" * 50)
    print("         ROBOT HOME POSITION")
    print("=" * 50)
    print("\nHome angles:")
    print(f"  Hip  servos (2,3,5,7) -> 140 degrees")
    print(f"  Knee servos (1,4,6,8) -> 90 degrees")
    print(f"\nMovement time: {MOVE_TIME}ms")
    print("\nInitializing port...")

    # Initialize the servo controller
    try:
        LX16A.initialize(PORT)
        print(f"  Port {PORT} opened successfully.")
    except Exception as e:
        print(f"  ERROR: Could not open port {PORT}")
        print(f"  Details: {e}")
        print("  Try running: ls /dev/ttyUSB* to find the correct port")
        exit()

    # Connect to all servos
    print("\nConnecting to servos...")
    servos = connect_servos()

    if not servos:
        print("\nERROR: No servos responded. Check your connections and power.")
        exit()

    print(f"\n{len(servos)}/{len(ALL_SERVOS)} servos connected.")

    # Move to home
    go_home(servos)

    print("\n" + "=" * 50)
    print("  Robot is in home position. Safe to proceed!")
    print("=" * 50)


if __name__ == '__main__':
    main()
