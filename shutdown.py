# @Title: Shutdown Routine
# @Description: Safely shuts down the robot by:
#   1. Running a final health check
#   2. Moving all servos to home position
#   3. Printing a full status report
#   4. Shutting down the Raspberry Pi cleanly
# Always run this instead of unplugging to protect the SD card.

from pylx16a.lx16a import *
import time
import os
import health_test  # reuse health check from health_test.py

# ---------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------
PORT     = "/dev/ttyUSB0"

# Home positions (must match home.py)
HIP_HOME  = 155   # degrees
KNEE_HOME = 90    # degrees

MOVE_TIME = 2000  # ms — slow smooth return to home

# Servo role labels
HIP_IDS  = [2, 3, 5, 7]
KNEE_IDS = [1, 4, 6, 8]
ALL_IDS  = HIP_IDS + KNEE_IDS

# ---------------------------------------------------------------

def connect_servos():
    """Connect to all servos, skipping any that don't respond."""
    servos = {}
    for servo_id in ALL_IDS:
        try:
            servos[servo_id] = LX16A(servo_id)
        except ServoTimeoutError:
            print(f"  [WARNING] Servo ID {servo_id} not responding — skipping.")
    return servos


def go_home(servos):
    """Smoothly move all servos to home position."""
    print("\nMoving all servos to home position...")
    try:
        for servo_id, servo in servos.items():
            if servo_id in HIP_IDS:
                servo.move(HIP_HOME, time=MOVE_TIME)
            else:
                servo.move(KNEE_HOME, time=MOVE_TIME)
            print(f"  Servo ID {servo_id} -> home position")
    except ServoTimeoutError as e:
        print(f"  [WARNING] Lost contact with Servo ID {e.id_} during homing")

    time.sleep(MOVE_TIME / 1000 + 0.5)
    print("  All servos at home position.")


def print_shutdown_report(servos, health_results):
    """Print a final status report before shutdown."""
    print("\n" + "=" * 60)
    print("            SHUTDOWN STATUS REPORT")
    print("=" * 60)

    print(f"\n  Servos homed     : {len(servos)}/{len(ALL_IDS)}")

    # Count health results
    passed  = sum(1 for r in health_results if r["overall"].startswith("PASS"))
    failed  = sum(1 for r in health_results if r["overall"].startswith("FAIL"))
    warned  = sum(1 for r in health_results if "warning" in r["overall"].lower())

    print(f"  Health passed    : {passed}/{len(ALL_IDS)}")
    if warned:
        print(f"  Warnings         : {warned} servo(s) ran warm")
    if failed:
        print(f"  Failures         : {failed} servo(s) had issues")

    # Print any servos that had issues
    issues = [r for r in health_results if not r["overall"].startswith("PASS")]
    if issues:
        print("\n  Issues to address before next run:")
        for r in issues:
            print(f"    - {r['label']}: {r['overall']}")
    else:
        print("\n  All servos healthy — good run!")

    print("\n" + "=" * 60)
    print("  Robot safely homed. Shutting down Raspberry Pi...")
    print("  Wait for green light to stop blinking before unplugging.")
    print("=" * 60)


def shutdown_pi():
    """Shut down the Raspberry Pi cleanly."""
    print("\nInitiating shutdown in 3 seconds...")
    time.sleep(3)
    os.system("sudo shutdown now")


def main():
    print("=" * 60)
    print("           ROBOT SHUTDOWN ROUTINE")
    print("=" * 60)

    # Initialize port
    try:
        LX16A.initialize(PORT)
        print(f"\nPort {PORT} opened successfully.")
    except Exception as e:
        print(f"ERROR: Could not open port {PORT}: {e}")
        exit()

    # Step 1: Run health check first
    print("\nStep 1: Running final health check...")
    health_results = []
    for servo_id in ALL_IDS:
        result = health_test.check_servo(servo_id)
        health_results.append(result)

    # Step 2: Connect and home all servos
    print("\nStep 2: Connecting to servos...")
    servos = connect_servos()

    if not servos:
        print("ERROR: No servos responded. Skipping home — shutting down anyway.")
    else:
        # Step 3: Move to home
        go_home(servos)

    # Step 4: Print full health report
    print("\nStep 3: Final health report:")
    health_test.print_report(health_results)

    # Step 5: Print shutdown summary
    print_shutdown_report(servos, health_results)

    # Step 6: Shut down Pi
    shutdown_pi()


if __name__ == '__main__':
    main()
