# @Title: Servo ID Test
# @Description: Moves each servo by 10 degrees one at a time, 5 seconds apart
#               so you can identify which servo is which on your quadruped robot.
#               Make sure only the servos you want to test are connected.

from pylx16a.lx16a import *
import time

# ---------------------------------------------------------------
# CONFIGURATION — change this list to match however many servos
# you have connected (e.g. [1,2,3,4,5,6,7,8])
# ---------------------------------------------------------------
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7, 8]

# The port your servo controller is connected to.
# On Raspberry Pi it is usually /dev/ttyUSB0
PORT = "/dev/ttyUSB0"

# How many degrees to move from the center position (120 degrees)
MOVE_DEGREES = 10

# Seconds to wait between each servo test
WAIT_SECONDS = 5

# Center position (neutral / resting angle)
CENTER = 120

# ---------------------------------------------------------------

def test_servo(servo, servo_id):
    """Move a single servo 10 degrees forward then back to center."""
    target = CENTER + MOVE_DEGREES
    print(f"\n>>> Testing Servo ID {servo_id}")
    print(f"    Moving from {CENTER} degrees to {target} degrees...")
    servo.move(target, time=1000)   # move over 1 second (smooth)
    time.sleep(2)
    print(f"    Returning to center ({CENTER} degrees)...")
    servo.move(CENTER, time=1000)
    time.sleep(2)
    print(f"    Done with Servo ID {servo_id}. Note which leg/joint moved!")
    print(f"    Waiting {WAIT_SECONDS} seconds before next servo...")
    time.sleep(WAIT_SECONDS)


def main():
    print("=" * 50)
    print("       SERVO ID IDENTIFICATION TEST")
    print("=" * 50)
    print(f"Testing servo IDs: {SERVO_IDS}")
    print(f"Port: {PORT}")
    print(f"Each servo will move {MOVE_DEGREES} degrees from center ({CENTER} deg)")
    print(f"There will be a {WAIT_SECONDS} second gap between each servo.")
    print("\nStarting in 3 seconds... get ready to watch!\n")
    time.sleep(3)

    # Initialize the controller
    try:
        LX16A.initialize(PORT)
    except Exception as e:
        print(f"ERROR: Could not open port {PORT}. Check your USB connection.")
        print(f"Details: {e}")
        print("Try running: ls /dev/ttyUSB* to see available ports")
        exit()

    # Connect to each servo and test it
    for servo_id in SERVO_IDS:
        try:
            servo = LX16A(servo_id)
        except ServoTimeoutError:
            print(f"\n[SKIP] Servo ID {servo_id} is not responding. Is it connected?")
            continue

        try:
            test_servo(servo, servo_id)
        except ServoTimeoutError:
            print(f"[ERROR] Lost connection to Servo ID {servo_id} during test.")
        except ServoChecksumError:
            print(f"[ERROR] Checksum error on Servo ID {servo_id}. Possible duplicate IDs.")

    print("\n" + "=" * 50)
    print("  TEST COMPLETE — all servos tested!")
    print("  Use your notes to map each ID to its leg/joint.")
    print("=" * 50)


if __name__ == '__main__':
    main()
