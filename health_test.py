# @Title: Health Test
# @Description: Checks the health of all servos by reading:
#   - Responsiveness (is the servo connected and responding?)
#   - Temperature (is the servo overheating?)
#   - Voltage (is the battery supplying enough power?)
#   - Current position (is the servo readable?)
# Prints a full health report at the end with pass/fail for each servo.

from pylx16a.lx16a import *
import time

# ---------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------
PORT = "/dev/ttyUSB0"

# Safety thresholds
MAX_TEMP_C     = 50   # degrees Celsius — above this is a warning
DANGER_TEMP_C  = 70   # degrees Celsius — above this is critical
MIN_VOLTAGE    = 6.0  # volts — below this battery is too low
MAX_VOLTAGE    = 8.4  # volts — above this is unexpected

# All servo IDs on the robot
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7, 8]

# Servo role labels for readable output
SERVO_LABELS = {
    1: "Knee A1 (ID 1)",
    2: "Hip  A1 (ID 2)",
    3: "Hip  B1 (ID 3)",
    4: "Knee A2 (ID 4)",
    5: "Hip  A2 (ID 5)",
    6: "Knee B1 (ID 6)",
    7: "Hip  B2 (ID 7)",
    8: "Knee B2 (ID 8)",
}

# ---------------------------------------------------------------

def check_servo(servo_id):
    """
    Run health checks on a single servo.
    Returns a dict with all results.
    """
    result = {
        "id":           servo_id,
        "label":        SERVO_LABELS.get(servo_id, f"Servo {servo_id}"),
        "responsive":   False,
        "temperature":  None,
        "voltage":      None,
        "position":     None,
        "temp_status":  "N/A",
        "volt_status":  "N/A",
        "overall":      "FAIL",
    }

    try:
        servo = LX16A(servo_id)
        result["responsive"] = True
    except ServoTimeoutError:
        result["overall"] = "FAIL — not responding"
        return result

    # Read temperature
    try:
        temp = servo.get_temp()
        result["temperature"] = temp
        if temp >= DANGER_TEMP_C:
            result["temp_status"] = f"CRITICAL ({temp}°C)"
        elif temp >= MAX_TEMP_C:
            result["temp_status"] = f"WARNING ({temp}°C)"
        else:
            result["temp_status"] = f"OK ({temp}°C)"
    except Exception:
        result["temp_status"] = "UNREADABLE"

    # Read voltage
    try:
        voltage = servo.get_vin() / 1000.0  # convert mV to V
        result["voltage"] = voltage
        if voltage < MIN_VOLTAGE:
            result["volt_status"] = f"LOW ({voltage:.2f}V)"
        elif voltage > MAX_VOLTAGE:
            result["volt_status"] = f"HIGH ({voltage:.2f}V)"
        else:
            result["volt_status"] = f"OK ({voltage:.2f}V)"
    except Exception:
        result["volt_status"] = "UNREADABLE"

    # Read current position
    try:
        position = servo.get_physical_angle()
        result["position"] = round(position, 1)
    except Exception:
        result["position"] = None

    # Overall pass/fail
    temp_ok = result["temp_status"].startswith("OK") or result["temp_status"].startswith("WARNING")
    volt_ok = result["volt_status"].startswith("OK")
    if result["responsive"] and temp_ok and volt_ok:
        if result["temp_status"].startswith("WARNING"):
            result["overall"] = "PASS (temp warning)"
        else:
            result["overall"] = "PASS"
    else:
        reasons = []
        if not result["responsive"]:
            reasons.append("not responding")
        if not temp_ok:
            reasons.append(f"temp {result['temp_status']}")
        if not volt_ok:
            reasons.append(f"voltage {result['volt_status']}")
        result["overall"] = "FAIL — " + ", ".join(reasons)

    return result


def print_report(results):
    """Print a formatted health report."""
    print("\n" + "=" * 60)
    print("              ROBOT HEALTH REPORT")
    print("=" * 60)

    pass_count = 0
    warn_count = 0
    fail_count = 0

    for r in results:
        status_icon = "✓" if r["overall"].startswith("PASS") else "✗"
        print(f"\n  {status_icon} {r['label']}")
        print(f"      Responsive : {'Yes' if r['responsive'] else 'NO'}")
        print(f"      Temperature: {r['temp_status']}")
        print(f"      Voltage    : {r['volt_status']}")
        print(f"      Position   : {r['position']}°" if r['position'] is not None else "      Position   : UNREADABLE")
        print(f"      Overall    : {r['overall']}")

        if r["overall"] == "PASS":
            pass_count += 1
        elif "WARNING" in r["overall"]:
            warn_count += 1
        else:
            fail_count += 1

    print("\n" + "-" * 60)
    print(f"  SUMMARY: {pass_count} passed | {warn_count} warnings | {fail_count} failed")
    print("-" * 60)

    if fail_count == 0 and warn_count == 0:
        print("  STATUS: ALL SYSTEMS GO ✓")
    elif fail_count == 0:
        print("  STATUS: OPERATIONAL — check warnings before long runs")
    else:
        print("  STATUS: ISSUES DETECTED — fix failures before running")
    print("=" * 60)

    return fail_count == 0


def main():
    print("=" * 60)
    print("           ROBOT HEALTH TEST ROUTINE")
    print("=" * 60)
    print(f"\nChecking {len(SERVO_IDS)} servos...")
    print(f"Temp thresholds : warning >{MAX_TEMP_C}°C | critical >{DANGER_TEMP_C}°C")
    print(f"Voltage range   : {MIN_VOLTAGE}V - {MAX_VOLTAGE}V")

    # Initialize port
    try:
        LX16A.initialize(PORT)
        print(f"\nPort {PORT} opened successfully.")
    except Exception as e:
        print(f"ERROR: Could not open port {PORT}: {e}")
        print("Try: ls /dev/ttyUSB* to find the correct port")
        exit()

    # Run health checks
    print("\nRunning checks...\n")
    results = []
    for servo_id in SERVO_IDS:
        print(f"  Checking {SERVO_LABELS[servo_id]}...", end=" ", flush=True)
        result = check_servo(servo_id)
        results.append(result)
        print("done")
        time.sleep(0.1)

    # Print full report
    all_ok = print_report(results)

    return all_ok


if __name__ == '__main__':
    main()
