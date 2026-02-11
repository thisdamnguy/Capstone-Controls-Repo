#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# =========================
# USER SETTINGS (UPDATED)
# =========================
# Using BCM numbering (GPIO numbers)
DIR_PIN = 17     # Direction
STEP_PIN = 18    # Pulse / Step

# Direction logic (flip if direction is backwards)
DIR_CW = 1
DIR_CCW = 0

# Driver + gearbox settings
PULSES_PER_REV = 800   # set to 800 if you flip DIP to 800, etc.
GEAR_RATIO = 4.0        # 4:1 gearbox -> motor revs / output rev

# Output shaft RPM limits (safe)
MIN_OUT_RPM = 0.1
MAX_OUT_RPM = 30.0
# =========================


def out_rpm_to_delay(out_rpm: float) -> float:
    """
    Convert desired OUTPUT shaft RPM into step pulse half-period delay (seconds).
    We toggle STEP HIGH then LOW with the same delay:
      pulses_per_sec = 1 / (2*delay)

    motor_rpm = out_rpm * GEAR_RATIO
    pulses_per_sec = (motor_rpm / 60) * PULSES_PER_REV
    """
    motor_rpm = out_rpm * GEAR_RATIO
    pulses_per_sec = (motor_rpm / 60.0) * PULSES_PER_REV
    if pulses_per_sec <= 0:
        return 0.01
    return 1.0 / (2.0 * pulses_per_sec)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    GPIO.setwarnings(False)

    # IMPORTANT: BCM mode because you said "pulse is 18 and direction is 17"
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)

    # Set direction (default)
    GPIO.output(DIR_PIN, DIR_CCW)
    GPIO.output(STEP_PIN, GPIO.LOW)

    current_out_rpm = 5.0
    delay = out_rpm_to_delay(current_out_rpm)

    print("\nContinuous stepper run (CTRL+C to stop)")
    print(f"STEP (PUL): GPIO{STEP_PIN} | DIR: GPIO{DIR_PIN} | MODE: BCM")
    print(f"PPR: {PULSES_PER_REV} | Gear ratio: {GEAR_RATIO}:1")
    print(f"Current OUTPUT RPM: {current_out_rpm:.2f}  (Motor RPM: {current_out_rpm * GEAR_RATIO:.2f})")
    print("Type a new OUTPUT RPM and press Enter anytime (example: 12.5)\n")

    last_prompt = time.time()

    try:
        while True:
            # Step pulses
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(delay)

            # Occasionally allow speed change
            if time.time() - last_prompt > 0.8:
                last_prompt = time.time()
                s = input(
                    f"New OUTPUT RPM [{MIN_OUT_RPM}-{MAX_OUT_RPM}] "
                    f"(Enter to keep {current_out_rpm:.2f}): "
                ).strip()

                if s:
                    try:
                        new_rpm = float(s)
                        new_rpm = clamp(new_rpm, MIN_OUT_RPM, MAX_OUT_RPM)
                        current_out_rpm = new_rpm
                        delay = out_rpm_to_delay(current_out_rpm)
                        print(f"Set OUTPUT RPM: {current_out_rpm:.2f}  (Motor RPM: {current_out_rpm * GEAR_RATIO:.2f})")
                    except ValueError:
                        print("Invalid number, keeping previous RPM.")

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        GPIO.output(STEP_PIN, GPIO.LOW)
        GPIO.cleanup()


if __name__ == "__main__":
    main()
