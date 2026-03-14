#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# =========================
# PIN ASSIGNMENTS
# =========================
# Direct pulse (fast)
STEP_PIN = 18    # GPIO 18 (Pin 12) - direct to driver PUL

# Shift register control
DATA_PIN = 17    # GPIO 17 (Pin 11) - serial data
CLOCK_PIN = 27   # GPIO 27 (Pin 13) - shift clock
LATCH_PIN = 22   # GPIO 22 (Pin 15) - latch

# BIT MAPPING (shift register outputs)
# Bit 0 = DIR (direction)
# Bit 1 = ENABLE (if used)
# Bits 2-7 = future motors

# =========================
# USER SETTINGS
# =========================
DIR_CW = 1
DIR_CCW = 0

PULSES_PER_REV = 800
GEAR_RATIO = 4.0

MIN_OUT_RPM = 0.1
MAX_OUT_RPM = 30.0
# =========================

sr_state = 0x00

def shift_out(byte_val):
    """Shift 8 bits to register"""
    GPIO.output(LATCH_PIN, GPIO.LOW)
    
    for i in range(7, -1, -1):
        bit = (byte_val >> i) & 1
        GPIO.output(DATA_PIN, bit)
        GPIO.output(CLOCK_PIN, GPIO.HIGH)
        GPIO.output(CLOCK_PIN, GPIO.LOW)
    
    GPIO.output(LATCH_PIN, GPIO.HIGH)
    GPIO.output(LATCH_PIN, GPIO.LOW)

def set_dir(direction):
    """Set direction via shift register (bit 0)"""
    global sr_state
    if direction:
        sr_state |= 0b00000001
    else:
        sr_state &= 0b11111110
    shift_out(sr_state)

def out_rpm_to_delay(out_rpm: float) -> float:
    motor_rpm = out_rpm * GEAR_RATIO
    pulses_per_sec = (motor_rpm / 60.0) * PULSES_PER_REV
    if pulses_per_sec <= 0:
        return 0.01
    return 1.0 / (2.0 * pulses_per_sec)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # Setup direct pulse pin
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.output(STEP_PIN, GPIO.LOW)

    # Setup shift register pins
    GPIO.setup(DATA_PIN, GPIO.OUT)
    GPIO.setup(CLOCK_PIN, GPIO.OUT)
    GPIO.setup(LATCH_PIN, GPIO.OUT)
    
    GPIO.output(DATA_PIN, GPIO.LOW)
    GPIO.output(CLOCK_PIN, GPIO.LOW)
    GPIO.output(LATCH_PIN, GPIO.LOW)

    # Initialize shift register
    shift_out(0x00)
    
    # Set direction via shift register
    set_dir(DIR_CCW)

    current_out_rpm = 5.0
    delay = out_rpm_to_delay(current_out_rpm)

    print("\nHybrid control: Direct pulse + Shift register DIR")
    print(f"STEP (direct): GPIO{STEP_PIN} (Pin 12)")
    print(f"DATA: GPIO{DATA_PIN} (Pin 11) | CLOCK: GPIO{CLOCK_PIN} (Pin 13) | LATCH: GPIO{LATCH_PIN} (Pin 15)")
    print(f"PPR: {PULSES_PER_REV} | Gear ratio: {GEAR_RATIO}:1")
    print(f"Current OUTPUT RPM: {current_out_rpm:.2f}")
    print("Type new RPM and press Enter\n")

    last_prompt = time.time()

    try:
        while True:
            # Direct pulse (fast, no shift register delay)
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(delay)

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
                        print(f"Set OUTPUT RPM: {current_out_rpm:.2f}")
                    except ValueError:
                        print("Invalid number.")

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        GPIO.output(STEP_PIN, GPIO.LOW)
        shift_out(0x00)
        GPIO.cleanup()

if __name__ == "__main__":
    main()