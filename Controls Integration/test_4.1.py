#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# =========================
# SHIFT REGISTER PINS
# =========================
DATA_PIN = 17    # Serial data
CLOCK_PIN = 27   # Shift clock
LATCH_PIN = 22   # Latch

# BIT MAPPING (8-bit output)
# Bit 0 = STEP
# Bit 1 = DIR
# Bits 2-7 = unused (set to 0)

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

# Current shift register state
sr_state = 0x00

def shift_out(byte_val):
    """Shift 8 bits out to register"""
    GPIO.output(LATCH_PIN, GPIO.LOW)
    
    for i in range(7, -1, -1):
        bit = (byte_val >> i) & 1
        GPIO.output(DATA_PIN, bit)
        GPIO.output(CLOCK_PIN, GPIO.HIGH)
        GPIO.output(CLOCK_PIN, GPIO.LOW)
    
    GPIO.output(LATCH_PIN, GPIO.HIGH)
    GPIO.output(LATCH_PIN, GPIO.LOW)

def set_dir(direction):
    """Set direction bit (bit 1)"""
    global sr_state
    if direction:
        sr_state |= 0b00000010
    else:
        sr_state &= 0b11111101
    shift_out(sr_state)

def step_high():
    """Set step bit HIGH (bit 0)"""
    global sr_state
    sr_state |= 0b00000001
    shift_out(sr_state)

def step_low():
    """Set step bit LOW (bit 0)"""
    global sr_state
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

    # Setup shift register pins
    GPIO.setup(DATA_PIN, GPIO.OUT)
    GPIO.setup(CLOCK_PIN, GPIO.OUT)
    GPIO.setup(LATCH_PIN, GPIO.OUT)
    
    GPIO.output(DATA_PIN, GPIO.LOW)
    GPIO.output(CLOCK_PIN, GPIO.LOW)
    GPIO.output(LATCH_PIN, GPIO.LOW)

    # Initialize - all LOW
    shift_out(0x00)
    
    # Set direction
    set_dir(DIR_CCW)

    current_out_rpm = 5.0
    delay = out_rpm_to_delay(current_out_rpm)

    print("\nShift register stepper control (CTRL+C to stop)")
    print(f"DATA: GPIO{DATA_PIN} | CLOCK: GPIO{CLOCK_PIN} | LATCH: GPIO{LATCH_PIN}")
    print(f"PPR: {PULSES_PER_REV} | Gear ratio: {GEAR_RATIO}:1")
    print(f"Current OUTPUT RPM: {current_out_rpm:.2f}  (Motor RPM: {current_out_rpm * GEAR_RATIO:.2f})")
    print("Type a new OUTPUT RPM and press Enter anytime\n")

    last_prompt = time.time()

    try:
        while True:
            # Step pulses
            step_high()
            time.sleep(delay)
            step_low()
            time.sleep(delay)

            # Speed change input
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
                        print("Invalid number.")

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        shift_out(0x00)  # All outputs LOW
        GPIO.cleanup()

if __name__ == "__main__":
    main()