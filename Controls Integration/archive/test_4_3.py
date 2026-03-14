#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# =========================
# PIN ASSIGNMENTS
# =========================
STEP_PIN = 18    # Direct pulse
DATA_PIN = 17    # Shift register data
CLOCK_PIN = 27   # Shift register clock
LATCH_PIN = 22   # Shift register latch

# =========================
# SETTINGS
# =========================
DIR_CW = 1
DIR_CCW = 0

OUTPUT_RPM = 10.0
PULSES_PER_REV = 800
GEAR_RATIO = 4.0

STEPS_PER_DIRECTION = 400  # Steps before reversing
# =========================

def shift_out(byte_val):
    GPIO.output(LATCH_PIN, 0)
    for i in range(7, -1, -1):
        GPIO.output(DATA_PIN, (byte_val >> i) & 1)
        GPIO.output(CLOCK_PIN, 1)
        GPIO.output(CLOCK_PIN, 0)
    GPIO.output(LATCH_PIN, 1)
    GPIO.output(LATCH_PIN, 0)

def set_direction(direction):
    """Enable=1 (bit 0), DIR=direction (bit 1)"""
    if direction:
        shift_out(0b00000011)  # Enable + CW
    else:
        shift_out(0b00000001)  # Enable + CCW

def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(DATA_PIN, GPIO.OUT)
    GPIO.setup(CLOCK_PIN, GPIO.OUT)
    GPIO.setup(LATCH_PIN, GPIO.OUT)
    
    GPIO.output(STEP_PIN, 0)
    GPIO.output(DATA_PIN, 0)
    GPIO.output(CLOCK_PIN, 0)
    GPIO.output(LATCH_PIN, 0)
    
    shift_out(0x00)
    time.sleep(0.5)
    
    # Calculate delay
    motor_rpm = OUTPUT_RPM * GEAR_RATIO
    pulses_per_sec = (motor_rpm / 60.0) * PULSES_PER_REV
    delay = 1.0 / (2.0 * pulses_per_sec)
    
    print(f"\nDirection test - alternating every {STEPS_PER_DIRECTION} steps")
    print(f"Output RPM: {OUTPUT_RPM}")
    print("Press CTRL+C to stop\n")
    
    current_dir = DIR_CW
    step_count = 0
    
    try:
        while True:
            # Change direction every N steps
            if step_count % STEPS_PER_DIRECTION == 0:
                current_dir = DIR_CW if current_dir == DIR_CCW else DIR_CCW
                dir_name = "CW (forward)" if current_dir == DIR_CW else "CCW (reverse)"
                print(f"\n>>> Direction: {dir_name}")
                set_direction(current_dir)
                time.sleep(0.2)  # Brief pause on direction change
            
            # Step
            GPIO.output(STEP_PIN, 1)
            time.sleep(delay)
            GPIO.output(STEP_PIN, 0)
            time.sleep(delay)
            
            step_count += 1
            
            # Status
            if step_count % 100 == 0:
                progress = step_count % STEPS_PER_DIRECTION
                print(f"Steps in this direction: {progress}/{STEPS_PER_DIRECTION}", end='\r')
    
    except KeyboardInterrupt:
        print(f"\n\nStopped at {step_count} total steps")
    
    finally:
        GPIO.output(STEP_PIN, 0)
        shift_out(0x00)
        GPIO.cleanup()
        print("Cleaned up")

if __name__ == "__main__":
    main()