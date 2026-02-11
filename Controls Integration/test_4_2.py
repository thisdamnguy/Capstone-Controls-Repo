#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# =========================
# PIN ASSIGNMENTS
# =========================
STEP_PIN = 18    # GPIO 18 (Pin 12) - direct pulse
DATA_PIN = 17    # GPIO 17 (Pin 11) - shift register data
CLOCK_PIN = 27   # GPIO 27 (Pin 13) - shift register clock
LATCH_PIN = 22   # GPIO 22 (Pin 15) - shift register latch

# =========================
# SETTINGS
# =========================
DIR_CW = 1
DIR_CCW = 0

TARGET_OUTPUT_RPM = 5.0  # Change this value as needed
PULSES_PER_REV = 800
GEAR_RATIO = 4.0
# =========================

def shift_out(byte_val):
    """Shift 8 bits to register"""
    GPIO.output(LATCH_PIN, 0)
    for i in range(7, -1, -1):
        GPIO.output(DATA_PIN, (byte_val >> i) & 1)
        GPIO.output(CLOCK_PIN, 1)
        GPIO.output(CLOCK_PIN, 0)
    GPIO.output(LATCH_PIN, 1)
    GPIO.output(LATCH_PIN, 0)

def set_direction(direction):
    """Set direction bit (bit 0 of shift register)"""
    if direction:
        shift_out(0b00000001)
    else:
        shift_out(0b00000000)

def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    
    # Setup pins
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(DATA_PIN, GPIO.OUT)
    GPIO.setup(CLOCK_PIN, GPIO.OUT)
    GPIO.setup(LATCH_PIN, GPIO.OUT)
    
    # Initialize
    GPIO.output(STEP_PIN, 0)
    GPIO.output(DATA_PIN, 0)
    GPIO.output(CLOCK_PIN, 0)
    GPIO.output(LATCH_PIN, 0)
    
    # Clear shift register
    shift_out(0x00)
    time.sleep(0.5)
    
    # Set direction
    print("Setting direction...")
    set_direction(DIR_CW)
    time.sleep(0.5)
    
    # Calculate delay
    motor_rpm = TARGET_OUTPUT_RPM * GEAR_RATIO
    pulses_per_sec = (motor_rpm / 60.0) * PULSES_PER_REV
    delay = 1.0 / (2.0 * pulses_per_sec)
    
    print(f"\nMotor running at {TARGET_OUTPUT_RPM} output RPM")
    print(f"Motor RPM: {motor_rpm:.1f}")
    print(f"Pulse frequency: {pulses_per_sec:.1f} Hz")
    print(f"Delay: {delay*1000:.3f} ms")
    print("\nPress CTRL+C to stop\n")
    
    step_count = 0
    start_time = time.time()
    
    try:
        while True:
            GPIO.output(STEP_PIN, 1)
            time.sleep(delay)
            GPIO.output(STEP_PIN, 0)
            time.sleep(delay)
            
            step_count += 1
            
            # Status every 500 steps
            if step_count % 500 == 0:
                elapsed = time.time() - start_time
                actual_freq = step_count / elapsed
                print(f"Steps: {step_count} | Actual freq: {actual_freq:.1f} Hz")
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
        print(f"Total steps: {step_count}")
    
    finally:
        GPIO.output(STEP_PIN, 0)
        shift_out(0x00)
        GPIO.cleanup()
        print("Cleaned up")

if __name__ == "__main__":
    main()