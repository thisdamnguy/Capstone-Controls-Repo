import RPi.GPIO as GPIO
import time

# Pin Definitions
PUL = 17
DIR = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

print("=== Stepper Motor Diagnostic Test ===")
print("Watch for red LED flashing on driver during pulse test\n")

try:
    # Test 1: Direction pin
    print("Test 1: Setting DIR HIGH for 2 seconds...")
    GPIO.output(DIR, GPIO.HIGH)
    time.sleep(2)
    print("Setting DIR LOW for 2 seconds...")
    GPIO.output(DIR, GPIO.LOW)
    time.sleep(2)
    print("DIR test complete.\n")
    
    # Test 2: Slow pulses (visible)
    print("Test 2: Sending 10 SLOW pulses (1 Hz)...")
    print("You should see red LED flash once per second")
    GPIO.output(DIR, GPIO.HIGH)
    
    for i in range(10):
        print(f"Pulse {i+1}/10")
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.5)
    
    print("Slow pulse test complete.\n")
    
    # Test 3: Normal speed
    print("Test 3: Sending 200 pulses at normal speed...")
    for i in range(200):
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.001)  # 1ms pulse width
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.001)
    
    print("Normal speed test complete.\n")
    
    # Test 4: Very slow continuous rotation
    print("Test 4: Continuous slow rotation (Ctrl+C to stop)...")
    print("Motor should step once per second")
    
    while True:
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(1.0)  # 1 second between steps

except KeyboardInterrupt:
    print("\n\nTest stopped by user.")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")