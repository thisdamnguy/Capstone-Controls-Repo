import RPi.GPIO as GPIO
import time

# Pin mapping (BCM mode)
PUL = 4      # Physical Pin 7
DIR = 17     # Physical Pin 11

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

# Set direction (1 = one way, 0 = reverse)
GPIO.output(DIR, 1)

print("Motor spinning... Press CTRL+C to stop.")

try:
    while True:
        GPIO.output(PUL, 1)
        time.sleep(0.01)   # Speed control (decrease  value for higher speed)
        GPIO.output(PUL, 0)
        time.sleep(0.01)    # Speed control (decrease  value for higher speed)

except KeyboardInterrupt:
    print("\nStopping motor...")
    GPIO.cleanup()