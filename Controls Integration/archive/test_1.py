import RPi.GPIO as GPIO
import time

# Pin Definitions
PUL = 17
DIR = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

# Set Direction (High or Low)
GPIO.output(DIR, GPIO.HIGH)

print("Starting basic motor test...")
try:
    # Loop for 200 pulses (1 full rotation on most motors)
    for i in range(200):
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.005) # Pulse width
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.005) # Frequency determines speed
    
    print("Test successful: Motor should have moved.")

except KeyboardInterrupt:
    print("Test stopped by user.")

finally:
    GPIO.cleanup()