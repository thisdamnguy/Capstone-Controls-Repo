import RPi.GPIO as GPIO
import time

class MotorController:
    """Simple velocity control for stepper motor via driver (STEP/DIR)."""

    def __init__(self, pul_pin, dir_pin, steps_per_rev=200, microsteps=8, pwm_start_hz=100):
        self.pul_pin = pul_pin
        self.dir_pin = dir_pin
        self.steps_per_rev = steps_per_rev * microsteps  # effective steps per rev at microstepping

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pul_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)

        # Create ONE PWM instance for this pin (RPi.GPIO requires this)
        self.pwm = GPIO.PWM(self.pul_pin, pwm_start_hz)
        self.pwm_running = False

    def set_velocity(self, velocity_rpm):
        """
        Set motor velocity in RPM
        Args:
            velocity_rpm: speed in RPM (positive or negative for direction)
        """
        # Stop if too slow
        if abs(velocity_rpm) < 1:
            self.stop()
            return

        # Direction
        GPIO.output(self.dir_pin, GPIO.HIGH if velocity_rpm > 0 else GPIO.LOW)

        # Calculate pulse frequency (Hz)
        freq = abs(velocity_rpm) * self.steps_per_rev / 60.0
        freq = min(freq, 10000)  # Cap at 10 kHz

        print(f"Commanding {velocity_rpm} RPM -> {freq:.1f} Hz")

        # Update PWM safely
        if not self.pwm_running:
            self.pwm.start(50)  # 50% duty cycle
            self.pwm_running = True

        self.pwm.ChangeFrequency(freq)
        self.pwm.ChangeDutyCycle(50)

    def stop(self):
        if self.pwm_running:
            # Either stop PWM entirely, or set duty to 0
            self.pwm.ChangeDutyCycle(0)
            self.pwm.stop()
            self.pwm_running = False

        # Ensure STEP is low
        GPIO.output(self.pul_pin, GPIO.LOW)

    def cleanup(self):
        try:
            self.stop()
        finally:
            GPIO.cleanup()

# Test script
if __name__ == "__main__":
    motor = MotorController(pul_pin=4, dir_pin=17, microsteps=8)

    try:
        print("Testing motor control with more dramatic speed differences...\n")

        print("=== 100 RPM (very slow) ===")
        motor.set_velocity(100)
        time.sleep(4)

        print("\n=== 400 RPM (medium) ===")
        motor.set_velocity(400)
        time.sleep(4)

        print("\n=== 1000 RPM (fast) ===")
        motor.set_velocity(1000)
        time.sleep(10)

        

        print("\n=== Stopping ===")
        motor.stop()
        time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        motor.cleanup()
