import RPi.GPIO as GPIO
import time

class MotorController:
    """Simple velocity control for stepper motor via driver (STEP/DIR)."""

    def __init__(self, pul_pin, dir_pin, steps_per_rev=200, microsteps=8, gear_ratio=4.0):
        self.pul_pin = pul_pin
        self.dir_pin = dir_pin
        self.steps_per_rev = steps_per_rev * microsteps  # 1600 steps/rev
        self.gear_ratio = gear_ratio  # 4:1 gearbox
        
        # Safety limits
        self.max_motor_rpm = 700  # Slightly above your 675 RPM requirement
        self.max_pulse_freq = (self.max_motor_rpm * self.steps_per_rev) / 60.0  # ~18.7 kHz

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pul_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)

        # Create ONE PWM instance
        self.pwm = GPIO.PWM(self.pul_pin, 100)
        self.pwm_running = False

    def set_velocity_rpm(self, motor_rpm):
        """
        Set motor velocity in RPM (at motor shaft, before gearbox)
        Args:
            motor_rpm: motor speed in RPM (positive or negative for direction)
        """
        # Stop if too slow
        if abs(motor_rpm) < 1:
            self.stop()
            return

        # Direction
        GPIO.output(self.dir_pin, GPIO.HIGH if motor_rpm > 0 else GPIO.LOW)

        # Calculate pulse frequency (Hz)
        freq = abs(motor_rpm) * self.steps_per_rev / 60.0
        
        # Safety limit based on motor capability, not arbitrary cap
        freq = min(freq, self.max_pulse_freq)  # Now ~18.7 kHz instead of 10 kHz!
        
        output_rpm = abs(motor_rpm) / self.gear_ratio

        print(f"Motor: {motor_rpm} RPM → Output: {output_rpm:.1f} RPM → Freq: {freq:.0f} Hz")

        # Update PWM safely
        if not self.pwm_running:
            self.pwm.start(50)  # 50% duty cycle
            self.pwm_running = True

        self.pwm.ChangeFrequency(freq)

    def stop(self):
        if self.pwm_running:
            self.pwm.stop()
            self.pwm_running = False
        GPIO.output(self.pul_pin, GPIO.LOW)

    def cleanup(self):
        try:
            self.stop()
        finally:
            GPIO.cleanup()


# Test script to hit your target speeds
if __name__ == "__main__":
    motor = MotorController(pul_pin=4, dir_pin=17, microsteps=8, gear_ratio=4.0)

    try:
        print("Testing motor speeds up to 675 RPM...\n")

        print("=== 100 RPM motor (25 RPM output) ===")
        motor.set_velocity_rpm(100)
        time.sleep(3)

        print("\n=== 300 RPM motor (75 RPM output) ===")
        motor.set_velocity_rpm(300)
        time.sleep(3)

        print("\n=== 500 RPM motor (125 RPM output) ===")
        motor.set_velocity_rpm(500)
        time.sleep(3)

        print("\n=== 675 RPM motor (168.75 RPM output) - TARGET SPEED! ===")
        motor.set_velocity_rpm(675)
        time.sleep(5)
        
        print("\n=== Reverse at 400 RPM ===")
        motor.set_velocity_rpm(-400)
        time.sleep(3)

        print("\n=== Stopping ===")
        motor.stop()

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        motor.cleanup()