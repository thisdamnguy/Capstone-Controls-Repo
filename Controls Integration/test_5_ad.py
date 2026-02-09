import RPi.GPIO as GPIO
import time

class MotorController:
    """Simple velocity control for stepper motor via driver"""
    
    def __init__(self, pul_pin, dir_pin, steps_per_rev=200, microsteps=8):
        self.pul_pin = pul_pin
        self.dir_pin = dir_pin
        self.steps_per_rev = steps_per_rev * microsteps
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pul_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.pul_pin, 1000)
        
    def set_velocity(self, velocity_rpm):
        """
        Set motor velocity in RPM
        Args:
            velocity_rpm: speed in RPM (positive or negative for direction)
        """
        if abs(velocity_rpm) < 1:  # Stop if too slow
            self.pwm.stop()
            return
        
        # Direction
        GPIO.output(self.dir_pin, 1 if velocity_rpm > 0 else 0)
        
        # Calculate pulse frequency
        freq = abs(velocity_rpm) * self.steps_per_rev / 60.0
        freq = min(freq, 10000)  # Cap at 10kHz for safety
        
        self.pwm.ChangeFrequency(freq)
        self.pwm.start(50)
    
    def stop(self):
        self.pwm.stop()
    
    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

# Test script
if __name__ == "__main__":
    motor = MotorController(pul_pin=4, dir_pin=17, microsteps=8)
    
    try:
        print("Testing motor control...")
        
        # Ramp up
        for rpm in [10, 30, 60, 100]:
            print(f"Setting {rpm} RPM")
            motor.set_velocity(rpm)
            time.sleep(2)
        
        # Reverse
        print("Reversing...")
        motor.set_velocity(-60)
        time.sleep(2)
        
        # Stop
        print("Stopping")
        motor.stop()
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        motor.cleanup()