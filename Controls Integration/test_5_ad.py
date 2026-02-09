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
        
        self.pwm = None  # Create on demand
        
    def set_velocity(self, velocity_rpm):
        """
        Set motor velocity in RPM
        Args:
            velocity_rpm: speed in RPM (positive or negative for direction)
        """
        if abs(velocity_rpm) < 1:  # Stop if too slow
            if self.pwm:
                self.pwm.stop()
            return
        
        # Direction
        GPIO.output(self.dir_pin, 1 if velocity_rpm > 0 else 0)
        
        # Calculate pulse frequency
        freq = abs(velocity_rpm) * self.steps_per_rev / 60.0
        freq = min(freq, 10000)  # Cap at 10kHz for safety
        
        print(f"Commanding {velocity_rpm} RPM → {freq:.1f} Hz")
        
        # Stop old PWM and create new one (more reliable)
        if self.pwm:
            self.pwm.stop()
        
        self.pwm = GPIO.PWM(self.pul_pin, freq)
        self.pwm.start(50)
    
    def stop(self):
        if self.pwm:
            self.pwm.stop()
    
    def cleanup(self):
        if self.pwm:
            self.pwm.stop()
        GPIO.cleanup()

# Test script
if __name__ == "__main__":
    motor = MotorController(pul_pin=4, dir_pin=17, microsteps=8)
    
    try:
        print("Testing motor control with more dramatic speed differences...\n")
        
        # Very slow
        print("=== 5 RPM (very slow) ===")
        motor.set_velocity(5)
        time.sleep(4)
        
        # Medium
        print("\n=== 30 RPM (medium) ===")
        motor.set_velocity(30)
        time.sleep(4)
        
        # Fast
        print("\n=== 100 RPM (fast) ===")
        motor.set_velocity(100)
        time.sleep(4)
        
        # Reverse medium
        print("\n=== -40 RPM (reverse) ===")
        motor.set_velocity(-40)
        time.sleep(4)
        
        # Stop
        print("\n=== Stopping ===")
        motor.stop()
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        motor.cleanup()