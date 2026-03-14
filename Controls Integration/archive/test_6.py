#!/usr/bin/env python3
import time
import pigpio

class StepperRPM:
    """
    Stable RPM control for STEP/DIR drivers using pigpio hardware-timed waveforms.
    STEP pin gets a 50% duty square wave at the required frequency.
    """

    def __init__(self, step_pin: int, dir_pin: int, steps_per_rev=200, microsteps=8, max_hz=50000):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.steps_per_rev = steps_per_rev * microsteps
        self.max_hz = max_hz

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Start with: sudo systemctl start pigpiod")

        self.pi.set_mode(self.step_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)
        self.stop()

    def set_rpm(self, rpm: float):
        if abs(rpm) < 1e-3:
            self.stop()
            return

        # Direction
        self.pi.write(self.dir_pin, 1 if rpm > 0 else 0)

        # RPM -> step frequency (Hz)
        hz = abs(rpm) * self.steps_per_rev / 60.0
        if hz > self.max_hz:
            hz = self.max_hz

        # 50% duty square wave on STEP pin (hardware timed by pigpio)
        # dutycycle is 0..1_000_000
        duty = 500_000
        self.pi.hardware_PWM(self.step_pin, int(hz), duty)

        print(f"Commanding {rpm:.1f} RPM -> {hz:.1f} Hz")

    def stop(self):
        # stop STEP waveform
        self.pi.hardware_PWM(self.step_pin, 0, 0)

    def close(self):
        self.stop()
        self.pi.stop()

if __name__ == "__main__":
    motor = StepperRPM(step_pin=18, dir_pin=17, steps_per_rev=200, microsteps=8)

    try:
        print("=== 100 RPM ===")
        motor.set_rpm(100)
        time.sleep(4)

        print("=== 400 RPM ===")
        motor.set_rpm(400)
        time.sleep(4)

        print("=== 1000 RPM ===")
        motor.set_rpm(4000)
        time.sleep(4)

        print("=== Stop ===")
        motor.stop()
        time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        motor.close()
