"""
Stepper Motor Driver Interface
For CL42T-V41 closed-loop stepper drivers

The CL42T drivers accept:
- PUL (pulse): Step signal
- DIR: Direction signal
- ENA: Enable signal (optional)

Control is via pulse frequency modulation.
"""

import time
import numpy as np
from typing import Optional
from dataclasses import dataclass
from threading import Thread, Event
from queue import Queue

# Try to import RPi.GPIO, fall back to mock for development
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    print("[STEPPER] RPi.GPIO not available, using simulation mode")


@dataclass
class StepperConfig:
    """Stepper motor and driver configuration"""
    # Pin assignments
    pul_pin: int                # Pulse output pin (BCM numbering)
    dir_pin: int                # Direction output pin
    ena_pin: Optional[int]      # Enable pin (optional)
    
    # Motor parameters
    steps_per_rev: int = 200    # Full steps per revolution
    microstepping: int = 16     # Microstep divisor
    
    # Drivetrain parameters
    gear_ratio: float = 5.18    # Gearbox reduction
    wheel_diameter: float = 1.5 # Drive wheel diameter [in]
    
    # Limits
    max_pulse_rate: float = 100000  # Max pulse frequency [Hz]
    max_velocity: float = 10.0      # Max velocity [in/s]
    
    @property
    def steps_per_rev_total(self) -> int:
        """Total steps per motor revolution (with microstepping)"""
        return self.steps_per_rev * self.microstepping
    
    @property
    def steps_per_inch(self) -> float:
        """Steps per inch of linear travel"""
        wheel_circumference = np.pi * self.wheel_diameter
        motor_revs_per_inch = self.gear_ratio / wheel_circumference
        return motor_revs_per_inch * self.steps_per_rev_total
    
    @property
    def K_conv(self) -> float:
        """
        Velocity-to-pulse-frequency conversion factor [steps/s per in/s]
        
        From CDR Appendix K.6:
            K_conv = steps_per_rev / (2π * r_wheel)
            
        But accounting for gear ratio and microstepping:
            K_conv = (steps_per_rev * microstepping * gear_ratio) / (π * wheel_diameter)
        
        This equals steps_per_inch, so:
            f_pulse [Hz] = v [in/s] * K_conv [steps/in]
        """
        return self.steps_per_inch


class StepperDriver:
    """
    Stepper motor driver for crane axes.
    
    Converts velocity/force commands to step pulses.
    Tracks position by counting steps.
    """
    
    def __init__(self, 
                 config: StepperConfig,
                 name: str = "stepper",
                 simulation_mode: bool = None):
        """
        Initialize stepper driver.
        
        Args:
            config: Stepper configuration
            name: Axis name for logging
            simulation_mode: Force simulation mode (auto-detect if None)
        """
        self.config = config
        self.name = name
        
        # Auto-detect simulation mode
        if simulation_mode is None:
            self.simulation_mode = not HAS_GPIO
        else:
            self.simulation_mode = simulation_mode
            
        # State tracking
        self._position_steps = 0
        self._velocity = 0.0
        self._direction = 1  # 1 = positive, -1 = negative
        self._enabled = False
        
        # Pulse generation state
        self._target_pulse_rate = 0.0
        self._running = False
        self._pulse_thread = None
        self._stop_event = Event()
        
    def initialize(self) -> bool:
        """
        Initialize GPIO pins.
        
        Returns:
            True if successful
        """
        if self.simulation_mode:
            print(f"[{self.name}] Running in simulation mode")
            self._enabled = True
            return True
            
        try:
            GPIO.setmode(GPIO.BCM)
            
            # Configure pins as outputs
            GPIO.setup(self.config.pul_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.dir_pin, GPIO.OUT, initial=GPIO.LOW)
            
            if self.config.ena_pin is not None:
                GPIO.setup(self.config.ena_pin, GPIO.OUT, initial=GPIO.HIGH)
                # Note: ENA is typically active-low
                
            print(f"[{self.name}] Initialized GPIO pins")
            return True
            
        except Exception as e:
            print(f"[{self.name}] Failed to initialize: {e}")
            return False
    
    def enable(self):
        """Enable the motor driver"""
        self._enabled = True
        if not self.simulation_mode and self.config.ena_pin is not None:
            GPIO.output(self.config.ena_pin, GPIO.LOW)  # Active low
        print(f"[{self.name}] Enabled")
    
    def disable(self):
        """Disable the motor driver (motor can freewheel)"""
        self.stop()
        self._enabled = False
        if not self.simulation_mode and self.config.ena_pin is not None:
            GPIO.output(self.config.ena_pin, GPIO.HIGH)  # Active low
        print(f"[{self.name}] Disabled")
    
    def set_velocity(self, velocity: float):
        """
        Set target velocity.
        
        Converts velocity to pulse frequency using K_conv from CDR K.6:
            f_pulse = v_commanded * K_conv
        
        Args:
            velocity: Target velocity [in/s], positive or negative
        """
        if not self._enabled:
            return
            
        # Clamp to max velocity
        velocity = np.clip(velocity, -self.config.max_velocity, self.config.max_velocity)
        self._velocity = velocity
        
        # Set direction
        if velocity >= 0:
            self._direction = 1
            if not self.simulation_mode:
                GPIO.output(self.config.dir_pin, GPIO.HIGH)
        else:
            self._direction = -1
            if not self.simulation_mode:
                GPIO.output(self.config.dir_pin, GPIO.LOW)
        
        # Convert velocity to pulse rate using K_conv
        # f_pulse [Hz] = v [in/s] * K_conv [steps/in]
        self._target_pulse_rate = abs(velocity) * self.config.K_conv
        
    def set_force(self, force: float, axis_config, dt: float):
        """
        Set motor output based on force command.
        
        Implements the force-to-pulse conversion from CDR Appendix K.6:
            a = F_saturated / m
            Δv = a * Δt
            v_commanded = v_actual + Δv
            f_pulse = v_commanded * K_conv
        
        Args:
            force: Commanded force [lbf] (F_saturated from controller)
            axis_config: Axis configuration with mass
            dt: Control timestep [s]
        """
        # Newton's 2nd law: a = F/m
        # Using g_c for unit conversion (lbf to lbm*in/s²)
        G_C = 386.09  # [lbm·in/(lbf·s²)]
        accel = (force * G_C) / axis_config.m_t  # [in/s²]
        
        # Velocity increment: Δv = a * Δt
        delta_v = accel * dt  # [in/s]
        
        # Update commanded velocity: v_commanded = v_actual + Δv
        # v_actual comes from our tracked velocity (from step counting)
        v_actual = self._velocity
        v_commanded = v_actual + delta_v
        
        # Clamp to max velocity
        v_commanded = np.clip(v_commanded, -self.config.max_velocity, self.config.max_velocity)
        
        # Set the new velocity (which converts to pulse frequency internally)
        self.set_velocity(v_commanded)
    
    def stop(self):
        """Stop motor immediately"""
        self._velocity = 0.0
        self._target_pulse_rate = 0.0
        self._stop_event.set()
        
        if self._pulse_thread is not None:
            self._pulse_thread.join(timeout=0.1)
            self._pulse_thread = None
    
    def step_once(self, dt: float):
        """
        Generate steps for one control period.
        
        For real-time control, call this at the control rate.
        
        Args:
            dt: Time period [s]
        """
        if not self._enabled or self._target_pulse_rate == 0:
            return
            
        # Number of steps to generate
        steps = int(self._target_pulse_rate * dt)
        
        if self.simulation_mode:
            # Just update position counter
            self._position_steps += steps * self._direction
        else:
            # Generate pulses
            pulse_period = 1.0 / self._target_pulse_rate if self._target_pulse_rate > 0 else 0
            half_period = pulse_period / 2
            
            for _ in range(steps):
                GPIO.output(self.config.pul_pin, GPIO.HIGH)
                time.sleep(half_period)
                GPIO.output(self.config.pul_pin, GPIO.LOW)
                time.sleep(half_period)
                self._position_steps += self._direction
    
    def get_position(self) -> float:
        """Get current position [in]"""
        return self._position_steps / self.config.steps_per_inch
    
    def get_velocity(self) -> float:
        """Get current commanded velocity [in/s]"""
        return self._velocity
    
    def get_pulse_rate(self) -> float:
        """Get current pulse rate [Hz]"""
        return self._target_pulse_rate
    
    def get_position_steps(self) -> int:
        """Get current position [steps]"""
        return self._position_steps
    
    def debug_conversion_chain(self, force: float, mass: float, dt: float) -> dict:
        """
        Show the full force-to-pulse conversion chain for debugging.
        
        Implements CDR Appendix K.6 step-by-step.
        
        Args:
            force: Input force [lbf]
            mass: Moving mass [lbm]
            dt: Timestep [s]
            
        Returns:
            Dict with intermediate values
        """
        G_C = 386.09
        
        # Step 1: F = ma → a = F/m
        accel = (force * G_C) / mass  # [in/s²]
        
        # Step 2: Δv = a * Δt
        delta_v = accel * dt  # [in/s]
        
        # Step 3: v_commanded = v_actual + Δv
        v_actual = self._velocity
        v_commanded = v_actual + delta_v  # [in/s]
        
        # Step 4: f_pulse = v_commanded * K_conv
        f_pulse = abs(v_commanded) * self.config.K_conv  # [Hz]
        
        return {
            'F_saturated [lbf]': force,
            'm [lbm]': mass,
            'dt [s]': dt,
            'a [in/s²]': accel,
            'Δv [in/s]': delta_v,
            'v_actual [in/s]': v_actual,
            'v_commanded [in/s]': v_commanded,
            'K_conv [steps/in]': self.config.K_conv,
            'f_pulse [Hz]': f_pulse
        }
    
    def reset_position(self, position: float = 0.0):
        """Reset position counter"""
        self._position_steps = int(position * self.config.steps_per_inch)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.disable()
        if not self.simulation_mode:
            GPIO.cleanup([self.config.pul_pin, self.config.dir_pin])
            if self.config.ena_pin is not None:
                GPIO.cleanup([self.config.ena_pin])


class DualAxisStepper:
    """
    Manages stepper drivers for trolley and bridge axes.
    """
    
    def __init__(self,
                 trolley_config: StepperConfig,
                 bridge_config: StepperConfig,
                 simulation_mode: bool = None):
        """
        Initialize dual-axis stepper controller.
        
        Args:
            trolley_config: Trolley axis stepper config
            bridge_config: Bridge axis stepper config
            simulation_mode: Force simulation mode
        """
        self.trolley = StepperDriver(trolley_config, "trolley", simulation_mode)
        self.bridge = StepperDriver(bridge_config, "bridge", simulation_mode)
        
    def initialize(self) -> bool:
        """Initialize both axes"""
        return self.trolley.initialize() and self.bridge.initialize()
    
    def enable(self):
        """Enable both axes"""
        self.trolley.enable()
        self.bridge.enable()
        
    def disable(self):
        """Disable both axes"""
        self.trolley.disable()
        self.bridge.disable()
        
    def stop(self):
        """Stop both axes"""
        self.trolley.stop()
        self.bridge.stop()
        
    def cleanup(self):
        """Clean up both axes"""
        self.trolley.cleanup()
        self.bridge.cleanup()


# Default configurations (update pin numbers for your wiring)
DEFAULT_TROLLEY_STEPPER = StepperConfig(
    pul_pin=17,
    dir_pin=27,
    ena_pin=22,
    steps_per_rev=200,
    microstepping=16,
    gear_ratio=5.18,
    wheel_diameter=1.5,
    max_velocity=10.0
)

DEFAULT_BRIDGE_STEPPER = StepperConfig(
    pul_pin=23,
    dir_pin=24,
    ena_pin=25,
    steps_per_rev=200,
    microstepping=16,
    gear_ratio=5.18,
    wheel_diameter=1.5,
    max_velocity=15.0
)
