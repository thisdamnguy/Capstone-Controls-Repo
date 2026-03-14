"""
Main Control Loop for Crane System
Real-time control on Raspberry Pi

This is the main entry point for running the crane control system.
"""

import time
import signal
import sys
from typing import Optional
from dataclasses import dataclass
from enum import Enum, auto

# Local imports
from core.config import SystemConfig, GainSet, get_gains
from core.controller import LQIController
from core.reference import ReferenceGenerator, TrapezoidalProfile, ManualModeGenerator
from core.estimator import StateEstimator, SensorReadings
from drivers.imu import MPU6050, IMUConfig
from drivers.stepper import StepperDriver, StepperConfig, DualAxisStepper


class ControlMode(Enum):
    """Operating modes"""
    IDLE = auto()
    MANUAL = auto()
    AUTO = auto()
    HOMING = auto()
    ESTOP = auto()


@dataclass
class ControlLoopConfig:
    """Control loop configuration"""
    rate_hz: float = 100.0          # Control loop rate [Hz]
    
    # Safety thresholds
    theta_limit_deg: float = 10.0   # Emergency stop if exceeded
    position_limit: float = 48.0    # Travel limit [in]
    
    # Timing
    watchdog_timeout_ms: float = 50.0  # Max time between loop iterations


class CraneController:
    """
    Main crane control class.
    
    Coordinates sensors, control law, and actuators.
    """
    
    def __init__(self,
                 system_config: SystemConfig,
                 loop_config: ControlLoopConfig,
                 simulation_mode: bool = True):
        """
        Initialize crane controller.
        
        Args:
            system_config: System parameters
            loop_config: Control loop settings
            simulation_mode: Run without hardware
        """
        self.sys_config = system_config
        self.loop_config = loop_config
        self.simulation_mode = simulation_mode
        
        self.dt = 1.0 / loop_config.rate_hz
        self.mode = ControlMode.IDLE
        
        # Initialize components
        self._init_sensors()
        self._init_actuators()
        self._init_controllers()
        
        # State
        self._running = False
        self._last_loop_time = 0.0
        self._loop_count = 0
        self._overruns = 0
        
        # Reference generator
        self.ref_gen = ReferenceGenerator()
        self.manual_gen_trolley = ManualModeGenerator(system_config.trolley.v_target)
        self.manual_gen_bridge = ManualModeGenerator(system_config.bridge.v_target)
        
        # Active axis
        self.active_axis: Optional[str] = None
        
    def _init_sensors(self):
        """Initialize sensor interfaces"""
        self.imu = MPU6050(
            config=IMUConfig(
                accel_range=2,
                gyro_range=250,
                dlpf_bandwidth=3
            ),
            simulation_mode=self.simulation_mode
        )
        
        self.estimator = StateEstimator(
            dt=self.dt,
            imu_cutoff_hz=self.sys_config.imu_filter_cutoff_hz
        )
        
    def _init_actuators(self):
        """Initialize motor drivers"""
        # Default configs - update pin numbers for your setup
        trolley_stepper = StepperConfig(
            pul_pin=17, dir_pin=27, ena_pin=22,
            steps_per_rev=200, microstepping=16,
            gear_ratio=5.18, wheel_diameter=1.5
        )
        bridge_stepper = StepperConfig(
            pul_pin=23, dir_pin=24, ena_pin=25,
            steps_per_rev=200, microstepping=16,
            gear_ratio=5.18, wheel_diameter=1.5
        )
        
        self.steppers = DualAxisStepper(
            trolley_stepper, bridge_stepper,
            simulation_mode=self.simulation_mode
        )
        
    def _init_controllers(self):
        """Initialize control algorithms"""
        # Get gains for current operating point
        trolley_gains = get_gains("trolley", self.sys_config.m_l, self.sys_config.L)
        bridge_gains = get_gains("bridge", self.sys_config.m_l, self.sys_config.L)
        
        self.trolley_ctrl = LQIController(
            trolley_gains,
            f_max=self.sys_config.trolley.f_max
        )
        self.bridge_ctrl = LQIController(
            bridge_gains,
            f_max=self.sys_config.bridge.f_max
        )
        
    def startup(self) -> bool:
        """
        Initialize hardware and prepare for operation.
        
        Returns:
            True if successful
        """
        print("[CRANE] Starting up...")
        
        # Initialize IMU
        if not self.imu.initialize():
            print("[CRANE] IMU initialization failed!")
            return False
            
        # Calibrate IMU (must be stationary)
        print("[CRANE] Calibrating IMU - keep crane stationary...")
        self.imu.calibrate(num_samples=100)
        
        # Initialize steppers
        if not self.steppers.initialize():
            print("[CRANE] Stepper initialization failed!")
            return False
            
        # Enable motors
        self.steppers.enable()
        
        # Reset state
        self.estimator.reset()
        self.trolley_ctrl.reset()
        self.bridge_ctrl.reset()
        
        self.mode = ControlMode.IDLE
        print("[CRANE] Startup complete")
        return True
    
    def shutdown(self):
        """Safely shut down the system"""
        print("[CRANE] Shutting down...")
        self.steppers.stop()
        self.steppers.disable()
        self.steppers.cleanup()
        self.imu.close()
        self._running = False
        print("[CRANE] Shutdown complete")
        
    def emergency_stop(self):
        """Emergency stop - immediate halt"""
        print("[CRANE] EMERGENCY STOP!")
        self.mode = ControlMode.ESTOP
        self.steppers.stop()
        self.steppers.disable()
        
    def set_mode(self, mode: ControlMode):
        """Change operating mode"""
        if self.mode == ControlMode.ESTOP and mode != ControlMode.IDLE:
            print("[CRANE] Cannot change mode while in E-STOP. Reset to IDLE first.")
            return
            
        prev_mode = self.mode
        self.mode = mode
        
        # Reset controllers on mode change
        if mode == ControlMode.AUTO:
            self.trolley_ctrl.reset()
            self.bridge_ctrl.reset()
            
        print(f"[CRANE] Mode: {prev_mode.name} -> {mode.name}")
        
    def start_auto_move(self, axis: str, profile: TrapezoidalProfile):
        """
        Start automatic move with sway control.
        
        Args:
            axis: "trolley" or "bridge"
            profile: Velocity profile to execute
        """
        if self.mode != ControlMode.IDLE:
            print(f"[CRANE] Cannot start move in {self.mode.name} mode")
            return
            
        self.active_axis = axis
        self.ref_gen.set_profile(profile)
        self.ref_gen.start()
        
        # Reset appropriate controller
        if axis == "trolley":
            self.trolley_ctrl.reset()
        else:
            self.bridge_ctrl.reset()
            
        self.set_mode(ControlMode.AUTO)
        print(f"[CRANE] Starting {axis} move: v_target={profile.v_target} in/s")
        
    def _read_sensors(self) -> SensorReadings:
        """Read all sensors"""
        # IMU
        accel, gyro = self.imu.read()
        
        # Position from stepper (commanded position)
        if self.active_axis == "trolley":
            position = self.steppers.trolley.get_position()
        elif self.active_axis == "bridge":
            position = self.steppers.bridge.get_position()
        else:
            position = 0.0
            
        return SensorReadings(
            accel_x=accel[0], accel_y=accel[1], accel_z=accel[2],
            gyro_x=gyro[0], gyro_y=gyro[1], gyro_z=gyro[2],
            position=position,
            timestamp=time.time()
        )
        
    def _check_safety(self, theta_deg: float, position: float) -> bool:
        """
        Check safety limits.
        
        Returns:
            True if safe, False if limit exceeded
        """
        # Sway angle limit
        if abs(theta_deg) > self.loop_config.theta_limit_deg:
            print(f"[CRANE] SAFETY: Sway limit exceeded! θ={theta_deg:.1f}°")
            return False
            
        # Position limits
        if abs(position) > self.loop_config.position_limit:
            print(f"[CRANE] SAFETY: Position limit exceeded! x={position:.1f} in")
            return False
            
        return True
        
    def _control_step(self):
        """Execute one control loop iteration"""
        # Read sensors
        readings = self._read_sensors()
        
        # Update state estimate
        v, theta, theta_dot = self.estimator.update(readings)
        theta_deg = self.estimator.theta_deg
        
        # Safety check
        if not self._check_safety(theta_deg, readings.position):
            self.emergency_stop()
            return
            
        # Mode-dependent control
        if self.mode == ControlMode.AUTO:
            # Get reference
            v_ref, _ = self.ref_gen.step(self.dt)
            
            # Compute control for active axis
            if self.active_axis == "trolley":
                u, _ = self.trolley_ctrl.compute(v, theta, theta_dot, v_ref, self.dt)
                self.steppers.trolley.set_force(u, self.sys_config.trolley, self.dt)
                self.steppers.trolley.step_once(self.dt)
            elif self.active_axis == "bridge":
                u, _ = self.bridge_ctrl.compute(v, theta, theta_dot, v_ref, self.dt)
                self.steppers.bridge.set_force(u, self.sys_config.bridge, self.dt)
                self.steppers.bridge.step_once(self.dt)
                
            # Check if move complete
            if self.ref_gen.is_complete():
                print(f"[CRANE] Move complete. Final position: {readings.position:.2f} in")
                self.set_mode(ControlMode.IDLE)
                
        elif self.mode == ControlMode.MANUAL:
            # Manual mode - pendant input would go here
            # For now, just idle
            pass
            
        elif self.mode == ControlMode.IDLE:
            # Idle - no motion
            pass
            
        elif self.mode == ControlMode.ESTOP:
            # E-stop - motors disabled
            pass
            
    def run(self):
        """
        Main control loop.
        
        Runs until interrupted or shutdown.
        """
        print(f"[CRANE] Starting control loop at {self.loop_config.rate_hz} Hz")
        self._running = True
        
        # Set up signal handlers
        def signal_handler(sig, frame):
            print("\n[CRANE] Interrupt received")
            self._running = False
            
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        target_period = self.dt
        self._last_loop_time = time.perf_counter()
        
        while self._running:
            loop_start = time.perf_counter()
            
            # Execute control
            try:
                self._control_step()
            except Exception as e:
                print(f"[CRANE] Control error: {e}")
                self.emergency_stop()
                break
                
            # Timing
            self._loop_count += 1
            elapsed = time.perf_counter() - loop_start
            
            # Check for overrun
            if elapsed > target_period:
                self._overruns += 1
                if self._overruns % 100 == 0:
                    print(f"[CRANE] Warning: {self._overruns} timing overruns")
            else:
                # Sleep for remainder of period
                time.sleep(target_period - elapsed)
                
            self._last_loop_time = time.perf_counter()
            
        # Clean up
        self.shutdown()
        
        print(f"[CRANE] Ran {self._loop_count} iterations, {self._overruns} overruns")


def main():
    """Main entry point"""
    print("="*60)
    print("UHplift Crane Control System")
    print("Team 11 - Capstone 2025-2026")
    print("="*60)
    
    # Configuration
    sys_config = SystemConfig()
    loop_config = ControlLoopConfig(rate_hz=100.0)
    
    # Check if running on Pi
    try:
        import RPi.GPIO
        simulation_mode = False
        print("Running on Raspberry Pi - HARDWARE MODE")
    except ImportError:
        simulation_mode = True
        print("Running in SIMULATION MODE")
    
    # Create controller
    controller = CraneController(sys_config, loop_config, simulation_mode)
    
    # Start up
    if not controller.startup():
        print("Startup failed!")
        return 1
        
    # Example: run a trolley move
    if simulation_mode:
        print("\nStarting test move...")
        profile = TrapezoidalProfile(
            v_target=4.6,
            accel_g=0.20,
            t_start=1.0,
            cruise_duration=3.0
        )
        controller.start_auto_move("trolley", profile)
        
    # Run control loop
    controller.run()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
