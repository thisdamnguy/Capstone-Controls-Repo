#!/usr/bin/env python3
"""
UHplift Crane Control System - Main Loop
Team 11 Capstone II

Integrates:
    - LS7366R encoder position feedback (SPI0 CE0)
    - LSM6DS3 IMU sway sensing (SPI0 CE1)
    - CL42T stepper drivers via pigpio DMA waveforms
    - USB game controller HMI
    - LQI sway suppression control

Control modes:
    DISABLED: Safe state, drives off
    MANUAL: Direct joystick control with rate limiting
    AUTO: Full LQR sway control
    HOMING: Calibration sequence
    FAULT: Error state

Usage:
    python3 main.py                           # Normal operation
    python3 main.py --sim                     # Simulation mode (no hardware)
    python3 main.py --sim --real-joystick     # Sim with real PS4 controller
    python3 main.py --sim --real-joystick --log  # ... plus CSV logging
    python3 main.py --test-sensors            # Sensor checkout only
    python3 main.py --test-motors             # Motor checkout only
"""

import sys
import time
import argparse
import signal
import csv
import os
from datetime import datetime
from typing import Optional
from dataclasses import dataclass

# Core modules
from core.config import (
    SystemConfig, ControlMode, DEFAULT_CONFIG,
    get_gains, G_IN_PER_S2
)
from core.controller import LQIController
from core.reference import ManualModeGenerator
from core.estimator import StateEstimator, SensorReadings
from core.plant import CranePlant

# Drivers
from drivers.encoder import LS7366R, TROLLEY_ENCODER, BRIDGE_ENCODER
from drivers.imu import LSM6DS3
from drivers.stepper import StepperDriver, TROLLEY_STEPPER, BRIDGE_STEPPER
from drivers.joystick import JoystickDriver


@dataclass
class SystemState:
    """Current system state"""
    mode: int = ControlMode.DISABLED
    fault_code: int = 0
    
    # Positions [in]
    x_trolley: float = 0.0
    x_bridge: float = 0.0
    x_hoist: float = 0.0
    
    # Velocities [in/s]
    v_trolley: float = 0.0
    v_bridge: float = 0.0
    v_hoist: float = 0.0
    
    # Velocity references [in/s] -- from ManualModeGenerator
    v_ref_trolley: float = 0.0
    v_ref_bridge: float = 0.0
    
    # Velocity commands [in/s] -- after force-to-velocity integrator
    v_cmd_trolley: float = 0.0
    v_cmd_bridge: float = 0.0
    
    # Sway state
    theta: float = 0.0          # [rad]
    theta_dot: float = 0.0      # [rad/s]
    
    # Control outputs [lbf]
    F_trolley: float = 0.0
    F_bridge: float = 0.0
    
    # Timing
    loop_time_ms: float = 0.0
    loop_count: int = 0


class CraneController:
    """
    Main crane control system.
    
    Coordinates all subsystems and runs the control loop.
    """
    
    def __init__(self, config: SystemConfig, simulation_mode: bool = False,
                 real_joystick: bool = False, log_csv: bool = False):
        """
        Initialize crane controller.
        
        Args:
            config: System configuration
            simulation_mode: If True, run without hardware
            real_joystick: If True, connect to real USB joystick even in sim mode
            log_csv: If True, write per-tick CSV log
        """
        self.config = config
        self.simulation_mode = simulation_mode
        self.real_joystick = real_joystick
        self.log_csv = log_csv
        self.state = SystemState()
        
        # Initialize subsystems (created but not started)
        self.encoder_trolley: Optional[LS7366R] = None
        self.encoder_bridge: Optional[LS7366R] = None
        self.imu: Optional[LSM6DS3] = None
        self.stepper_trolley: Optional[StepperDriver] = None
        self.stepper_bridge: Optional[StepperDriver] = None
        self.joystick: Optional[JoystickDriver] = None
        
        # Control components
        self.estimator = StateEstimator(
            dt=config.dt,
            imu_cutoff_hz=config.imu_filter_cutoff_hz,
            velocity_cutoff_hz=config.velocity_filter_cutoff_hz,
        )
        
        # Get gains for current operating point
        trolley_gains = get_gains("trolley", config.m_l, config.L)
        bridge_gains = get_gains("bridge", config.m_l, config.L)
        
        self.controller_trolley = LQIController(
            gains=trolley_gains,
            f_max=config.trolley.f_max,
        )
        self.controller_bridge = LQIController(
            gains=bridge_gains,
            f_max=config.bridge.f_max,
        )
        
        # Reference generators for manual mode
        self.ref_trolley = ManualModeGenerator(
            v_max=config.trolley.v_target,
            accel_max_g=0.20,
        )
        self.ref_bridge = ManualModeGenerator(
            v_max=config.bridge.v_target,
            accel_max_g=0.20,
        )
        
        # Control loop state
        self._running = False
        self._last_loop_time = 0.0
        
        # Plant model for closed-loop simulation
        # When --sim, the plant replaces encoder/IMU with simulated dynamics
        if self.simulation_mode:
            self._sim_plant = CranePlant(config, axis="trolley")
            self._sim_plant.reset()
        else:
            self._sim_plant = None
        
        # Force-to-velocity integrator state [in/s]
        self._v_cmd_trolley: float = 0.0
        self._v_cmd_bridge: float = 0.0
        
        # Button edge detection
        self._enable_last = False
        self._mode_last = False
        self._reset_last = False
        
        # CSV logger
        self._csv_file = None
        self._csv_writer = None
        self._t0: float = 0.0  # Loop start epoch for relative timestamps
    
    def initialize(self) -> bool:
        """
        Initialize all hardware subsystems.
        
        Returns:
            True if all critical systems initialized successfully
        """
        print("=" * 60)
        print("  UHplift Crane Control System")
        print("  Team 11 Capstone II")
        print("=" * 60)
        print()
        
        success = True
        
        # Initialize encoders
        print("[INIT] Encoders...")
        self.encoder_trolley = LS7366R(TROLLEY_ENCODER, self.simulation_mode)
        if not self.encoder_trolley.initialize():
            print("  WARNING: Trolley encoder failed")
            success = False
        
        self.encoder_bridge = LS7366R(BRIDGE_ENCODER, self.simulation_mode)
        if not self.encoder_bridge.initialize():
            print("  WARNING: Bridge encoder failed")
            # Non-critical for single-axis testing
        
        # Initialize IMU
        print("[INIT] IMU...")
        self.imu = LSM6DS3(simulation_mode=self.simulation_mode)
        if not self.imu.initialize():
            print("  WARNING: IMU failed - sway control disabled")
            # Can still run in manual mode
        
        # Initialize steppers
        print("[INIT] Stepper drivers...")
        self.stepper_trolley = StepperDriver(TROLLEY_STEPPER, self.simulation_mode)
        if not self.stepper_trolley.initialize():
            print("  ERROR: Trolley stepper failed")
            success = False
        
        self.stepper_bridge = StepperDriver(BRIDGE_STEPPER, self.simulation_mode)
        if not self.stepper_bridge.initialize():
            print("  WARNING: Bridge stepper failed")
        
        # Initialize joystick
        # With --real-joystick, connect to real USB even when rest is simulated
        print("[INIT] Joystick HMI...")
        joy_sim = self.simulation_mode and not self.real_joystick
        self.joystick = JoystickDriver(simulation_mode=joy_sim)
        if not self.joystick.initialize():
            print("  WARNING: No joystick found")
            if not self.simulation_mode:
                print("  -> Connect USB gamepad for manual control")
        
        print()
        if success:
            print("[INIT] System ready")
            self.state.mode = ControlMode.DISABLED
        else:
            print("[INIT] Critical errors - check hardware")
            self.state.mode = ControlMode.FAULT
            self.state.fault_code = 1
        
        return success
    
    def calibrate_imu(self):
        """Run IMU calibration routine."""
        if self.imu is not None:
            print("\n[CALIBRATE] IMU - keep system stationary...")
            self.imu.calibrate(num_samples=100, delay_ms=10)
    
    def zero_encoders(self):
        """Zero all encoder positions."""
        print("[ZERO] Zeroing encoders...")
        if self.encoder_trolley is not None:
            self.encoder_trolley.zero()
        if self.encoder_bridge is not None:
            self.encoder_bridge.zero()
        if self.stepper_trolley is not None:
            self.stepper_trolley.zero_position()
        if self.stepper_bridge is not None:
            self.stepper_bridge.zero_position()
        print("[ZERO] Complete")
    
    def enable_drives(self):
        """Enable motor drivers."""
        print("[ENABLE] Enabling drives...")
        if self.stepper_trolley is not None:
            self.stepper_trolley.enable()
        if self.stepper_bridge is not None:
            self.stepper_bridge.enable()
        # Reset sim plant to zero state on enable
        if self._sim_plant is not None:
            self._sim_plant.reset()
        print("[ENABLE] Drives enabled")
    
    def disable_drives(self):
        """Disable motor drivers (safe state)."""
        print("[DISABLE] Disabling drives...")
        if self.stepper_trolley is not None:
            self.stepper_trolley.disable()
        if self.stepper_bridge is not None:
            self.stepper_bridge.disable()
        self.state.mode = ControlMode.DISABLED
        print("[DISABLE] Drives disabled")
    
    def read_sensors(self) -> SensorReadings:
        """
        Read all sensors and return unified readings.
        
        Position comes from encoder (not step counting).
        Sway comes from IMU.
        """
        readings = SensorReadings()
        readings.timestamp = time.time()
        
        # Read encoder position
        if self.encoder_trolley is not None:
            pos, vel = self.encoder_trolley.read_position_and_velocity()
            self.state.x_trolley = pos
            self.state.v_trolley = vel
            readings.position = pos
        
        if self.encoder_bridge is not None:
            pos, vel = self.encoder_bridge.read_position_and_velocity()
            self.state.x_bridge = pos
            self.state.v_bridge = vel
        
        # Read IMU
        if self.imu is not None:
            accel, gyro = self.imu.read()
            readings.accel_x = accel[0]
            readings.accel_y = accel[1]
            readings.accel_z = accel[2]
            readings.gyro_x = gyro[0]
            readings.gyro_y = gyro[1]
            readings.gyro_z = gyro[2]
        
        return readings
    
    def process_hmi(self):
        """
        Process joystick inputs and handle mode transitions.
        """
        if self.joystick is None:
            return
        
        self.joystick.update()
        
        # Soft stop (held state, no edge detection)
        if self.joystick.get_estop_pressed():
            if self.state.mode != ControlMode.DISABLED:
                print("[HMI] SOFT STOP pressed")
                self.disable_drives()
                self.state.mode = ControlMode.DISABLED
            return
        
        # Enable button (edge triggered)
        enable_now = self.joystick.get_enable_pressed()
        if enable_now and not self._enable_last:
            if self.state.mode == ControlMode.DISABLED:
                self.enable_drives()
                self.state.mode = ControlMode.MANUAL
                print("[HMI] Switched to MANUAL mode")
            elif self.state.mode in (ControlMode.MANUAL, ControlMode.AUTO):
                self.disable_drives()
                print("[HMI] Drives disabled")
        self._enable_last = enable_now
        
        # Mode toggle (edge triggered)
        mode_now = self.joystick.get_mode_pressed()
        if mode_now and not self._mode_last:
            if self.state.mode == ControlMode.MANUAL:
                self.state.mode = ControlMode.AUTO
                self.controller_trolley.reset()
                self.controller_bridge.reset()
                # Seed v_cmd from current measured velocity for bumpless transfer
                self._v_cmd_trolley = self.state.v_trolley
                self._v_cmd_bridge = self.state.v_bridge
                print("[HMI] Switched to AUTO mode (LQR active)")
            elif self.state.mode == ControlMode.AUTO:
                self.state.mode = ControlMode.MANUAL
                print("[HMI] Switched to MANUAL mode")
        self._mode_last = mode_now
        
        # Reset button (edge triggered)
        reset_now = self.joystick.get_reset_pressed()
        if reset_now and not self._reset_last:
            if self.state.mode == ControlMode.FAULT:
                self.state.mode = ControlMode.DISABLED
                self.state.fault_code = 0
                print("[HMI] Fault cleared")
            else:
                self.zero_encoders()
        self._reset_last = reset_now
    
    def control_step(self, readings: SensorReadings):
        """
        Execute one control loop iteration.
        
        In simulation mode with plant model: the LQI force drives the
        state-space model and the plant's outputs feed back as sensor
        data, closing the loop.  In real hardware mode: sensors and
        estimator provide feedback as before.
        
        Args:
            readings: Current sensor readings
        """
        dt = self.config.dt
        
        # -- State feedback source -----------------------------------------
        if self._sim_plant is not None:
            # Sim: read state directly from plant model (bypass estimator)
            v = self.state.v_trolley      # written by plant on previous tick
            theta = self.state.theta
            theta_dot = self.state.theta_dot
        else:
            # Real hardware: use estimator (complementary filter on encoder+IMU)
            v, theta, theta_dot = self.estimator.update(readings)
            self.state.theta = theta
            self.state.theta_dot = theta_dot
        
        # -- Joystick input ------------------------------------------------
        if self.joystick is not None:
            joy_trolley = self.joystick.get_trolley_input()
            joy_bridge = self.joystick.get_bridge_input()
        else:
            joy_trolley = 0.0
            joy_bridge = 0.0
        
        # -- Velocity reference (rate-limited by ManualModeGenerator) ------
        v_ref_trolley = self.ref_trolley.update(joy_trolley, dt)
        v_ref_bridge = self.ref_bridge.update(joy_bridge, dt)
        
        # Store for status display and logging
        self.state.v_ref_trolley = v_ref_trolley
        self.state.v_ref_bridge = v_ref_bridge
        
        # -- Force applied to the plant this tick --------------------------
        # (used by the sim plant propagation at the end)
        F_plant = 0.0
        
        # -- Mode-specific control -----------------------------------------
        if self.state.mode == ControlMode.MANUAL:
            # Direct velocity command, no sway control
            self.state.F_trolley = 0.0
            self.state.F_bridge = 0.0
            
            # Drive trolley at joystick-commanded velocity
            if self.stepper_trolley is not None:
                self.stepper_trolley.set_velocity_continuous(v_ref_trolley)
            
            # Track v_cmd for logging and bumpless transfer
            self._v_cmd_trolley = v_ref_trolley
            self._v_cmd_bridge = v_ref_bridge
            
            # Sim plant: stepper is a velocity source.  Apply a
            # proportional servo force so the plant's trolley tracks
            # v_ref.  This correctly excites pendulum sway from trolley
            # acceleration.
            #
            # Stability constraint (Euler):  b2 * K * dt < 1.0
            #   b2 = G_C / m_t = 386.09 / 6.7 ≈ 57.6 in/s² per lbf
            #   K = 0.5 → 57.6 * 0.5 * 0.005 = 0.14   (stable)
            #   convergence τ ≈ 1/(b2·K) ≈ 35ms ≈ 7 ticks
            if self._sim_plant is not None:
                K_SERVO = 0.5   # [lbf / (in/s)]
                F_plant = K_SERVO * (v_ref_trolley - v)
                F_plant = max(-self.config.trolley.f_max,
                              min(self.config.trolley.f_max, F_plant))
                self.state.F_trolley = F_plant
                
        elif self.state.mode == ControlMode.AUTO:
            # Full LQR control with sway suppression
            F_trolley, info = self.controller_trolley.compute(
                v=self.state.v_trolley,
                theta=theta,
                theta_dot=theta_dot,
                v_ref=v_ref_trolley,
                dt=dt,
            )
            self.state.F_trolley = F_trolley
            F_plant = F_trolley
            
            F_bridge, info = self.controller_bridge.compute(
                v=self.state.v_bridge,
                theta=theta,
                theta_dot=theta_dot,
                v_ref=v_ref_bridge,
                dt=dt,
            )
            self.state.F_bridge = F_bridge
            
            # -- Force -> velocity command (trolley) -------------------------
            m_total_trolley = self.config.trolley.m_t + self.config.m_l
            a_trolley = (F_trolley * G_IN_PER_S2) / m_total_trolley
            self._v_cmd_trolley += a_trolley * dt
            self._v_cmd_trolley = max(-self.config.trolley.v_target,
                                      min(self.config.trolley.v_target,
                                          self._v_cmd_trolley))
            
            if self.stepper_trolley is not None:
                self.stepper_trolley.set_velocity_continuous(self._v_cmd_trolley)
            
            # -- Force -> velocity command (bridge) --------------------------
            m_total_bridge = self.config.bridge.m_t + self.config.m_l
            a_bridge = (F_bridge * G_IN_PER_S2) / m_total_bridge
            self._v_cmd_bridge += a_bridge * dt
            self._v_cmd_bridge = max(-self.config.bridge.v_target,
                                     min(self.config.bridge.v_target,
                                         self._v_cmd_bridge))
            
            if self.stepper_bridge is not None:
                self.stepper_bridge.set_velocity_continuous(self._v_cmd_bridge)
        
        # -- Update state for logging --------------------------------------
        self.state.v_cmd_trolley = self._v_cmd_trolley
        self.state.v_cmd_bridge = self._v_cmd_bridge
        
        # -- Propagate sim plant -------------------------------------------
        if self._sim_plant is not None:
            x_full, plant_info = self._sim_plant.step(F_plant, v_ref_trolley, dt)
            self.state.x_trolley = plant_info['position']
            self.state.v_trolley = plant_info['velocity']
            self.state.theta     = plant_info['theta_rad']
            self.state.theta_dot = plant_info['theta_dot']
    
    def check_safety(self) -> bool:
        """
        Check safety limits.
        
        Returns:
            True if safe, False if fault detected
        """
        # Check sway angle
        theta_deg = abs(self.state.theta) * 180.0 / 3.14159
        if theta_deg > self.config.theta_emergency_deg:
            print(f"[SAFETY] Sway limit exceeded: {theta_deg:.1f} deg")
            self.state.fault_code = 2
            return False
        
        # Check position limits
        if abs(self.state.x_trolley) > self.config.position_limit_trolley:
            print(f"[SAFETY] Trolley position limit: {self.state.x_trolley:.2f} in")
            self.state.fault_code = 3
            return False
        
        if abs(self.state.x_bridge) > self.config.position_limit_bridge:
            print(f"[SAFETY] Bridge position limit: {self.state.x_bridge:.2f} in")
            self.state.fault_code = 4
            return False
        
        return True
    
    # -- CSV Logging ---------------------------------------------------------
    
    def _open_log(self):
        """Open timestamped CSV log file."""
        if not self.log_csv:
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.expanduser("~/crane_logs")
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f"run_{timestamp}.csv")
        
        self._csv_file = open(log_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "t_s", "mode", "x_trolley", "v_trolley", "v_ref_trolley",
            "theta_deg", "theta_dot", "F_trolley", "v_cmd_trolley",
            "loop_ms",
        ])
        
        print(f"[LOG] Writing to {log_path}")
    
    def _log_tick(self):
        """Write one row of data for the current tick."""
        if self._csv_writer is None:
            return
        
        t_rel = time.time() - self._t0
        mode_names = {
            ControlMode.DISABLED: "DISABLED",
            ControlMode.MANUAL: "MANUAL",
            ControlMode.AUTO: "AUTO",
            ControlMode.HOMING: "HOMING",
            ControlMode.FAULT: "FAULT",
        }
        
        self._csv_writer.writerow([
            f"{t_rel:.4f}",
            mode_names.get(self.state.mode, "UNKNOWN"),
            f"{self.state.x_trolley:.4f}",
            f"{self.state.v_trolley:.4f}",
            f"{self.state.v_ref_trolley:.4f}",
            f"{self.state.theta * 180.0 / 3.14159:.4f}",
            f"{self.state.theta_dot:.4f}",
            f"{self.state.F_trolley:.4f}",
            f"{self.state.v_cmd_trolley:.4f}",
            f"{self.state.loop_time_ms:.2f}",
        ])
    
    def _close_log(self):
        """Flush and close the CSV log."""
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
            print("[LOG] CSV log saved and closed")
    
    # -- Main loop -----------------------------------------------------------
    
    def run(self):
        """
        Main control loop.
        
        Runs at config.control_rate_hz until stopped.
        """
        self._running = True
        target_dt = self.config.dt
        self._t0 = time.time()
        
        # Open CSV log if requested
        self._open_log()
        
        print()
        print("[RUN] Control loop starting")
        print(f"  Rate: {self.config.control_rate_hz} Hz")
        print(f"  Mode: {self.state.mode}")
        if self.log_csv:
            print(f"  Logging: ENABLED")
        print("  Press Ctrl+C to stop")
        print()
        
        try:
            while self._running:
                loop_start = time.time()
                
                # Process HMI inputs
                self.process_hmi()
                
                # Read sensors
                readings = self.read_sensors()
                
                # Check safety (only when drives are active)
                if self.state.mode in (ControlMode.MANUAL, ControlMode.AUTO):
                    if not self.check_safety():
                        self.disable_drives()
                        self.state.mode = ControlMode.FAULT
                
                # Execute control
                if self.state.mode in (ControlMode.MANUAL, ControlMode.AUTO):
                    self.control_step(readings)
                
                # Timing
                loop_end = time.time()
                self.state.loop_time_ms = (loop_end - loop_start) * 1000
                self.state.loop_count += 1
                
                # Log this tick
                self._log_tick()
                
                # Sleep to maintain rate
                elapsed = loop_end - loop_start
                sleep_time = target_dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif self.state.loop_count % 100 == 0:
                    print(f"[WARN] Loop overrun: {self.state.loop_time_ms:.1f}ms "
                          f"> {target_dt*1000:.1f}ms")
                
                # Periodic status (suppress in FAULT/DISABLED — nothing is changing)
                if self.state.loop_count % 500 == 0:
                    if self.state.mode in (ControlMode.MANUAL, ControlMode.AUTO):
                        self._print_status()
                    
        except KeyboardInterrupt:
            print("\n[RUN] Interrupted by user")
        
        finally:
            self.shutdown()
    
    def _print_status(self):
        """Print periodic status."""
        mode_names = {
            ControlMode.DISABLED: "DISABLED",
            ControlMode.MANUAL: "MANUAL",
            ControlMode.AUTO: "AUTO",
            ControlMode.HOMING: "HOMING",
            ControlMode.FAULT: "FAULT",
        }
        mode_str = mode_names.get(self.state.mode, "UNKNOWN")
        
        print(f"[STATUS] Mode:{mode_str} | "
              f"X:{self.state.x_trolley:+6.2f} | "
              f"V:{self.state.v_trolley:+5.2f} | "
              f"Vref:{self.state.v_ref_trolley:+5.2f} | "
              f"theta:{self.state.theta*180/3.14159:+5.2f}deg | "
              f"F:{self.state.F_trolley:+6.2f} | "
              f"Loop:{self.state.loop_time_ms:.1f}ms")
    
    def stop(self):
        """Signal control loop to stop."""
        self._running = False
    
    def shutdown(self):
        """Clean shutdown of all subsystems."""
        print("\n[SHUTDOWN] Cleaning up...")
        
        # Close CSV log first (before disabling drives which prints)
        self._close_log()
        
        self.disable_drives()
        
        if self.encoder_trolley is not None:
            self.encoder_trolley.close()
        if self.encoder_bridge is not None:
            self.encoder_bridge.close()
        if self.imu is not None:
            self.imu.close()
        if self.stepper_trolley is not None:
            self.stepper_trolley.close()
        if self.stepper_bridge is not None:
            self.stepper_bridge.close()
        if self.joystick is not None:
            self.joystick.close()
        
        print("[SHUTDOWN] Complete")


def test_sensors(simulation_mode: bool = False):
    """Sensor checkout mode."""
    print("=" * 60)
    print("  Sensor Checkout")
    print("=" * 60)
    
    # Test encoder
    print("\n[TEST] Encoder (trolley)...")
    encoder = LS7366R(TROLLEY_ENCODER, simulation_mode)
    if encoder.initialize():
        for i in range(5):
            counts = encoder.read_counts()
            pos = encoder.read_position_inches()
            print(f"  {i+1}: {counts:8d} counts | {pos:+8.4f} in")
            time.sleep(0.5)
    encoder.close()
    
    # Test IMU
    print("\n[TEST] IMU...")
    imu = LSM6DS3(simulation_mode=simulation_mode)
    if imu.initialize():
        for i in range(5):
            theta, theta_dot = imu.read_angle_and_rate()
            print(f"  {i+1}: theta={theta*180/3.14159:+6.2f}deg | "
                  f"theta_dot={theta_dot:+6.3f} rad/s")
            time.sleep(0.5)
    imu.close()
    
    # Test joystick
    print("\n[TEST] Joystick...")
    joy = JoystickDriver(simulation_mode=simulation_mode)
    if joy.initialize():
        print("  Move sticks for 5 seconds...")
        for i in range(50):
            joy.update()
            inputs = joy.get_all_inputs()
            if i % 10 == 0:
                print(f"  Trolley:{inputs['trolley']:+5.2f} | "
                      f"Bridge:{inputs['bridge']:+5.2f} | "
                      f"Hoist:{inputs['hoist']:+5.2f}")
            time.sleep(0.1)
    joy.close()
    
    print("\n[TEST] Complete")


def test_motors(simulation_mode: bool = False):
    """Motor checkout mode."""
    print("=" * 60)
    print("  Motor Checkout")
    print("=" * 60)
    
    stepper = StepperDriver(TROLLEY_STEPPER, simulation_mode)
    if not stepper.initialize():
        print("[ERROR] Stepper initialization failed")
        return
    
    try:
        input("\nPress Enter to enable driver...")
        stepper.enable()
        
        input("Press Enter to run forward 1 rev...")
        stepper.set_direction(True)
        stepper._send_pulses_ramped_blocking(
            total_pulses=TROLLEY_STEPPER.pulses_per_output_rev,
            target_hz=2000,
        )
        
        time.sleep(1.0)
        
        input("Press Enter to run reverse 1 rev...")
        stepper.set_direction(False)
        stepper._send_pulses_ramped_blocking(
            total_pulses=TROLLEY_STEPPER.pulses_per_output_rev,
            target_hz=2000,
        )
        
        print("[TEST] Complete")
        
    except KeyboardInterrupt:
        print("\n[TEST] Interrupted")
    
    finally:
        stepper.close()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="UHplift Crane Control System")
    parser.add_argument("--sim", action="store_true",
                        help="Run in simulation mode (no hardware)")
    parser.add_argument("--real-joystick", action="store_true",
                        help="Use real USB joystick even in --sim mode")
    parser.add_argument("--log", action="store_true",
                        help="Write per-tick CSV log to ~/crane_logs/")
    parser.add_argument("--test-sensors", action="store_true",
                        help="Sensor checkout")
    parser.add_argument("--test-motors", action="store_true",
                        help="Motor checkout")
    args = parser.parse_args()
    
    if args.test_sensors:
        test_sensors(args.sim)
        return
    
    if args.test_motors:
        test_motors(args.sim)
        return
    
    # Normal operation
    config = DEFAULT_CONFIG
    controller = CraneController(
        config,
        simulation_mode=args.sim,
        real_joystick=args.real_joystick,
        log_csv=args.log,
    )
    
    # Handle SIGINT gracefully
    def signal_handler(sig, frame):
        print("\n[SIGNAL] Shutdown requested")
        controller.stop()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize and run
    if controller.initialize():
        controller.calibrate_imu()
        controller.run()
    else:
        print("[ERROR] Initialization failed")
        sys.exit(1)


if __name__ == "__main__":
    main()