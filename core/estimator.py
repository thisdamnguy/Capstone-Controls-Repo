"""
State Estimation Module
Processes raw sensor data into state estimates for control

State vector: [v, θ, θ̇, ∫e_v]

Sensors:
- IMU (MPU6050): Provides θ (via accel) and θ̇ (via gyro)
- Stepper position: Provides x, from which v is derived
- Encoders (optional): Alternative velocity measurement
"""

import numpy as np
from collections import deque
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class SensorReadings:
    """Container for raw sensor data"""
    # IMU data
    accel_x: float = 0.0    # [g] - along cable
    accel_y: float = 0.0    # [g] - perpendicular to cable  
    accel_z: float = 0.0    # [g] - vertical
    gyro_x: float = 0.0     # [rad/s]
    gyro_y: float = 0.0     # [rad/s]  
    gyro_z: float = 0.0     # [rad/s]
    
    # Position data (from stepper or encoder)
    position: float = 0.0   # [in]
    
    # Timestamp
    timestamp: float = 0.0  # [s]


class LowPassFilter:
    """First-order IIR low-pass filter"""
    
    def __init__(self, cutoff_hz: float, dt: float):
        """
        Initialize filter.
        
        Args:
            cutoff_hz: Cutoff frequency [Hz]
            dt: Sample time [s]
        """
        self.cutoff_hz = cutoff_hz
        self.dt = dt
        
        # Compute filter coefficient
        tau = 1.0 / (2 * np.pi * cutoff_hz)
        self.alpha = dt / (tau + dt)
        
        self._y = 0.0
        self._initialized = False
        
    def reset(self, initial_value: float = 0.0):
        """Reset filter state"""
        self._y = initial_value
        self._initialized = True
        
    def update(self, x: float) -> float:
        """
        Apply filter to new sample.
        
        Args:
            x: New input sample
            
        Returns:
            Filtered output
        """
        if not self._initialized:
            self._y = x
            self._initialized = True
        else:
            self._y = (1 - self.alpha) * self._y + self.alpha * x
        return self._y
    
    @property
    def value(self) -> float:
        """Current filtered value"""
        return self._y


class DerivativeEstimator:
    """
    Estimates derivative from position/angle measurements.
    
    Uses simple finite difference with optional filtering.
    """
    
    def __init__(self, dt: float, filter_cutoff_hz: Optional[float] = None):
        """
        Initialize derivative estimator.
        
        Args:
            dt: Sample time [s]
            filter_cutoff_hz: Optional LPF cutoff for derivative
        """
        self.dt = dt
        self._last_value = 0.0
        self._last_derivative = 0.0
        self._initialized = False
        
        if filter_cutoff_hz is not None:
            self._filter = LowPassFilter(filter_cutoff_hz, dt)
        else:
            self._filter = None
            
    def reset(self):
        """Reset estimator state"""
        self._last_value = 0.0
        self._last_derivative = 0.0
        self._initialized = False
        if self._filter is not None:
            self._filter.reset()
            
    def update(self, value: float) -> float:
        """
        Compute derivative from new sample.
        
        Args:
            value: New measurement
            
        Returns:
            Estimated derivative
        """
        if not self._initialized:
            self._last_value = value
            self._initialized = True
            return 0.0
            
        # Finite difference
        derivative = (value - self._last_value) / self.dt
        self._last_value = value
        
        # Optional filtering
        if self._filter is not None:
            derivative = self._filter.update(derivative)
            
        self._last_derivative = derivative
        return derivative


class ComplementaryFilter:
    """
    Complementary filter for angle estimation.
    
    Fuses accelerometer (low-freq accurate) with gyro (high-freq accurate).
    θ_fused = α·(θ_prev + gyro·dt) + (1-α)·θ_accel
    """
    
    def __init__(self, alpha: float = 0.98, dt: float = 0.01):
        """
        Initialize complementary filter.
        
        Args:
            alpha: Filter coefficient (0.95-0.99 typical)
            dt: Sample time [s]
        """
        self.alpha = alpha
        self.dt = dt
        self._theta = 0.0
        
    def reset(self, theta: float = 0.0):
        """Reset to initial angle"""
        self._theta = theta
        
    def update(self, gyro_rate: float, accel_angle: float) -> float:
        """
        Fuse gyro and accelerometer measurements.
        
        Args:
            gyro_rate: Angular rate from gyro [rad/s]
            accel_angle: Angle computed from accelerometer [rad]
            
        Returns:
            Fused angle estimate [rad]
        """
        # Gyro integration (high-frequency, drifts)
        theta_gyro = self._theta + gyro_rate * self.dt
        
        # Fusion
        self._theta = self.alpha * theta_gyro + (1 - self.alpha) * accel_angle
        
        return self._theta
    
    @property
    def angle(self) -> float:
        """Current angle estimate [rad]"""
        return self._theta


class StateEstimator:
    """
    Full state estimator for crane control.
    
    Estimates: [v, θ, θ̇] from sensor readings
    (Integrator state is maintained by controller, not estimator)
    """
    
    def __init__(self, 
                 dt: float,
                 imu_cutoff_hz: float = 15.0,
                 velocity_cutoff_hz: float = 20.0,
                 use_complementary: bool = True,
                 complementary_alpha: float = 0.98):
        """
        Initialize state estimator.
        
        Args:
            dt: Sample time [s]
            imu_cutoff_hz: Low-pass filter cutoff for IMU data [Hz]
            velocity_cutoff_hz: Low-pass filter cutoff for velocity [Hz]
            use_complementary: Use complementary filter for angle
            complementary_alpha: Complementary filter coefficient
        """
        self.dt = dt
        
        # Filters for IMU data
        self.theta_filter = LowPassFilter(imu_cutoff_hz, dt)
        self.theta_dot_filter = LowPassFilter(imu_cutoff_hz, dt)
        
        # Velocity estimation
        self.velocity_estimator = DerivativeEstimator(dt, velocity_cutoff_hz)
        
        # Complementary filter for angle fusion
        self.use_complementary = use_complementary
        if use_complementary:
            self.comp_filter = ComplementaryFilter(complementary_alpha, dt)
        
        # State storage
        self._position = 0.0
        self._velocity = 0.0
        self._theta = 0.0
        self._theta_dot = 0.0
        
    def reset(self):
        """Reset all estimator state"""
        self.theta_filter.reset()
        self.theta_dot_filter.reset()
        self.velocity_estimator.reset()
        if self.use_complementary:
            self.comp_filter.reset()
        self._position = 0.0
        self._velocity = 0.0
        self._theta = 0.0
        self._theta_dot = 0.0
        
    def update(self, readings: SensorReadings) -> Tuple[float, float, float]:
        """
        Process sensor readings and estimate state.
        
        Args:
            readings: Raw sensor data
            
        Returns:
            Tuple of (velocity [in/s], theta [rad], theta_dot [rad/s])
        """
        # --- Position and Velocity ---
        self._position = readings.position
        self._velocity = self.velocity_estimator.update(readings.position)
        
        # --- Sway Angle (θ) ---
        # From accelerometer: θ ≈ atan2(accel_y, accel_z) for small angles
        # Assuming IMU mounted on load, accel_y is lateral, accel_z is vertical
        theta_accel = np.arctan2(readings.accel_y, readings.accel_z)
        
        # From gyro: integrate angular rate
        # Assuming gyro_x measures sway rotation (depends on mounting)
        theta_dot_raw = readings.gyro_x
        
        if self.use_complementary:
            # Fuse accel and gyro
            self._theta = self.comp_filter.update(theta_dot_raw, theta_accel)
        else:
            # Just use filtered accelerometer angle
            self._theta = self.theta_filter.update(theta_accel)
        
        # --- Sway Rate (θ̇) ---
        self._theta_dot = self.theta_dot_filter.update(theta_dot_raw)
        
        return self._velocity, self._theta, self._theta_dot
    
    def get_state(self) -> np.ndarray:
        """Get current state estimate as array [v, θ, θ̇]"""
        return np.array([self._velocity, self._theta, self._theta_dot])
    
    @property
    def position(self) -> float:
        return self._position
    
    @property
    def velocity(self) -> float:
        return self._velocity
    
    @property
    def theta(self) -> float:
        return self._theta
    
    @property
    def theta_deg(self) -> float:
        return np.degrees(self._theta)
    
    @property
    def theta_dot(self) -> float:
        return self._theta_dot


class SimulatedSensors:
    """
    Simulated sensor readings for testing without hardware.
    
    Takes true state and adds realistic noise and delay.
    """
    
    def __init__(self,
                 position_noise_std: float = 0.001,   # [in]
                 theta_noise_std: float = 0.001,       # [rad]
                 theta_dot_noise_std: float = 0.01,    # [rad/s]
                 accel_noise_std: float = 0.01,        # [g]
                 delay_samples: int = 1):
        """
        Initialize simulated sensors.
        
        Args:
            position_noise_std: Position measurement noise std
            theta_noise_std: Angle measurement noise std  
            theta_dot_noise_std: Angular rate noise std
            accel_noise_std: Accelerometer noise std
            delay_samples: Measurement delay in samples
        """
        self.position_noise_std = position_noise_std
        self.theta_noise_std = theta_noise_std
        self.theta_dot_noise_std = theta_dot_noise_std
        self.accel_noise_std = accel_noise_std
        self.delay_samples = delay_samples
        
        # Delay buffer
        self._buffer = deque(maxlen=max(1, delay_samples))
        
    def generate_reading(self, 
                         true_position: float,
                         true_theta: float,
                         true_theta_dot: float,
                         timestamp: float) -> SensorReadings:
        """
        Generate simulated sensor readings from true state.
        
        Args:
            true_position: True position [in]
            true_theta: True sway angle [rad]
            true_theta_dot: True sway rate [rad/s]
            timestamp: Current time [s]
            
        Returns:
            Simulated sensor readings with noise
        """
        # Add noise to measurements
        position = true_position + np.random.normal(0, self.position_noise_std)
        
        # Simulate accelerometer reading (measures gravity + acceleration)
        # For sway angle estimation: accel_y ≈ sin(θ), accel_z ≈ cos(θ)
        accel_y = np.sin(true_theta) + np.random.normal(0, self.accel_noise_std)
        accel_z = np.cos(true_theta) + np.random.normal(0, self.accel_noise_std)
        
        # Gyro directly measures angular rate
        gyro_x = true_theta_dot + np.random.normal(0, self.theta_dot_noise_std)
        
        reading = SensorReadings(
            accel_x=np.random.normal(0, self.accel_noise_std),
            accel_y=accel_y,
            accel_z=accel_z,
            gyro_x=gyro_x,
            gyro_y=np.random.normal(0, self.theta_dot_noise_std),
            gyro_z=np.random.normal(0, self.theta_dot_noise_std),
            position=position,
            timestamp=timestamp
        )
        
        # Apply delay
        self._buffer.append(reading)
        if len(self._buffer) >= self.delay_samples:
            return self._buffer[0]
        else:
            return reading
