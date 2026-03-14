"""
IMU Driver - MPU6050 Interface
Placeholder implementation for Raspberry Pi

The actual implementation will use smbus2 for I2C communication.
This file provides the interface and can be run in simulation mode.
"""

import time
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

# MPU6050 Register addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


@dataclass
class IMUConfig:
    """IMU configuration parameters"""
    accel_range: int = 2        # ±2g, ±4g, ±8g, or ±16g
    gyro_range: int = 250       # ±250, ±500, ±1000, or ±2000 deg/s
    dlpf_bandwidth: int = 3     # Digital low-pass filter setting (0-6)
    sample_rate_div: int = 9    # Sample rate = 1kHz / (1 + div) = 100Hz


class MPU6050:
    """
    MPU6050 IMU driver.
    
    On Raspberry Pi, uses smbus2 for I2C.
    In simulation mode, generates fake data.
    """
    
    # Scale factors based on range setting
    ACCEL_SCALES = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
    GYRO_SCALES = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
    
    def __init__(self, 
                 config: Optional[IMUConfig] = None,
                 simulation_mode: bool = True,
                 i2c_bus: int = 1):
        """
        Initialize IMU.
        
        Args:
            config: IMU configuration
            simulation_mode: If True, generate fake data instead of reading hardware
            i2c_bus: I2C bus number (typically 1 on Pi)
        """
        self.config = config or IMUConfig()
        self.simulation_mode = simulation_mode
        self.i2c_bus = i2c_bus
        
        # Scale factors
        self.accel_scale = self.ACCEL_SCALES[self.config.accel_range]
        self.gyro_scale = self.GYRO_SCALES[self.config.gyro_range]
        
        # I2C bus (set when initialized)
        self._bus = None
        
        # Calibration offsets
        self.accel_offset = np.zeros(3)
        self.gyro_offset = np.zeros(3)
        
        # Simulation state
        self._sim_theta = 0.0
        self._sim_theta_dot = 0.0
        
    def initialize(self) -> bool:
        """
        Initialize the IMU hardware.
        
        Returns:
            True if successful
        """
        if self.simulation_mode:
            print("[IMU] Running in simulation mode")
            return True
            
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.i2c_bus)
            
            # Wake up MPU6050 (clear sleep bit)
            self._bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            # Set sample rate divider
            self._bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, self.config.sample_rate_div)
            
            # Set DLPF (digital low-pass filter)
            self._bus.write_byte_data(MPU6050_ADDR, CONFIG, self.config.dlpf_bandwidth)
            
            # Set accelerometer range
            accel_config_val = {2: 0, 4: 1, 8: 2, 16: 3}[self.config.accel_range] << 3
            self._bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, accel_config_val)
            
            # Set gyroscope range
            gyro_config_val = {250: 0, 500: 1, 1000: 2, 2000: 3}[self.config.gyro_range] << 3
            self._bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, gyro_config_val)
            
            print(f"[IMU] Initialized MPU6050 on I2C bus {self.i2c_bus}")
            return True
            
        except Exception as e:
            print(f"[IMU] Failed to initialize: {e}")
            return False
    
    def calibrate(self, num_samples: int = 100, delay_ms: int = 10) -> bool:
        """
        Calibrate by measuring offsets at rest.
        
        IMU should be stationary during calibration!
        
        Args:
            num_samples: Number of samples to average
            delay_ms: Delay between samples [ms]
            
        Returns:
            True if successful
        """
        print(f"[IMU] Calibrating with {num_samples} samples...")
        
        accel_sum = np.zeros(3)
        gyro_sum = np.zeros(3)
        
        for _ in range(num_samples):
            ax, ay, az, gx, gy, gz = self._read_raw()
            accel_sum += np.array([ax, ay, az])
            gyro_sum += np.array([gx, gy, gz])
            time.sleep(delay_ms / 1000.0)
        
        # Compute averages
        accel_avg = accel_sum / num_samples
        gyro_avg = gyro_sum / num_samples
        
        # Gyro offset: should be zero at rest
        self.gyro_offset = gyro_avg
        
        # Accel offset: should be [0, 0, 1g] at rest (assuming Z is vertical)
        # We'll just store the raw average and let the estimator handle orientation
        self.accel_offset = accel_avg - np.array([0, 0, 1.0])
        
        print(f"[IMU] Calibration complete")
        print(f"  Accel offset: {self.accel_offset}")
        print(f"  Gyro offset: {self.gyro_offset}")
        
        return True
    
    def _read_raw(self) -> Tuple[float, float, float, float, float, float]:
        """Read raw (uncalibrated) sensor values"""
        if self.simulation_mode:
            return self._generate_sim_data()
            
        # Read accelerometer (6 bytes starting at ACCEL_XOUT_H)
        accel_data = self._bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 6)
        ax = self._bytes_to_int(accel_data[0], accel_data[1]) / self.accel_scale
        ay = self._bytes_to_int(accel_data[2], accel_data[3]) / self.accel_scale
        az = self._bytes_to_int(accel_data[4], accel_data[5]) / self.accel_scale
        
        # Read gyroscope (6 bytes starting at GYRO_XOUT_H)
        gyro_data = self._bus.read_i2c_block_data(MPU6050_ADDR, GYRO_XOUT_H, 6)
        gx = self._bytes_to_int(gyro_data[0], gyro_data[1]) / self.gyro_scale
        gy = self._bytes_to_int(gyro_data[2], gyro_data[3]) / self.gyro_scale
        gz = self._bytes_to_int(gyro_data[4], gyro_data[5]) / self.gyro_scale
        
        # Convert gyro to rad/s
        gx = np.radians(gx)
        gy = np.radians(gy)
        gz = np.radians(gz)
        
        return ax, ay, az, gx, gy, gz
    
    def _bytes_to_int(self, high: int, low: int) -> int:
        """Convert two bytes to signed 16-bit integer"""
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value
    
    def read(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Read calibrated sensor values.
        
        Returns:
            Tuple of (accel [g], gyro [rad/s]) as 3-element arrays
        """
        ax, ay, az, gx, gy, gz = self._read_raw()
        
        accel = np.array([ax, ay, az]) - self.accel_offset
        gyro = np.array([gx, gy, gz]) - self.gyro_offset
        
        return accel, gyro
    
    def read_angle_and_rate(self) -> Tuple[float, float]:
        """
        Read sway angle (from accel) and rate (from gyro).
        
        Convenience method for crane control.
        
        Returns:
            Tuple of (theta [rad], theta_dot [rad/s])
        """
        accel, gyro = self.read()
        
        # Angle from accelerometer (assuming Y is lateral sway, Z is vertical)
        theta = np.arctan2(accel[1], accel[2])
        
        # Rate from gyroscope (assuming X-axis rotation is sway)
        theta_dot = gyro[0]
        
        return theta, theta_dot
    
    def _generate_sim_data(self) -> Tuple[float, float, float, float, float, float]:
        """Generate simulated sensor data"""
        # Add some noise
        noise_a = 0.01  # [g]
        noise_g = 0.02  # [rad/s]
        
        # Simulate accelerometer based on internal angle state
        ax = np.random.normal(0, noise_a)
        ay = np.sin(self._sim_theta) + np.random.normal(0, noise_a)
        az = np.cos(self._sim_theta) + np.random.normal(0, noise_a)
        
        # Simulate gyroscope
        gx = self._sim_theta_dot + np.random.normal(0, noise_g)
        gy = np.random.normal(0, noise_g)
        gz = np.random.normal(0, noise_g)
        
        return ax, ay, az, gx, gy, gz
    
    def set_sim_state(self, theta: float, theta_dot: float):
        """Set simulation state (for testing)"""
        self._sim_theta = theta
        self._sim_theta_dot = theta_dot
    
    def close(self):
        """Close I2C connection"""
        if self._bus is not None:
            self._bus.close()
            self._bus = None
