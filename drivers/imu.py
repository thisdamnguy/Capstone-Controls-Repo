"""
LSM6DS3 IMU Driver - SPI Interface
Replaces original MPU6050 I2C driver

Hardware: NOYITO LSM6DS3 breakout board
Interface: SPI0, CE1 (GPIO 7), Mode 3

Based on Control Philosophy document section 3.3 and lsm6ds3_spi_bringup.py

Key differences from MPU6050:
    - SPI instead of I2C
    - Mode 3 (CPOL=1, CPHA=1)
    - WHO_AM_I = 0x69 (not 0x68)
    - Different register addresses and sensitivity values
    
Output contract (unchanged from original):
    read_angle_and_rate() → (theta [rad], theta_dot [rad/s])
    
This allows estimator.py to work without modification.
"""

import spidev
import time
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass


# ── LSM6DS3 Register Addresses ────────────────────────────────────────────────
REG_WHO_AM_I    = 0x0F
REG_CTRL1_XL    = 0x10   # Accelerometer control
REG_CTRL2_G     = 0x11   # Gyroscope control
REG_CTRL3_C     = 0x12   # Control register 3
REG_CTRL4_C     = 0x13   # Control register 4
REG_FIFO_CTRL5  = 0x0A   # FIFO control
REG_OUTX_L_G    = 0x22   # Gyro X low byte (start of 12-byte burst read)

# Expected WHO_AM_I response
WHO_AM_I_EXPECTED = 0x69

# SPI read/write flags
SPI_READ  = 0x80   # Set bit 7 for read
SPI_WRITE = 0x00   # Clear bit 7 for write


@dataclass
class LSM6DS3Config:
    """IMU configuration parameters"""
    # Accelerometer settings
    accel_odr: int = 5        # Output data rate: 5 = 208 Hz
    accel_fs: int = 2         # Full scale: 2 = ±4g
    
    # Gyroscope settings
    gyro_odr: int = 5         # Output data rate: 5 = 208 Hz
    gyro_fs: int = 1          # Full scale: 1 = ±500 dps
    
    @property
    def ctrl1_xl_value(self) -> int:
        """Compute CTRL1_XL register value"""
        # [ODR_XL(4)][FS_XL(2)][BW_XL(2)]
        return (self.accel_odr << 4) | (self.accel_fs << 2) | 0x00
    
    @property
    def ctrl2_g_value(self) -> int:
        """Compute CTRL2_G register value"""
        # [ODR_G(4)][FS_G(2)][FS_125(1)][0]
        return (self.gyro_odr << 4) | (self.gyro_fs << 2) | 0x00
    
    @property
    def accel_sensitivity(self) -> float:
        """Accelerometer sensitivity [g/LSB]"""
        # From datasheet table
        sens_map = {0: 0.000061, 1: 0.000122, 2: 0.000122, 3: 0.000488}
        return sens_map.get(self.accel_fs, 0.000122)  # Default ±4g
    
    @property
    def gyro_sensitivity(self) -> float:
        """Gyroscope sensitivity [dps/LSB]"""
        # From datasheet table
        sens_map = {0: 0.00875, 1: 0.0175, 2: 0.035, 3: 0.070}
        return sens_map.get(self.gyro_fs, 0.0175)  # Default ±500 dps


class LSM6DS3:
    """
    LSM6DS3 6-axis IMU driver via SPI.
    
    Provides acceleration and angular rate measurements for sway estimation.
    """
    
    def __init__(self,
                 config: Optional[LSM6DS3Config] = None,
                 simulation_mode: bool = False,
                 spi_bus: int = 1,  # SPI1 on Raspberry Pi
                 spi_ce: int = 0):    # CE0 = GPIO18
        """
        Initialize IMU driver.
        
        Args:
            config: IMU configuration
            simulation_mode: If True, generate fake data
            spi_bus: SPI bus number (0)
            spi_ce: Chip enable (1 for CE1/GPIO7)
        """
        self.config = config or LSM6DS3Config()
        self.simulation_mode = simulation_mode
        self.spi_bus = spi_bus
        self.spi_ce = spi_ce
        
        self._spi: Optional[spidev.SpiDev] = None
        self._initialized = False
        
        # Calibration offsets
        self.accel_offset = np.zeros(3)
        self.gyro_offset = np.zeros(3)
        
        # Simulation state
        self._sim_theta = 0.0
        self._sim_theta_dot = 0.0
    
    def initialize(self) -> bool:
        """
        Initialize SPI and configure LSM6DS3.
        
        Returns:
            True if WHO_AM_I = 0x69 confirmed
        """
        if self.simulation_mode:
            print("[IMU] Running in simulation mode")
            self._initialized = True
            return True
        
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(self.spi_bus, self.spi_ce)
            self._spi.mode = 0b00  # Mode 0: - SPI1 aux controller rejects Mode 3
            self._spi.max_speed_hz = 1_000_000  # 1 MHz
            
            # Step 1: Verify WHO_AM_I
            who_am_i = self._read_register(REG_WHO_AM_I)
            if who_am_i != WHO_AM_I_EXPECTED:
                print(f"[IMU] WHO_AM_I mismatch: got 0x{who_am_i:02X}, "
                      f"expected 0x{WHO_AM_I_EXPECTED:02X}")
                if who_am_i in (0x00, 0xFF):
                    print("  → Check wiring: headers may not be soldered")
                    print("  → Verify SPI CE1 (GPIO7) connection")
                return False
            
            print(f"[IMU] WHO_AM_I = 0x{who_am_i:02X} ✓")
            
            # Step 2: Configure accelerometer
            self._write_register(REG_CTRL1_XL, self.config.ctrl1_xl_value)
            time.sleep(0.001)
            
            # Step 3: Configure gyroscope
            self._write_register(REG_CTRL2_G, self.config.ctrl2_g_value)
            time.sleep(0.001)
            
            # Step 4: Configure control registers
            # CTRL3_C: BDU=1, IF_INC=1, SIM=0 (4-wire SPI)
            ctrl3_c = 0x44  # BDU + IF_INC
            self._write_register(REG_CTRL3_C, ctrl3_c)
            time.sleep(0.001)
            
            # CTRL4_C: Disable I2C interface
            ctrl4_c = 0x04  # I2C_disable
            self._write_register(REG_CTRL4_C, ctrl4_c)
            time.sleep(0.001)
            
            # Step 5: Verify configuration
            readback_xl = self._read_register(REG_CTRL1_XL)
            readback_g = self._read_register(REG_CTRL2_G)
            
            print(f"[IMU] CTRL1_XL = 0x{readback_xl:02X}, CTRL2_G = 0x{readback_g:02X}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            print(f"[IMU] Failed to initialize: {e}")
            return False
    
    def _read_register(self, reg: int) -> int:
        """Read single register."""
        if self._spi is None:
            return 0
        resp = self._spi.xfer2([reg | SPI_READ, 0x00])
        return resp[1]
    
    def _write_register(self, reg: int, value: int):
        """Write single register."""
        if self._spi is None:
            return
        self._spi.xfer2([reg | SPI_WRITE, value])
    
    def _read_burst(self, start_reg: int, num_bytes: int) -> list:
        """
        Burst read multiple consecutive registers.
        
        IF_INC must be enabled for this to work.
        """
        if self._spi is None:
            return [0] * num_bytes
        
        # First byte: register address with read flag
        tx = [start_reg | SPI_READ] + [0x00] * num_bytes
        rx = self._spi.xfer2(tx)
        return rx[1:]  # Skip first byte (garbage during address phase)
    
    def read_raw(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Read raw sensor values (uncalibrated).
        
        Returns:
            Tuple of (accel [g], gyro [rad/s]) as 3-element arrays
        """
        if self.simulation_mode:
            return self._generate_sim_data()
        
        if not self._initialized:
            return np.zeros(3), np.zeros(3)
        
        # Burst read 12 bytes starting at OUTX_L_G
        # Order: Gx_L, Gx_H, Gy_L, Gy_H, Gz_L, Gz_H,
        #        Ax_L, Ax_H, Ay_L, Ay_H, Az_L, Az_H
        data = self._read_burst(REG_OUTX_L_G, 12)
        
        # Parse gyroscope (first 6 bytes)
        gx_raw = self._bytes_to_int16(data[0], data[1])
        gy_raw = self._bytes_to_int16(data[2], data[3])
        gz_raw = self._bytes_to_int16(data[4], data[5])
        
        # Parse accelerometer (last 6 bytes)
        ax_raw = self._bytes_to_int16(data[6], data[7])
        ay_raw = self._bytes_to_int16(data[8], data[9])
        az_raw = self._bytes_to_int16(data[10], data[11])
        
        # Convert to physical units
        accel_sens = self.config.accel_sensitivity
        gyro_sens = self.config.gyro_sensitivity
        
        accel = np.array([ax_raw, ay_raw, az_raw]) * accel_sens  # [g]
        gyro_dps = np.array([gx_raw, gy_raw, gz_raw]) * gyro_sens  # [dps]
        gyro = np.radians(gyro_dps)  # [rad/s]
        
        return accel, gyro
    
    def _bytes_to_int16(self, low: int, high: int) -> int:
        """Convert two bytes to signed 16-bit integer (little-endian)."""
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
        accel_raw, gyro_raw = self.read_raw()
        
        accel = accel_raw - self.accel_offset
        gyro = gyro_raw - self.gyro_offset
        
        return accel, gyro
    
    def read_angle_and_rate(self) -> Tuple[float, float]:
        """
        Read sway angle (from accel) and rate (from gyro).
        
        This is the primary interface for crane control.
        Output contract matches original MPU6050 driver.
        
        Returns:
            Tuple of (theta [rad], theta_dot [rad/s])
        """
        accel, gyro = self.read()
        
        # Sway angle from accelerometer
        # θ_x ≈ atan2(a_y, a_z) - angle about X-axis (trolley sway)
        # θ_y ≈ atan2(a_x, a_z) - angle about Y-axis (bridge sway)
        # Using Y/Z for primary sway direction (adjust based on IMU mounting)
        theta = np.arctan2(accel[1], accel[2])
        
        # Sway rate from gyroscope
        # Using X-axis gyro for sway rate (adjust based on mounting)
        theta_dot = gyro[0]
        
        return theta, theta_dot
    
    def calibrate(self, num_samples: int = 100, delay_ms: int = 10) -> bool:
        """
        Calibrate by measuring offsets at rest.
        
        IMU must be stationary and level during calibration!
        
        Args:
            num_samples: Number of samples to average
            delay_ms: Delay between samples [ms]
            
        Returns:
            True if successful
        """
        print(f"[IMU] Calibrating with {num_samples} samples...")
        print("  → Keep IMU stationary!")
        
        accel_sum = np.zeros(3)
        gyro_sum = np.zeros(3)
        
        for i in range(num_samples):
            accel, gyro = self.read_raw()
            accel_sum += accel
            gyro_sum += gyro
            time.sleep(delay_ms / 1000.0)
            
            if (i + 1) % 25 == 0:
                print(f"  → {i + 1}/{num_samples}")
        
        # Compute averages
        accel_avg = accel_sum / num_samples
        gyro_avg = gyro_sum / num_samples
        
        # Gyro offset: should be zero at rest
        self.gyro_offset = gyro_avg
        
        # Accel offset: should be [0, 0, 1g] at rest (Z vertical)
        self.accel_offset = accel_avg - np.array([0.0, 0.0, 1.0])
        
        print(f"[IMU] Calibration complete")
        print(f"  Accel offset: [{self.accel_offset[0]:.4f}, "
              f"{self.accel_offset[1]:.4f}, {self.accel_offset[2]:.4f}] g")
        print(f"  Gyro offset:  [{self.gyro_offset[0]:.4f}, "
              f"{self.gyro_offset[1]:.4f}, {self.gyro_offset[2]:.4f}] rad/s")
        
        return True
    
    def _generate_sim_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """Generate simulated sensor data."""
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
        
        return np.array([ax, ay, az]), np.array([gx, gy, gz])
    
    def set_sim_state(self, theta: float, theta_dot: float):
        """Set simulation state (for testing)."""
        self._sim_theta = theta
        self._sim_theta_dot = theta_dot
    
    def close(self):
        """Close SPI connection."""
        if self._spi is not None:
            self._spi.close()
            self._spi = None
            self._initialized = False


# For backwards compatibility with original interface name
IMU = LSM6DS3
