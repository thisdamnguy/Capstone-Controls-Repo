"""
LS7366R Quadrature Encoder Counter - SPI Driver
Based on validated ls7366r_spi_validated.py and ls7366r_encoder_motor_validated.py

Hardware chain:
    Encoder (1000 PPR) → AM26LS32ACN (RS-422 receiver) → LS7366R (SPI counter)
    → CD74HCT244E (5V→3.3V level shifter) → Raspberry Pi 4

SPI Configuration:
    - Bus 0, CE0 (GPIO 8)
    - Mode 0 (CPOL=0, CPHA=0)
    - 500 kHz (conservative, can go higher)

Validated: MDR0 readback = 0x03 confirms communication
"""

import spidev
import time
from typing import Optional, Tuple
from dataclasses import dataclass


# ── LS7366R SPI Opcodes ───────────────────────────────────────────────────────
# Instruction format: [action(2)][register(3)][000]
CMD_WR_MDR0  = 0x88   # 10 001 000 — Write Mode Register 0
CMD_WR_MDR1  = 0x90   # 10 010 000 — Write Mode Register 1
CMD_RD_MDR0  = 0x48   # 01 001 000 — Read Mode Register 0 (verification)
CMD_CLR_CNTR = 0x20   # 00 100 000 — Clear counter register
CMD_LOAD_OTR = 0xE8   # 11 101 000 — Latch CNTR → OTR (snapshot)
CMD_RD_OTR   = 0x68   # 01 101 000 — Read OTR

# MDR0 Configuration Values
MDR0_X4_FREE       = 0x03   # x4 quadrature, free-running (normal operation)
MDR0_X4_INDEX_CLR  = 0x13   # x4 quadrature, free-running, clear on index (homing)

# MDR1 Configuration Values
MDR1_4BYTE = 0x00   # 4-byte (32-bit) counter mode


@dataclass
class EncoderConfig:
    """Configuration for a single encoder axis"""
    name: str
    spi_bus: int = 0
    spi_ce: int = 0              # CE0 = GPIO8, CE1 = GPIO7
    spi_speed_hz: int = 500_000
    counts_per_rev: int = 4000   # 1000 PPR × 4 (x4 quadrature)
    gear_ratio: float = 4.0      # Motor:output shaft
    wheel_radius_in: float = 0.5 # Drive wheel/drum radius [in]
    in_per_count: float = 0.000190  # Calibrated conversion factor [in/count]


class LS7366R:
    """
    LS7366R quadrature counter driver.
    
    Provides position reading from encoder via SPI.
    Position counting happens in hardware, independent of Pi timing.
    """
    
    def __init__(self, config: EncoderConfig, simulation_mode: bool = False):
        """
        Initialize encoder driver.
        
        Args:
            config: Encoder configuration
            simulation_mode: If True, generate fake data (for testing without hardware)
        """
        self.config = config
        self.simulation_mode = simulation_mode
        self._spi: Optional[spidev.SpiDev] = None
        self._initialized = False
        
        # Simulation state
        self._sim_counts = 0
        
        # Last read position (for velocity estimation)
        self._last_counts = 0
        self._last_time = 0.0
        
    def initialize(self) -> bool:
        """
        Initialize SPI and configure LS7366R.
        
        Returns:
            True if communication verified (MDR0 readback = 0x03)
        """
        if self.simulation_mode:
            print(f"[ENCODER:{self.config.name}] Running in simulation mode")
            self._initialized = True
            return True
        
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(self.config.spi_bus, self.config.spi_ce)
            self._spi.max_speed_hz = self.config.spi_speed_hz
            self._spi.mode = 0b00  # CPOL=0, CPHA=0 required by LS7366R
            
            # Configure MDR0: x4 quadrature, free-running
            self._spi.xfer2([CMD_WR_MDR0, MDR0_X4_FREE])
            time.sleep(0.001)
            
            # Configure MDR1: 4-byte counter
            self._spi.xfer2([CMD_WR_MDR1, MDR1_4BYTE])
            time.sleep(0.001)
            
            # Clear counter
            self._spi.xfer2([CMD_CLR_CNTR])
            time.sleep(0.001)
            
            # Verify communication by reading back MDR0
            result = self._spi.xfer2([CMD_RD_MDR0, 0x00])
            readback = result[1]
            
            if readback == MDR0_X4_FREE:
                print(f"[ENCODER:{self.config.name}] LS7366R comms confirmed. "
                      f"MDR0=0x{readback:02X}. Counter zeroed.")
                self._initialized = True
                return True
            elif readback in (0x00, 0xFF):
                print(f"[ENCODER:{self.config.name}] WARNING: MDR0 readback is "
                      f"0x{readback:02X}. Check MISO divider, CS wiring, or oscillator.")
                return False
            else:
                print(f"[ENCODER:{self.config.name}] WARNING: Unexpected MDR0 "
                      f"readback 0x{readback:02X}. Expected 0x03.")
                return False
                
        except Exception as e:
            print(f"[ENCODER:{self.config.name}] Failed to initialize: {e}")
            return False
    
    def read_counts(self) -> int:
        """
        Read encoder count (signed 32-bit).
        
        The LS7366R maintains the count in hardware. This latches the
        current value to OTR and reads it, avoiding race conditions.
        
        Returns:
            Signed encoder count
        """
        if self.simulation_mode:
            return self._sim_counts
        
        if not self._initialized or self._spi is None:
            return 0
        
        # Latch counter to OTR
        self._spi.xfer2([CMD_LOAD_OTR])
        time.sleep(0.0001)  # 100µs settling
        
        # Read OTR (4 bytes, MSB first)
        raw = self._spi.xfer2([CMD_RD_OTR, 0x00, 0x00, 0x00, 0x00])
        count = (raw[1] << 24) | (raw[2] << 16) | (raw[3] << 8) | raw[4]
        
        # Convert to signed
        if count >= 0x80000000:
            count -= 0x100000000
        
        return count
    
    def read_position_inches(self) -> float:
        """
        Read position in inches using calibrated conversion factor.
        
        Returns:
            Position [in]
        """
        counts = self.read_counts()
        return counts * self.config.in_per_count
    
    def read_position_and_velocity(self) -> Tuple[float, float]:
        """
        Read position and compute velocity from finite difference.
        
        Note: Velocity accuracy depends on consistent sampling interval.
        
        Returns:
            Tuple of (position [in], velocity [in/s])
        """
        current_time = time.time()
        counts = self.read_counts()
        position = counts * self.config.in_per_count
        
        # Compute velocity
        dt = current_time - self._last_time
        if dt > 0 and self._last_time > 0:
            d_counts = counts - self._last_counts
            velocity = (d_counts * self.config.in_per_count) / dt
        else:
            velocity = 0.0
        
        # Update last values
        self._last_counts = counts
        self._last_time = current_time
        
        return position, velocity
    
    def zero(self):
        """Clear the counter register to zero."""
        if self.simulation_mode:
            self._sim_counts = 0
            return
        
        if self._spi is not None:
            self._spi.xfer2([CMD_CLR_CNTR])
            time.sleep(0.001)
            print(f"[ENCODER:{self.config.name}] Counter zeroed")
    
    def enable_index_reset(self, enable: bool = True):
        """
        Enable/disable index pulse counter reset (for homing).
        
        When enabled, counter clears on index pulse (once per rev).
        Use during homing, then disable for normal operation.
        
        Args:
            enable: True to enable index reset, False for free-running
        """
        if self.simulation_mode:
            return
        
        if self._spi is None:
            return
        
        mdr0_val = MDR0_X4_INDEX_CLR if enable else MDR0_X4_FREE
        self._spi.xfer2([CMD_WR_MDR0, mdr0_val])
        time.sleep(0.001)
        
        mode_str = "INDEX RESET" if enable else "FREE-RUNNING"
        print(f"[ENCODER:{self.config.name}] Mode: {mode_str}")
    
    def set_sim_counts(self, counts: int):
        """Set simulated count value (for testing)."""
        self._sim_counts = counts
    
    def close(self):
        """Close SPI connection."""
        if self._spi is not None:
            self._spi.close()
            self._spi = None
            self._initialized = False


# ── Pre-configured Encoders ───────────────────────────────────────────────────

# Calibrated conversion factors from Control Philosophy document
TROLLEY_ENCODER = EncoderConfig(
    name="trolley",
    spi_bus=0,
    spi_ce=0,           # CE0 = GPIO8
    counts_per_rev=4000,
    gear_ratio=5.0,
    in_per_count=0.000152,  # Trolley (X-axis)
)

BRIDGE_ENCODER = EncoderConfig(
    name="bridge",
    spi_bus=0,
    spi_ce=0,           # Same CE for now - may need separate CS
    counts_per_rev=4000,
    gear_ratio=4.0,
    in_per_count=0.000190,  # Bridge (Y-axis)
)

HOIST_ENCODER = EncoderConfig(
    name="hoist",
    spi_bus=0,
    spi_ce=0,           # Same CE for now
    counts_per_rev=4000,
    gear_ratio=10.0,
    in_per_count=0.000118,  # Hoist (Z-axis)
)


def create_encoder(axis: str, simulation_mode: bool = False) -> LS7366R:
    """
    Factory function to create encoder driver for specified axis.
    
    Args:
        axis: "trolley", "bridge", or "hoist"
        simulation_mode: If True, run without hardware
        
    Returns:
        Configured LS7366R driver
    """
    configs = {
        "trolley": TROLLEY_ENCODER,
        "bridge": BRIDGE_ENCODER,
        "hoist": HOIST_ENCODER,
    }
    
    if axis not in configs:
        raise ValueError(f"Unknown axis: {axis}. Use 'trolley', 'bridge', or 'hoist'.")
    
    return LS7366R(configs[axis], simulation_mode)
