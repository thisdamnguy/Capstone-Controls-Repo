"""
UHplift Crane Control System Configuration
Updated to match Control Philosophy document v0

Hardware: Raspberry Pi 4, CL42T-V41 drivers, LS7366R encoders, LSM6DS3 IMU

Units: lbm, inches, seconds (FPS system with inches)
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Tuple, Optional

# ── Physical Constants ────────────────────────────────────────────────────────
G_IN_PER_S2 = 386.09  # Gravitational acceleration [in/s²]
G_C = 386.09          # Gravitational constant for unit conversion


# ── Encoder Calibration Constants ─────────────────────────────────────────────
# From Control Philosophy document section 3.2.5
# Empirically validated with encoder chain

ENCODER_CALIBRATION = {
    'trolley': {
        'counts_per_rev': 4000,      # 1000 PPR × 4 quadrature
        'gear_ratio': 5.0,           # 5:1 gearbox
        'wheel_diameter_in': 1.5,    # Drive wheel diameter [in]
        'in_per_count': 0.000152,    # Calibrated [in/count]
    },
    'bridge': {
        'counts_per_rev': 4000,
        'gear_ratio': 4.0,           # 4:1 gearbox
        'wheel_diameter_in': 2.0,
        'in_per_count': 0.000190,
    },
    'hoist': {
        'counts_per_rev': 4000,
        'gear_ratio': 10.0,          # 10:1 gearbox
        'wheel_diameter_in': 1.5,    # Drum diameter
        'in_per_count': 0.000118,
    },
}


# ── Motor Specifications ──────────────────────────────────────────────────────
# From Control Philosophy document section 2.2

@dataclass
class MotorSpec:
    """Stepper motor specifications"""
    name: str
    part_number: str
    step_angle_deg: float = 1.8       # 200 steps/rev
    rated_current_a: float = 2.0
    holding_torque_ozfin: float = 70.0
    gear_ratio: float = 1.0
    v_target_in_s: float = 4.0        # Commanded velocity [in/s]


MOTOR_SPECS = {
    'trolley': MotorSpec(
        name="trolley",
        part_number="17HS19-2004ME1K",
        rated_current_a=2.0,
        holding_torque_ozfin=79.30,
        gear_ratio=5.0,
        v_target_in_s=4.9,
    ),
    'bridge': MotorSpec(
        name="bridge",
        part_number="17HS15-1504ME1K",
        rated_current_a=1.5,
        holding_torque_ozfin=63.73,
        gear_ratio=4.0,
        v_target_in_s=8.3,
    ),
    'hoist': MotorSpec(
        name="hoist",
        part_number="17E1KBK05",
        rated_current_a=2.0,
        holding_torque_ozfin=67.97,
        gear_ratio=10.0,
        v_target_in_s=1.25,
    ),
}


# ── Axis Configuration ────────────────────────────────────────────────────────

@dataclass
class AxisConfig:
    """Configuration for a single axis (trolley or bridge)"""
    name: str
    m_t: float              # Moving mass [lbm] (trolley or trolley+bridge)
    b_x: float              # Linear damping [lbf·s/in]
    v_target: float         # Reference velocity [in/s]
    f_max: float            # Force saturation [lbf]
    
    # Drivetrain parameters
    gear_ratio: float = 5.0
    wheel_diameter: float = 1.5
    steps_per_rev: float = 200.0
    microstepping: int = 4
    
    @property
    def pulses_per_rev(self) -> int:
        return int(self.steps_per_rev * self.microstepping)
    
    @property
    def K_conv(self) -> float:
        """Force-to-pulse conversion factor [pulses/s per in/s]"""
        # This converts velocity to pulse frequency
        circumference = np.pi * self.wheel_diameter
        pulses_per_inch = self.pulses_per_rev * self.gear_ratio / circumference
        return pulses_per_inch


@dataclass
class SystemConfig:
    """Full system configuration"""
    
    # Load parameters (variable during operation)
    m_l: float = 39.09          # Load mass [lbm] (max capacity)
    L: float = 41.138           # Cable length [in] (max hoist)
    c_theta: float = 0.05       # Rotational damping [lbf·in·s]
    
    # Axis configurations
    trolley: AxisConfig = field(default_factory=lambda: AxisConfig(
        name="trolley",
        m_t=6.7,                # Trolley mass only [lbm]
        b_x=0.10,               # Trolley damping [lbf·s/in]
        v_target=4.9,           # From Control Philosophy doc
        f_max=32.18,            # Force saturation [lbf]
        gear_ratio=5.0,
        wheel_diameter=1.5,
        microstepping=4,
    ))
    
    bridge: AxisConfig = field(default_factory=lambda: AxisConfig(
        name="bridge",
        m_t=14.4,               # Trolley + bridge mass [lbm]
        b_x=0.20,               # Bridge damping [lbf·s/in]
        v_target=8.3,           # From Control Philosophy doc
        f_max=32.18,
        gear_ratio=4.0,
        wheel_diameter=2.0,
        microstepping=4,
    ))
    
    # Control constraints
    a_slip_limit: float = 0.268     # Max acceleration [g]
    sway_limit_deg: float = 2.0     # Max sway angle [deg]
    ss_error_limit: float = 1.0     # Max steady-state error [%]
    
    # Timing parameters
    control_rate_hz: float = 200.0  # Control loop rate [Hz]
    imu_filter_cutoff_hz: float = 10.0  # IMU low-pass filter cutoff [Hz]
    velocity_filter_cutoff_hz: float = 20.0
    
    # Safety limits
    theta_emergency_deg: float = 10.0
    position_limit_trolley: float = 48.0
    position_limit_bridge: float = 48.0
    position_limit_hoist_min: float = 12.0   # Min cable length [in]
    position_limit_hoist_max: float = 41.138 # Max cable length [in]
    
    @property
    def dt(self) -> float:
        """Control timestep [s]"""
        return 1.0 / self.control_rate_hz
    
    @property
    def omega_n(self) -> float:
        """Natural frequency [rad/s]"""
        return np.sqrt(G_IN_PER_S2 / self.L)
    
    @property
    def period(self) -> float:
        """Swing period [s]"""
        return 2 * np.pi / self.omega_n


# ── LQR Gain Sets ─────────────────────────────────────────────────────────────

@dataclass
class GainSet:
    """LQR gain set for a specific operating point"""
    K_vel: float            # Velocity feedback gain
    K_theta: float          # Sway angle feedback gain
    K_theta_dot: float      # Sway rate feedback gain
    K_int: float            # Integral gain
    
    # Operating point these gains were tuned for
    m_l: float              # Load mass [lbm]
    L: float                # Cable length [in]
    
    def as_array(self) -> np.ndarray:
        """Return gains as numpy array [K_vel, K_theta, K_theta_dot, K_int]"""
        return np.array([self.K_vel, self.K_theta, self.K_theta_dot, self.K_int])


# Design point gains from LQR_Results_Design_Point.pdf
TROLLEY_GAINS = {
    # Key: (m_l, L) tuple
    (39.09, 41.138): GainSet(
        K_vel=1.542656,
        K_theta=-17.632736,
        K_theta_dot=9.460403,
        K_int=-2.353382,
        m_l=39.09,
        L=41.138
    ),
    # TODO: Add remaining 47 operating points from MATLAB sweep
    # Format: (m_l, L): GainSet(...)
}

BRIDGE_GAINS = {
    # Bridge axis gains (different due to higher mass)
    (39.09, 41.138): GainSet(
        K_vel=1.5,          # Placeholder - update from MATLAB
        K_theta=-15.0,
        K_theta_dot=8.0,
        K_int=-2.0,
        m_l=39.09,
        L=41.138
    ),
}


def get_gains(axis: str, m_l: float, L: float) -> GainSet:
    """
    Get optimal gains for given operating point.
    
    If exact match not found, returns nearest available gains.
    TODO: Implement bilinear interpolation.
    
    Args:
        axis: "trolley" or "bridge"
        m_l: Load mass [lbm]
        L: Cable length [in]
        
    Returns:
        GainSet for the operating point
    """
    gains_dict = TROLLEY_GAINS if axis == "trolley" else BRIDGE_GAINS
    
    # Exact match
    key = (m_l, L)
    if key in gains_dict:
        return gains_dict[key]
    
    # Find nearest (simple nearest-neighbor)
    min_dist = float('inf')
    nearest_key = None
    for k in gains_dict.keys():
        dist = (k[0] - m_l)**2 + (k[1] - L)**2
        if dist < min_dist:
            min_dist = dist
            nearest_key = k
    
    if nearest_key is not None:
        return gains_dict[nearest_key]
    
    raise ValueError(f"No gains available for axis={axis}, m_l={m_l}, L={L}")


# ── SPI Bus Configuration ─────────────────────────────────────────────────────

SPI_CONFIG = {
    'encoder': {
        'bus': 0,
        'ce': 0,              # CE0 = GPIO8
        'mode': 0,            # CPOL=0, CPHA=0
        'speed_hz': 500_000,
    },
    'imu': {
        'bus': 0,
        'ce': 1,              # CE1 = GPIO7
        'mode': 3,            # CPOL=1, CPHA=1
        'speed_hz': 1_000_000,
    },
}


# ── GPIO Pin Assignments ──────────────────────────────────────────────────────
# From validated test scripts

GPIO_PINS = {
    'trolley': {
        'pul': 22,
        'dir': 27,
        'ena': 17,
    },
    'bridge': {
        'pul': 23,    # Different pins for multi-axis
        'dir': 24,
        'ena': 25,
    },
    'hoist': {
        'pul': 5,
        'dir': 6,
        'ena': 13,
    },
    'spi': {
        'sclk': 11,
        'mosi': 10,
        'miso': 9,
        'ce0': 8,     # Encoder
        'ce1': 7,     # IMU
    },
}


# ── Control Modes ─────────────────────────────────────────────────────────────

class ControlMode:
    """Enumeration of control modes"""
    DISABLED = 0        # Drives disabled, safe state
    MANUAL = 1          # Direct joystick control, no sway suppression
    AUTO = 2            # LQR sway control active
    HOMING = 3          # Homing/calibration sequence
    FAULT = 4           # Fault condition, drives disabled


# ── Default Configuration Instance ────────────────────────────────────────────

DEFAULT_CONFIG = SystemConfig()
