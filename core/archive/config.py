"""
Crane Control System Configuration
Updated with 48-point Gain Lookup and Bilinear Interpolation
Matches MATLAB parameters from LQR_Control.m and run_LQR_dual_objective.m

Units: lbm, inches, seconds (FPS system with inches)
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Tuple, List

# Physical constants
G_IN_PER_S2 = 386.09  # gravitational acceleration [in/s^2]
G_C = 386.09          # gravitational constant for unit conversion


@dataclass
class AxisConfig:
    """Configuration for a single axis (trolley or bridge)"""
    name: str
    m_t: float          # Moving mass [lbm] (trolley or trolley+bridge)
    b_x: float          # Linear damping [lbf·s/in]
    v_target: float     # Reference velocity [in/s]
    f_max: float        # Force saturation [lbf]
    
    # Drivetrain parameters (for force-to-steps conversion)
    gear_ratio: float = 5.18        # Planetary gearbox ratio
    wheel_diameter: float = 1.5     # Drive wheel diameter [in]
    steps_per_rev: float = 200.0    # Stepper motor steps/rev
    microstepping: int = 16         # Microstep divisor
    

@dataclass 
class SystemConfig:
    """Full system configuration"""
    
    # Load parameters (variable during operation)
    m_l: float = 39.09      # Load mass [lbm] (max capacity nominal)
    L: float = 41.138       # Cable length [in] (max hoist)
    c_theta: float = 0.05   # Rotational damping [lbf·in·s]
    
    # Axis configurations
    trolley: AxisConfig = field(default_factory=lambda: AxisConfig(
        name="trolley",
        m_t=6.7,            # Trolley mass only [lbm]
        b_x=0.10,           # Trolley damping [lbf·s/in]
        v_target=4.6,       # Trolley velocity [in/s]
        f_max=32.18,        # Force saturation [lbf]
    ))
    
    bridge: AxisConfig = field(default_factory=lambda: AxisConfig(
        name="bridge",
        m_t=14.4,           # Trolley + bridge mass [lbm] (6.7 + 7.7)
        b_x=0.20,           # Bridge damping (2× trolley) [lbf·s/in]
        v_target=8.3,       # Bridge velocity [in/s]
        f_max=32.18,        # Force saturation [lbf]
    ))
    
    # Control constraints
    a_slip_limit: float = 0.268     # Max acceleration [g]
    sway_limit_deg: float = 2.0     # Max sway angle [deg]
    ss_error_limit: float = 1.0     # Max steady-state error [%]
    
    # Timing parameters
    control_rate_hz: float = 100.0  # Control loop rate [Hz]
    imu_filter_cutoff_hz: float = 15.0  # IMU low-pass filter cutoff [Hz]
    
    # Safety limits
    theta_emergency_deg: float = 10.0   # Emergency stop threshold [deg]
    position_limit_trolley: float = 48.0  # Trolley travel limit [in]
    position_limit_bridge: float = 48.0   # Bridge travel limit [in]
    
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


@dataclass
class GainSet:
    """LQR gain set for a specific operating point"""
    K_vel: float        # Velocity feedback gain
    K_theta: float      # Sway angle feedback gain  
    K_theta_dot: float  # Sway rate feedback gain
    K_int: float        # Integral gain
    
    # Operating point these gains were tuned for
    m_l: float          # Load mass [lbm]
    L: float            # Cable length [in]
    
    def as_array(self) -> np.ndarray:
        """Return gains as numpy array [K_vel, K_theta, K_theta_dot, K_int]"""
        return np.array([self.K_vel, self.K_theta, self.K_theta_dot, self.K_int])


# --- GAIN LOOKUP TABLES ---
# Extracted from Controller_Analysis spreadsheets

TROLLEY_GAINS = {
    (5.0, 12.0): GainSet(K_vel=1.551546, K_theta=-68.840671, K_theta_dot=4.647204, K_int=-3.280408, m_l=5.0, L=12.0),
    (5.0, 18.0): GainSet(K_vel=0.914711, K_theta=-52.156980, K_theta_dot=4.534950, K_int=-1.688328, m_l=5.0, L=18.0),
    (5.0, 24.0): GainSet(K_vel=1.020649, K_theta=-67.788956, K_theta_dot=7.452011, K_int=-1.688328, m_l=5.0, L=24.0),
    (5.0, 30.0): GainSet(K_vel=1.148304, K_theta=-77.596405, K_theta_dot=9.587261, K_int=-1.688328, m_l=5.0, L=30.0),
    (5.0, 36.0): GainSet(K_vel=1.044449, K_theta=-77.290674, K_theta_dot=10.747009, K_int=-1.430010, m_l=5.0, L=36.0),
    (5.0, 41.138): GainSet(K_vel=0.957642, K_theta=-75.642958, K_theta_dot=11.378903, K_int=-1.211216, m_l=5.0, L=41.138),
    (10.0, 12.0): GainSet(K_vel=1.700140, K_theta=-56.326887, K_theta_dot=4.143232, K_int=-3.973281, m_l=10.0, L=12.0),
    (10.0, 18.0): GainSet(K_vel=1.282583, K_theta=-50.485125, K_theta_dot=4.897455, K_int=-2.215456, m_l=10.0, L=18.0),
    (10.0, 24.0): GainSet(K_vel=1.282583, K_theta=-58.374100, K_theta_dot=6.852462, K_int=-2.215456, m_l=10.0, L=24.0),
    (10.0, 30.0): GainSet(K_vel=1.365318, K_theta=-65.659616, K_theta_dot=8.608304, K_int=-2.110545, m_l=10.0, L=30.0),
    (10.0, 36.0): GainSet(K_vel=1.189721, K_theta=-63.633497, K_theta_dot=9.324505, K_int=-1.688328, m_l=10.0, L=36.0),
    (10.0, 41.138): GainSet(K_vel=1.082987, K_theta=-61.644485, K_theta_dot=9.728876, K_int=-1.430010, m_l=10.0, L=41.138),
    (15.0, 12.0): GainSet(K_vel=1.758362, K_theta=-48.748366, K_theta_dot=3.766861, K_int=-4.381120, m_l=15.0, L=12.0),
    (15.0, 18.0): GainSet(K_vel=1.455447, K_theta=-47.530279, K_theta_dot=4.945532, K_int=-2.628994, m_l=15.0, L=18.0),
    (15.0, 24.0): GainSet(K_vel=1.455447, K_theta=-53.513470, K_theta_dot=6.619088, K_int=-2.628994, m_l=15.0, L=24.0),
    (15.0, 30.0): GainSet(K_vel=1.516560, K_theta=-58.910398, K_theta_dot=8.106090, K_int=-2.499882, m_l=15.0, L=30.0),
    (15.0, 36.0): GainSet(K_vel=1.314647, K_theta=-56.702787, K_theta_dot=8.705852, K_int=-1.993309, m_l=15.0, L=36.0),
    (15.0, 41.138): GainSet(K_vel=1.196695, K_theta=-54.810578, K_theta_dot=9.049442, K_int=-1.688328, m_l=15.0, L=41.138),
    (20.0, 12.0): GainSet(K_vel=1.782803, K_theta=-43.510651, K_theta_dot=3.484252, K_int=-4.654203, m_l=20.0, L=12.0),
    (20.0, 18.0): GainSet(K_vel=1.551546, K_theta=-44.394627, K_theta_dot=4.851910, K_int=-2.946394, m_l=20.0, L=18.0),
    (20.0, 24.0): GainSet(K_vel=1.564491, K_theta=-49.799794, K_theta_dot=6.398642, K_int=-2.946394, m_l=20.0, L=24.0),
    (20.0, 30.0): GainSet(K_vel=1.637257, K_theta=-54.582697, K_theta_dot=7.771032, K_int=-2.812328, m_l=20.0, L=30.0),
    (20.0, 36.0): GainSet(K_vel=1.423087, K_theta=-52.418292, K_theta_dot=8.309620, K_int=-2.242398, m_l=20.0, L=36.0),
    (20.0, 41.138): GainSet(K_vel=1.295415, K_theta=-50.626786, K_theta_dot=8.625686, K_int=-1.899453, m_l=20.0, L=41.138),
    (25.0, 12.0): GainSet(K_vel=1.791550, K_theta=-39.569485, K_theta_dot=3.262529, K_int=-4.849929, m_l=25.0, L=12.0),
    (25.0, 18.0): GainSet(K_vel=1.611110, K_theta=-41.344849, K_theta_dot=4.710777, K_int=-3.197025, m_l=25.0, L=18.0),
    (25.0, 24.0): GainSet(K_vel=1.639145, K_theta=-46.611417, K_theta_dot=6.182436, K_int=-3.197025, m_l=25.0, L=24.0),
    (25.0, 30.0): GainSet(K_vel=1.737035, K_theta=-51.365310, K_theta_dot=7.508544, K_int=-3.068222, m_l=25.0, L=30.0),
    (25.0, 36.0): GainSet(K_vel=1.517399, K_theta=-49.423999, K_theta_dot=8.026402, K_int=-2.469502, m_l=25.0, L=36.0),
    (25.0, 41.138): GainSet(K_vel=1.381363, K_theta=-47.727546, K_theta_dot=8.330453, K_int=-2.091993, m_l=25.0, L=41.138),
    (30.0, 12.0): GainSet(K_vel=1.791782, K_theta=-36.467367, K_theta_dot=3.082729, K_int=-4.996160, m_l=30.0, L=12.0),
    (30.0, 18.0): GainSet(K_vel=1.649646, K_theta=-38.487541, K_theta_dot=4.551846, K_int=-3.400569, m_l=30.0, L=18.0),
    (30.0, 24.0): GainSet(K_vel=1.692348, K_theta=-43.766779, K_theta_dot=5.972302, K_int=-3.400569, m_l=30.0, L=24.0),
    (30.0, 30.0): GainSet(K_vel=1.821172, K_theta=-48.749021, K_theta_dot=7.283995, K_int=-3.280408, m_l=30.0, L=30.0),
    (30.0, 36.0): GainSet(K_vel=1.600115, K_theta=-47.126839, K_theta_dot=7.808064, K_int=-2.671509, m_l=30.0, L=36.0),
    (30.0, 41.138): GainSet(K_vel=1.456722, K_theta=-45.529259, K_theta_dot=8.109159, K_int=-2.269661, m_l=30.0, L=41.138),
    (35.0, 12.0): GainSet(K_vel=1.787687, K_theta=-33.957297, K_theta_dot=2.933333, K_int=-5.108428, m_l=35.0, L=12.0),
    (35.0, 18.0): GainSet(K_vel=1.674751, K_theta=-35.849315, K_theta_dot=4.387114, K_int=-3.569426, m_l=35.0, L=18.0),
    (35.0, 24.0): GainSet(K_vel=1.730704, K_theta=-41.189490, K_theta_dot=5.768686, K_int=-3.569426, m_l=35.0, L=24.0),
    (35.0, 30.0): GainSet(K_vel=1.892994, K_theta=-46.505419, K_theta_dot=7.081829, K_int=-3.459341, m_l=35.0, L=30.0),
    (35.0, 36.0): GainSet(K_vel=1.673238, K_theta=-45.239255, K_theta_dot=7.629399, K_int=-2.853049, m_l=35.0, L=36.0),
    (35.0, 41.138): GainSet(K_vel=1.523315, K_theta=-43.755498, K_theta_dot=7.933190, K_int=-2.434418, m_l=35.0, L=41.138),
    (39.06, 12.0): GainSet(K_vel=1.781896, K_theta=-32.227447, K_theta_dot=2.827055, K_int=-5.188737, m_l=39.06, L=12.0),
    (39.06, 18.0): GainSet(K_vel=1.690858, K_theta=-33.978255, K_theta_dot=4.261882, K_int=-3.693170, m_l=39.06, L=18.0),
    (39.06, 24.0): GainSet(K_vel=1.758254, K_theta=-39.314902, K_theta_dot=5.613317, K_int=-3.693170, m_l=39.06, L=24.0),
    (39.06, 30.0): GainSet(K_vel=1.946394, K_theta=-44.821935, K_theta_dot=6.924847, K_int=-3.593259, m_l=39.06, L=30.0),
    (39.06, 36.0): GainSet(K_vel=1.728286, K_theta=-43.766157, K_theta_dot=7.487802, K_int=-2.989396, m_l=39.06, L=36.0),
    (39.06, 41.138): GainSet(K_vel=1.573983, K_theta=-42.368817, K_theta_dot=7.794217, K_int=-2.559384, m_l=39.06, L=41.138),
}

BRIDGE_GAINS = {
    (5.0, 12.0): GainSet(K_vel=1.474438, K_theta=-85.153712, K_theta_dot=2.959575, K_int=-2.778498, m_l=5.0, L=12.0),
    (5.0, 18.0): GainSet(K_vel=1.214407, K_theta=-71.393359, K_theta_dot=4.006137, K_int=-1.993309, m_l=5.0, L=18.0),
    (5.0, 24.0): GainSet(K_vel=1.145392, K_theta=-81.289995, K_theta_dot=5.874692, K_int=-1.688328, m_l=5.0, L=24.0),
    (5.0, 30.0): GainSet(K_vel=1.084614, K_theta=-81.175448, K_theta_dot=6.719478, K_int=-1.430010, m_l=5.0, L=30.0),
    (5.0, 36.0): GainSet(K_vel=0.988081, K_theta=-81.011883, K_theta_dot=7.566789, K_int=-1.211216, m_l=5.0, L=36.0),
    (5.0, 41.138): GainSet(K_vel=0.906169, K_theta=-80.601053, K_theta_dot=8.083321, K_int=-1.025893, m_l=5.0, L=41.138),
    (10.0, 12.0): GainSet(K_vel=1.583569, K_theta=-65.992224, K_theta_dot=2.459800, K_int=-3.153714, m_l=10.0, L=12.0),
    (10.0, 18.0): GainSet(K_vel=1.353392, K_theta=-65.418047, K_theta_dot=4.149176, K_int=-2.368739, m_l=10.0, L=18.0),
    (10.0, 24.0): GainSet(K_vel=1.272522, K_theta=-68.270914, K_theta_dot=5.122709, K_int=-2.006248, m_l=10.0, L=24.0),
    (10.0, 30.0): GainSet(K_vel=1.196593, K_theta=-67.896792, K_theta_dot=5.798150, K_int=-1.699285, m_l=10.0, L=30.0),
    (10.0, 36.0): GainSet(K_vel=1.085810, K_theta=-67.659695, K_theta_dot=6.551000, K_int=-1.430010, m_l=10.0, L=36.0),
    (10.0, 41.138): GainSet(K_vel=0.993683, K_theta=-67.142017, K_theta_dot=7.009470, K_int=-1.211216, m_l=10.0, L=41.138),
    (15.0, 12.0): GainSet(K_vel=1.639401, K_theta=-55.772591, K_theta_dot=2.148007, K_int=-3.367202, m_l=15.0, L=12.0),
    (15.0, 18.0): GainSet(K_vel=1.428741, K_theta=-59.600980, K_theta_dot=4.081198, K_int=-2.590520, m_l=15.0, L=18.0),
    (15.0, 24.0): GainSet(K_vel=1.341177, K_theta=-61.357731, K_theta_dot=4.770281, K_int=-2.193717, m_l=15.0, L=24.0),
    (15.0, 30.0): GainSet(K_vel=1.256860, K_theta=-60.835467, K_theta_dot=5.378957, K_int=-1.858223, m_l=15.0, L=30.0),
    (15.0, 36.0): GainSet(K_vel=1.138379, K_theta=-60.485141, K_theta_dot=6.082725, K_int=-1.563855, m_l=15.0, L=36.0),
    (15.0, 41.138): GainSet(K_vel=1.040713, K_theta=-59.939228, K_theta_dot=6.512630, K_int=-1.324545, m_l=15.0, L=41.138),
    (20.0, 12.0): GainSet(K_vel=1.673809, K_theta=-49.278566, K_theta_dot=1.936082, K_int=-3.509748, m_l=20.0, L=12.0),
    (20.0, 18.0): GainSet(K_vel=1.477017, K_theta=-54.671049, K_theta_dot=3.953579, K_int=-2.744158, m_l=20.0, L=18.0),
    (20.0, 24.0): GainSet(K_vel=1.385078, K_theta=-56.331206, K_theta_dot=4.510006, K_int=-2.323985, m_l=20.0, L=24.0),
    (20.0, 30.0): GainSet(K_vel=1.295325, K_theta=-55.700085, K_theta_dot=5.074706, K_int=-1.968532, m_l=20.0, L=30.0),
    (20.0, 36.0): GainSet(K_vel=1.171900, K_theta=-55.266205, K_theta_dot=5.742360, K_int=-1.656678, m_l=20.0, L=36.0),
    (20.0, 41.138): GainSet(K_vel=1.070685, K_theta=-54.704285, K_theta_dot=6.151740, K_int=-1.403009, m_l=20.0, L=41.138),
    (25.0, 12.0): GainSet(K_vel=1.697415, K_theta=-44.757099, K_theta_dot=1.782806, K_int=-3.614169, m_l=25.0, L=12.0),
    (25.0, 18.0): GainSet(K_vel=1.511197, K_theta=-50.569472, K_theta_dot=3.805560, K_int=-2.860155, m_l=25.0, L=18.0),
    (25.0, 24.0): GainSet(K_vel=1.416091, K_theta=-52.333201, K_theta_dot=4.296884, K_int=-2.422237, m_l=25.0, L=24.0),
    (25.0, 30.0): GainSet(K_vel=1.322475, K_theta=-51.604723, K_theta_dot=4.832264, K_int=-2.051833, m_l=25.0, L=30.0),
    (25.0, 36.0): GainSet(K_vel=1.195536, K_theta=-51.106428, K_theta_dot=5.471241, K_int=-1.726779, m_l=25.0, L=36.0),
    (25.0, 41.138): GainSet(K_vel=1.091807, K_theta=-50.536104, K_theta_dot=5.864387, K_int=-1.462378, m_l=25.0, L=41.138),
    (30.0, 12.0): GainSet(K_vel=1.714856, K_theta=-41.411130, K_theta_dot=1.667232, K_int=-3.695305, m_l=30.0, L=12.0),
    (30.0, 18.0): GainSet(K_vel=1.536968, K_theta=-47.112117, K_theta_dot=3.654316, K_int=-2.952174, m_l=30.0, L=18.0),
    (30.0, 24.0): GainSet(K_vel=1.439462, K_theta=-49.029803, K_theta_dot=4.116744, K_int=-2.500171, m_l=30.0, L=24.0),
    (30.0, 30.0): GainSet(K_vel=1.342918, K_theta=-48.216654, K_theta_dot=4.631835, K_int=-2.117859, m_l=30.0, L=30.0),
    (30.0, 36.0): GainSet(K_vel=1.213322, K_theta=-47.668728, K_theta_dot=5.247167, K_int=-1.782352, m_l=30.0, L=36.0),
    (30.0, 41.138): GainSet(K_vel=1.107693, K_theta=-47.098416, K_theta_dot=5.627244, K_int=-1.509426, m_l=30.0, L=41.138),
    (35.0, 12.0): GainSet(K_vel=1.728399, K_theta=-38.834032, K_theta_dot=1.577457, K_int=-3.760773, m_l=35.0, L=12.0),
    (35.0, 18.0): GainSet(K_vel=1.557223, K_theta=-44.155799, K_theta_dot=3.507672, K_int=-3.027663, m_l=35.0, L=18.0),
    (35.0, 24.0): GainSet(K_vel=1.457816, K_theta=-46.230973, K_theta_dot=3.961803, K_int=-2.564115, m_l=35.0, L=24.0),
    (35.0, 30.0): GainSet(K_vel=1.358963, K_theta=-45.348270, K_theta_dot=4.462447, K_int=-2.172023, m_l=35.0, L=30.0),
    (35.0, 36.0): GainSet(K_vel=1.227273, K_theta=-44.762885, K_theta_dot=5.057771, K_int=-1.827947, m_l=35.0, L=36.0),
    (35.0, 41.138): GainSet(K_vel=1.120152, K_theta=-44.198305, K_theta_dot=5.427042, K_int=-1.547990, m_l=35.0, L=41.138),
    (39.06, 12.0): GainSet(K_vel=1.737409, K_theta=-37.067347, K_theta_dot=1.515273, K_int=-3.805560, m_l=39.06, L=12.0),
    (39.06, 18.0): GainSet(K_vel=1.570762, K_theta=-42.022933, K_theta_dot=3.393430, K_int=-3.079234, m_l=39.06, L=18.0),
    (39.06, 24.0): GainSet(K_vel=1.470081, K_theta=-44.209172, K_theta_dot=3.844697, K_int=-2.607802, m_l=39.06, L=24.0),
    (39.06, 30.0): GainSet(K_vel=1.369680, K_theta=-43.280145, K_theta_dot=4.336495, K_int=-2.209040, m_l=39.06, L=30.0),
    (39.06, 36.0): GainSet(K_vel=1.236585, K_theta=-42.671378, K_theta_dot=4.918073, K_int=-1.859063, m_l=39.06, L=36.0),
    (39.06, 41.138): GainSet(K_vel=1.128468, K_theta=-42.115315, K_theta_dot=5.279618, K_int=-1.574328, m_l=39.06, L=41.138),
}


def get_gains(axis: str, m_l: float, L: float) -> GainSet:
    """
    Get optimal gains for given operating point using Bilinear Interpolation.
    
    If m_l or L are outside the table bounds, the nearest boundary values are used
    (clipping), which prevents extrapolation errors.
    
    Args:
        axis: "trolley" or "bridge"
        m_l: Load mass [lbm]
        L: Cable length [in]
        
    Returns:
        Interpolated GainSet
    """
    gains_dict = TROLLEY_GAINS if axis == "trolley" else BRIDGE_GAINS
    
    # 1. Extract the grid coordinates
    keys = list(gains_dict.keys())
    # Use a set to get unique values, then sort them
    masses = sorted(list(set(k[0] for k in keys)))
    lengths = sorted(list(set(k[1] for k in keys)))
    
    # Clip inputs to table bounds to avoid extrapolation errors
    m_val = np.clip(m_l, masses[0], masses[-1])
    L_val = np.clip(L, lengths[0], lengths[-1])
    
    # 2. Find the bounding indices
    # searchsorted finds the index where m_val should be inserted to maintain order
    # We subtract 1 to get the lower bound index
    i = np.searchsorted(masses, m_val)
    if i > 0 and (i == len(masses) or masses[i] != m_val):
        i -= 1
    # Safety clamp for edge cases
    i = max(0, min(i, len(masses) - 2))

    j = np.searchsorted(lengths, L_val)
    if j > 0 and (j == len(lengths) or lengths[j] != L_val):
        j -= 1
    j = max(0, min(j, len(lengths) - 2))
    
    # Get the four corner points
    m0, m1 = masses[i], masses[i+1]
    L0, L1 = lengths[j], lengths[j+1]
    
    # 3. Get gains at the four corners
    try:
        q11 = gains_dict[(m0, L0)].as_array()
        q21 = gains_dict[(m1, L0)].as_array()
        q12 = gains_dict[(m0, L1)].as_array()
        q22 = gains_dict[(m1, L1)].as_array()
    except KeyError:
        # Fallback if grid is not perfectly rectangular (should not happen with this data)
        return gains_dict[(m0, L0)]
    
    # 4. Bilinear Interpolation Formula
    # Weights based on distance from corners
    denom = (m1 - m0) * (L1 - L0)
    if denom == 0: # Should not happen unless grid points are duplicate
        return gains_dict[(m0, L0)]
        
    wa = (m1 - m_val) * (L1 - L_val)
    wb = (m_val - m0) * (L1 - L_val)
    wc = (m1 - m_val) * (L_val - L0)
    wd = (m_val - m0) * (L_val - L0)
    
    k_interp = (wa*q11 + wb*q21 + wc*q12 + wd*q22) / denom
    
    return GainSet(
        K_vel=k_interp[0], 
        K_theta=k_interp[1], 
        K_theta_dot=k_interp[2], 
        K_int=k_interp[3], 
        m_l=m_val, 
        L=L_val
    )