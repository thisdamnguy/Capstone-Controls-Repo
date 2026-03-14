"""
UHplift Crane Control - Core Modules

Modules:
    config      - System configuration and gain tables
    controller  - LQI control law implementation
    reference   - Velocity profile generation
    estimator   - State estimation from sensors
    plant       - Plant model for simulation
"""

from .config import (
    SystemConfig,
    AxisConfig,
    GainSet,
    ControlMode,
    DEFAULT_CONFIG,
    get_gains,
    G_IN_PER_S2,
    G_C,
)

from .controller import (
    LQIController,
    DualAxisController,
    create_controller,
)

from .reference import (
    TrapezoidalProfile,
    ReferenceGenerator,
    ManualModeGenerator,
    create_default_trolley_profile,
    create_default_bridge_profile,
)

from .estimator import (
    SensorReadings,
    StateEstimator,
    LowPassFilter,
    ComplementaryFilter,
    SimulatedSensors,
)

from .plant import (
    CranePlant,
    PlantMatrices,
    build_plant_matrices,
    compute_natural_frequency,
    compute_period,
)
