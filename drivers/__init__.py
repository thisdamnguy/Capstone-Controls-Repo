"""
UHplift Crane Control - Hardware Drivers

Drivers:
    encoder   - LS7366R quadrature counter (SPI)
    imu       - LSM6DS3 6-axis IMU (SPI)
    stepper   - CL42T driver interface (pigpio DMA)
    joystick  - USB game controller HMI (pygame)
"""

from .encoder import (
    LS7366R,
    EncoderConfig,
    TROLLEY_ENCODER,
    BRIDGE_ENCODER,
    HOIST_ENCODER,
    create_encoder,
)

from .imu import (
    LSM6DS3,
    LSM6DS3Config,
    IMU,  # Alias for backwards compatibility
)

from .stepper import (
    StepperDriver,
    StepperConfig,
    TROLLEY_STEPPER,
    BRIDGE_STEPPER,
    HOIST_STEPPER,
    create_stepper,
)

from .joystick import (
    JoystickDriver,
    JoystickConfig,
    detect_controller_type,
)
