"""
LQI Controller Implementation
Matches the control law from LQR_Control.m and Simulink model

Control law:
    u = -K_vel·v - K_θ·θ - K_θ̇·θ̇ - K_int·∫(v_ref - v)dt

With:
    - Output saturation: |u| ≤ f_max
    - Anti-windup: Stop integrating when saturated
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
from .config import GainSet, get_gains, AxisConfig


@dataclass
class ControllerState:
    """Internal state of the controller"""
    integrator: float = 0.0
    last_error: float = 0.0
    last_output: float = 0.0
    saturated: bool = False


class LQIController:
    """
    LQR + Integral action controller for crane sway control.
    
    Implements velocity tracking with sway suppression.
    """
    
    def __init__(self, 
                 gains: GainSet,
                 f_max: float = 32.18,
                 anti_windup: bool = True,
                 integrator_limit: Optional[float] = None):
        """
        Initialize controller.
        
        Args:
            gains: LQR gain set (K_vel, K_theta, K_theta_dot, K_int)
            f_max: Force saturation limit [lbf]
            anti_windup: Enable anti-windup (stop integration when saturated)
            integrator_limit: Optional hard limit on integrator state
        """
        self.gains = gains
        self.f_max = f_max
        self.anti_windup = anti_windup
        self.integrator_limit = integrator_limit
        
        # Convert gains to array for vectorized computation
        # Note: The gain array is [K_vel, K_theta, K_theta_dot, K_int]
        self.K = gains.as_array()
        
        # Internal state
        self.state = ControllerState()
        
    def reset(self):
        """Reset controller state (integrator, etc.)"""
        self.state = ControllerState()
    
    def set_gains(self, gains: GainSet):
        """Update gains (e.g., when operating point changes)"""
        self.gains = gains
        self.K = gains.as_array()
        
    def compute(self, 
                v: float, 
                theta: float, 
                theta_dot: float,
                v_ref: float,
                dt: float) -> Tuple[float, dict]:
        """
        Compute control output.
        
        Args:
            v: Current velocity [in/s]
            theta: Current sway angle [rad]
            theta_dot: Current sway rate [rad/s]
            v_ref: Reference velocity [in/s]
            dt: Timestep [s]
            
        Returns:
            Tuple of (control_force [lbf], info_dict)
        """
        # Velocity error for integral term
        v_error = v_ref - v
        
        # State feedback portion: u_fb = -K_vel·v - K_θ·θ - K_θ̇·θ̇
        # Note: In the MATLAB code, the gains are applied as:
        #   u_fb = -([K_vel, K_theta] * x_curr(2:4))
        # Where x_curr(2:4) = [v, θ, θ̇]
        u_feedback = -(self.K[0] * v + self.K[1] * theta + self.K[2] * theta_dot)
        
        # Integral term: u_int = -K_int · ∫e_v
        u_integral = -self.K[3] * self.state.integrator
        
        # Total control (before saturation)
        u_total = u_feedback + u_integral
        
        # Saturation
        u_saturated = np.clip(u_total, -self.f_max, self.f_max)
        is_saturated = abs(u_total) > self.f_max
        
        # Update integrator (with anti-windup)
        if self.anti_windup and is_saturated:
            # Don't integrate when saturated (prevents windup)
            pass
        else:
            self.state.integrator += v_error * dt
        
        # Apply integrator limit if specified
        if self.integrator_limit is not None:
            self.state.integrator = np.clip(
                self.state.integrator, 
                -self.integrator_limit, 
                self.integrator_limit
            )
        
        # Update state tracking
        self.state.last_error = v_error
        self.state.last_output = u_saturated
        self.state.saturated = is_saturated
        
        info = {
            'u_feedback': u_feedback,
            'u_integral': u_integral,
            'u_total': u_total,
            'u_saturated': u_saturated,
            'saturated': is_saturated,
            'integrator': self.state.integrator,
            'v_error': v_error,
        }
        
        return u_saturated, info


class DualAxisController:
    """
    Manages controllers for both trolley and bridge axes.
    
    In a full implementation, this coordinates motion between axes
    and handles mode switching.
    """
    
    def __init__(self, 
                 trolley_gains: GainSet,
                 bridge_gains: GainSet,
                 f_max: float = 32.18):
        """
        Initialize dual-axis controller.
        
        Args:
            trolley_gains: Gains for trolley axis
            bridge_gains: Gains for bridge axis  
            f_max: Force saturation limit [lbf]
        """
        self.trolley = LQIController(trolley_gains, f_max)
        self.bridge = LQIController(bridge_gains, f_max)
        
        self.active_axis = None  # "trolley", "bridge", or None
        
    def reset(self):
        """Reset both controllers"""
        self.trolley.reset()
        self.bridge.reset()
        
    def set_active_axis(self, axis: Optional[str]):
        """Set which axis is currently active (for sequential moves)"""
        self.active_axis = axis
        
    def compute_trolley(self, v, theta, theta_dot, v_ref, dt):
        """Compute trolley control"""
        return self.trolley.compute(v, theta, theta_dot, v_ref, dt)
    
    def compute_bridge(self, v, theta, theta_dot, v_ref, dt):
        """Compute bridge control"""
        return self.bridge.compute(v, theta, theta_dot, v_ref, dt)


class DisturbanceObserver:
    """
    Optional disturbance observer for improved rejection.
    
    This estimates the lumped disturbance d acting on the system:
        ẋ = Ax + Bu + Bd·d
        
    And compensates for it in the control law.
    
    This is a more advanced addition beyond the basic LQI.
    """
    
    def __init__(self, 
                 observer_bandwidth: float = 10.0,
                 dt: float = 0.01):
        """
        Initialize disturbance observer.
        
        Args:
            observer_bandwidth: Observer bandwidth [rad/s]
            dt: Sample time [s]
        """
        self.bandwidth = observer_bandwidth
        self.dt = dt
        
        # Observer state
        self.d_hat = 0.0  # Estimated disturbance
        self.x_hat = np.zeros(4)  # State estimate
        
        # Low-pass filter coefficient
        self.alpha = dt * observer_bandwidth / (1 + dt * observer_bandwidth)
        
    def reset(self):
        """Reset observer state"""
        self.d_hat = 0.0
        self.x_hat = np.zeros(4)
        
    def update(self, 
               x_measured: np.ndarray, 
               u: float,
               A: np.ndarray,
               B: np.ndarray) -> float:
        """
        Update disturbance estimate.
        
        Args:
            x_measured: Measured state [x, v, θ, θ̇]
            u: Applied control input
            A: System A matrix
            B: System B matrix
            
        Returns:
            Estimated disturbance force [lbf]
        """
        # Predict next state (model-based)
        x_pred = self.x_hat + (A @ self.x_hat + B.flatten() * u) * self.dt
        
        # Innovation (difference between measured and predicted)
        innovation = x_measured - x_pred
        
        # Disturbance shows up primarily in velocity (state 1)
        # Estimate disturbance from velocity innovation
        d_innovation = innovation[1] / self.dt  # Approximate acceleration disturbance
        
        # Low-pass filter the disturbance estimate
        self.d_hat = (1 - self.alpha) * self.d_hat + self.alpha * d_innovation
        
        # Update state estimate
        self.x_hat = x_measured.copy()
        
        return self.d_hat


# Factory function for easy controller creation
def create_controller(axis: str, 
                      m_l: float, 
                      L: float,
                      f_max: float = 32.18) -> LQIController:
    """
    Create a controller with appropriate gains for the operating point.
    
    Args:
        axis: "trolley" or "bridge"
        m_l: Load mass [lbm]
        L: Cable length [in]
        f_max: Force saturation limit [lbf]
        
    Returns:
        Configured LQIController
    """
    gains = get_gains(axis, m_l, L)
    return LQIController(gains, f_max)
