"""
Crane Control Simulation
Tests control logic without hardware

This replicates the MATLAB simulation to verify Python implementation.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.config import SystemConfig, GainSet, G_IN_PER_S2
from core.plant import CranePlant, compute_period
from core.controller import LQIController
from core.reference import ReferenceGenerator, TrapezoidalProfile
from core.estimator import StateEstimator, SimulatedSensors, SensorReadings


@dataclass
class SimulationConfig:
    """Simulation parameters"""
    t_final: float = 15.0       # Simulation duration [s]
    dt: float = 0.005           # Timestep [s] (matches MATLAB)
    
    # Disturbance injection (for testing robustness)
    disturbance_force: float = 0.0      # Disturbance magnitude [lbf]
    disturbance_start: float = 8.0      # When to apply disturbance [s]
    disturbance_duration: float = 0.5   # How long disturbance lasts [s]
    
    # Sensor noise
    add_sensor_noise: bool = False
    position_noise_std: float = 0.001
    theta_noise_std: float = 0.002
    theta_dot_noise_std: float = 0.01


@dataclass
class SimulationResults:
    """Container for simulation results"""
    t: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # State histories
    position: np.ndarray = field(default_factory=lambda: np.array([]))
    velocity: np.ndarray = field(default_factory=lambda: np.array([]))
    theta: np.ndarray = field(default_factory=lambda: np.array([]))
    theta_dot: np.ndarray = field(default_factory=lambda: np.array([]))
    integrator: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # Reference
    v_ref: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # Control
    force: np.ndarray = field(default_factory=lambda: np.array([]))
    
    # Derived metrics
    accel_g: np.ndarray = field(default_factory=lambda: np.array([]))


def run_simulation(gains: GainSet,
                   profile: TrapezoidalProfile,
                   config: SystemConfig,
                   sim_config: SimulationConfig,
                   axis: str = "trolley") -> SimulationResults:
    """
    Run closed-loop simulation.
    
    Args:
        gains: LQR gain set
        profile: Velocity profile to track
        config: System configuration
        sim_config: Simulation parameters
        axis: "trolley" or "bridge"
        
    Returns:
        SimulationResults with time histories
    """
    # Time vector
    t = np.arange(0, sim_config.t_final + sim_config.dt, sim_config.dt)
    n_steps = len(t)
    
    # Initialize components
    plant = CranePlant(config, axis)
    controller = LQIController(gains, config.trolley.f_max if axis == "trolley" else config.bridge.f_max)
    ref_gen = ReferenceGenerator(profile)
    
    # Pre-allocate result arrays
    results = SimulationResults(
        t=t,
        position=np.zeros(n_steps),
        velocity=np.zeros(n_steps),
        theta=np.zeros(n_steps),
        theta_dot=np.zeros(n_steps),
        integrator=np.zeros(n_steps),
        v_ref=np.zeros(n_steps),
        force=np.zeros(n_steps),
        accel_g=np.zeros(n_steps)
    )
    
    # Simulation loop
    for i, ti in enumerate(t[:-1]):
        # Get reference
        v_ref, _ = ref_gen.get_reference(ti)
        results.v_ref[i] = v_ref
        
        # Get current state
        state = plant.x_full
        results.position[i] = state[0]
        results.velocity[i] = state[1]
        results.theta[i] = state[2]
        results.theta_dot[i] = state[3]
        results.integrator[i] = controller.state.integrator
        
        # Compute control
        u, ctrl_info = controller.compute(
            v=state[1],
            theta=state[2],
            theta_dot=state[3],
            v_ref=v_ref,
            dt=sim_config.dt
        )
        
        # Add disturbance if specified
        if (sim_config.disturbance_force != 0 and 
            sim_config.disturbance_start <= ti < sim_config.disturbance_start + sim_config.disturbance_duration):
            u += sim_config.disturbance_force
        
        results.force[i] = ctrl_info['u_saturated']
        
        # Step plant
        _, plant_info = plant.step(u, v_ref, sim_config.dt)
        results.accel_g[i] = plant_info['accel_g']
    
    # Fill last values
    results.v_ref[-1] = results.v_ref[-2]
    results.position[-1] = plant.x_full[0]
    results.velocity[-1] = plant.x_full[1]
    results.theta[-1] = plant.x_full[2]
    results.theta_dot[-1] = plant.x_full[3]
    results.integrator[-1] = controller.state.integrator
    results.force[-1] = results.force[-2]
    results.accel_g[-1] = results.accel_g[-2]
    
    return results


def compute_metrics(results: SimulationResults,
                    profile: TrapezoidalProfile,
                    config: SystemConfig) -> Dict:
    """
    Compute performance metrics from simulation results.
    
    Matches the metric definitions in MATLAB code.
    """
    dt = results.t[1] - results.t[0]
    v_target = profile.v_target
    
    # Peak acceleration
    peak_accel_g = np.max(np.abs(results.accel_g))
    
    # Peak sway angle
    peak_sway_deg = np.max(np.abs(np.degrees(results.theta)))
    
    # Velocity settling time (time to enter and stay within 5% band)
    band_low = v_target * 0.95
    band_high = v_target * 1.05
    
    t_start = profile.t_start
    t_cruise_end = profile.t_cruise_end
    
    idx_start = int(t_start / dt)
    idx_end = int(t_cruise_end / dt)
    
    vel_window = results.velocity[idx_start:idx_end]
    outside = (vel_window < band_low) | (vel_window > band_high)
    last_outside = np.where(outside)[0]
    
    if len(last_outside) == 0:
        vel_settle_time = 0.0
    else:
        vel_settle_time = last_outside[-1] * dt
    
    # Steady-state error
    idx_ss_start = int((t_cruise_end - 0.5) / dt)
    idx_ss_end = int(t_cruise_end / dt)
    v_avg_ss = np.mean(results.velocity[idx_ss_start:idx_ss_end])
    ss_error_pct = abs(v_avg_ss - v_target) / v_target * 100
    
    # Sway settling time (time after stop for sway to settle below 0.1°)
    t_stop = profile.t_stop
    idx_stop = int(t_stop / dt)
    
    sway_after_stop = np.degrees(results.theta[idx_stop:])
    sway_threshold = 0.1  # degrees
    outside_sway = np.abs(sway_after_stop) > sway_threshold
    last_outside_sway = np.where(outside_sway)[0]
    
    if len(last_outside_sway) == 0:
        sway_settle_time = 0.0
    elif last_outside_sway[-1] == len(sway_after_stop) - 1:
        sway_settle_time = float('inf')  # Never settled
    else:
        sway_settle_time = (last_outside_sway[-1] + 1) * dt
    
    # Braking distance
    braking_dist = results.position[-1] - results.position[idx_stop]
    
    return {
        'peak_accel_g': peak_accel_g,
        'peak_sway_deg': peak_sway_deg,
        'vel_settle_time': vel_settle_time,
        'ss_error_pct': ss_error_pct,
        'sway_settle_time': sway_settle_time,
        'braking_dist': braking_dist
    }


def plot_results(results: SimulationResults,
                 metrics: Dict,
                 profile: TrapezoidalProfile,
                 title: str = "LQR Control Simulation"):
    """
    Plot simulation results (mimics MATLAB plots).
    """
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    
    t = results.t
    
    # Position
    ax = axes[0]
    ax.plot(t, results.position, 'b-', linewidth=1.5)
    ax.axvline(profile.t_cruise_end, color='g', linestyle='--', label='Stop Cmd')
    ax.set_ylabel('Position [in]')
    ax.set_title(f"Position (Braking Distance: {metrics['braking_dist']:.2f} in)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Velocity
    ax = axes[1]
    ax.plot(t, results.velocity, 'b-', linewidth=1.5, label='Actual')
    ax.plot(t, results.v_ref, 'k--', linewidth=1, label='Reference')
    ax.fill_between(t, profile.v_target * 0.95, profile.v_target * 1.05,
                    alpha=0.2, color='g', label='±5% Band')
    ax.set_ylabel('Velocity [in/s]')
    ax.set_title(f"Velocity (Settling Time: {metrics['vel_settle_time']:.3f} s)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Acceleration
    ax = axes[2]
    ax.plot(t, results.accel_g, 'b-', linewidth=1.5)
    ax.axhline(0.268, color='r', linestyle='--', label='Limit')
    ax.axhline(-0.268, color='r', linestyle='--')
    ax.set_ylabel('Acceleration [g]')
    ax.set_title(f"Acceleration (Peak: {metrics['peak_accel_g']:.3f} g)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Sway angle
    ax = axes[3]
    ax.plot(t, np.degrees(results.theta), 'b-', linewidth=1.5)
    ax.axhline(2.0, color='r', linestyle='--', label='Limit')
    ax.axhline(-2.0, color='r', linestyle='--')
    ax.set_ylabel('Sway Angle [°]')
    ax.set_title(f"Pendulum Sway (Settling Time: {metrics['sway_settle_time']:.3f} s)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # Control force
    ax = axes[4]
    ax.plot(t, results.force, 'b-', linewidth=1.5)
    ax.set_ylabel('Force [lbf]')
    ax.set_xlabel('Time [s]')
    ax.set_title('Control Effort')
    ax.grid(True, alpha=0.3)
    
    fig.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    return fig


def run_disturbance_comparison(gains: GainSet,
                               profile: TrapezoidalProfile,
                               config: SystemConfig,
                               disturbance_force: float = 2.0):
    """
    Run simulation with and without disturbance to demonstrate rejection.
    """
    sim_config_nominal = SimulationConfig()
    sim_config_disturbed = SimulationConfig(
        disturbance_force=disturbance_force,
        disturbance_start=8.0,
        disturbance_duration=0.5
    )
    
    print("Running nominal simulation...")
    results_nominal = run_simulation(gains, profile, config, sim_config_nominal)
    metrics_nominal = compute_metrics(results_nominal, profile, config)
    
    print("Running simulation with disturbance...")
    results_disturbed = run_simulation(gains, profile, config, sim_config_disturbed)
    metrics_disturbed = compute_metrics(results_disturbed, profile, config)
    
    # Plot comparison
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    t = results_nominal.t
    
    # Velocity comparison
    ax = axes[0]
    ax.plot(t, results_nominal.velocity, 'b-', linewidth=1.5, label='Nominal')
    ax.plot(t, results_disturbed.velocity, 'r-', linewidth=1.5, label=f'With {disturbance_force} lbf disturbance')
    ax.plot(t, results_nominal.v_ref, 'k--', linewidth=1, label='Reference')
    ax.axvline(8.0, color='orange', linestyle=':', label='Disturbance start')
    ax.set_ylabel('Velocity [in/s]')
    ax.set_title('Velocity - Disturbance Rejection')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Sway comparison
    ax = axes[1]
    ax.plot(t, np.degrees(results_nominal.theta), 'b-', linewidth=1.5, label='Nominal')
    ax.plot(t, np.degrees(results_disturbed.theta), 'r-', linewidth=1.5, label='With disturbance')
    ax.axvline(8.0, color='orange', linestyle=':')
    ax.set_ylabel('Sway Angle [°]')
    ax.set_title('Sway Angle - Disturbance Rejection')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Integrator state
    ax = axes[2]
    ax.plot(t, results_nominal.integrator, 'b-', linewidth=1.5, label='Nominal')
    ax.plot(t, results_disturbed.integrator, 'r-', linewidth=1.5, label='With disturbance')
    ax.axvline(8.0, color='orange', linestyle=':')
    ax.set_ylabel('Integrator State')
    ax.set_xlabel('Time [s]')
    ax.set_title('Integrator Action (Disturbance Rejection Mechanism)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    fig.suptitle('LQI Disturbance Rejection Demonstration', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    return fig, (results_nominal, metrics_nominal), (results_disturbed, metrics_disturbed)


if __name__ == "__main__":
    # Test with design point gains from MATLAB
    print("="*60)
    print("Crane LQI Control Simulation - Python Implementation")
    print("="*60)
    
    # Design point gains (from LQR_Results_Design_Point.pdf)
    gains = GainSet(
        K_vel=1.542656,
        K_theta=-17.632736,
        K_theta_dot=9.460403,
        K_int=-2.353382,
        m_l=39.09,
        L=41.138
    )
    
    # System configuration
    config = SystemConfig()
    config.m_l = 39.09
    config.L = 41.138
    
    # Profile matching MATLAB
    profile = TrapezoidalProfile(
        v_target=4.6,
        accel_g=0.20,
        t_start=1.0,
        cruise_duration=4.0  # Will cruise until ~5s, then ramp down
    )
    
    # Run nominal simulation
    sim_config = SimulationConfig()
    print(f"\nRunning simulation: {sim_config.t_final}s, dt={sim_config.dt}s")
    print(f"Gains: K_vel={gains.K_vel:.3f}, K_θ={gains.K_theta:.3f}, " +
          f"K_θ̇={gains.K_theta_dot:.3f}, K_int={gains.K_int:.3f}")
    
    results = run_simulation(gains, profile, config, sim_config)
    metrics = compute_metrics(results, profile, config)
    
    print("\n" + "="*40)
    print("Performance Metrics:")
    print("="*40)
    print(f"  Peak Acceleration: {metrics['peak_accel_g']:.3f} g (limit: 0.268 g)")
    print(f"  Peak Sway:         {metrics['peak_sway_deg']:.3f}° (limit: 2.0°)")
    print(f"  Vel Settling:      {metrics['vel_settle_time']:.3f} s")
    print(f"  SS Error:          {metrics['ss_error_pct']:.2f}% (limit: 1.0%)")
    print(f"  Sway Settling:     {metrics['sway_settle_time']:.3f} s")
    print(f"  Braking Distance:  {metrics['braking_dist']:.2f} in")
    
    # Plot
    fig = plot_results(results, metrics, profile)
    plt.savefig('simulation_results.png', dpi=150, bbox_inches='tight')
    print("\nPlot saved to simulation_results.png")
    
    # Run disturbance comparison
    print("\n" + "="*60)
    print("Disturbance Rejection Test")
    print("="*60)
    
    fig_dist, (res_nom, met_nom), (res_dist, met_dist) = run_disturbance_comparison(
        gains, profile, config, disturbance_force=2.0
    )
    plt.savefig('disturbance_rejection.png', dpi=150, bbox_inches='tight')
    print("Disturbance plot saved to disturbance_rejection.png")
    
    print("\nSimulation complete!")
