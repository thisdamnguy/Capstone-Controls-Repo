"""
Dynamic Hoisting Simulation
Tests Gain Scheduling by changing cable length L during the move.
"""

import numpy as np
import matplotlib.pyplot as plt
from core.config import SystemConfig, get_gains
from core.plant import CranePlant
from core.controller import LQIController
from core.reference import ReferenceGenerator, TrapezoidalProfile

def run_dynamic_test():
    # 1. Setup
    config = SystemConfig()
    m_l = 25.0          # Fixed Mass [lbm]
    L_start = 40.0      # Start low [in]
    L_end = 20.0        # Hoist up to [in]
    
    # Create Profile (Standard Move)
    profile = TrapezoidalProfile(
        v_target=4.6,
        accel_g=0.20,
        t_start=1.0,
        cruise_duration=4.0
    )
    
    t_final = 10.0
    dt = 0.01
    t_array = np.arange(0, t_final, dt)
    
    # Initialize Plant & Controller at Start Conditions
    plant = CranePlant(config, "trolley")
    plant.set_load_config(m_l, L_start)
    
    initial_gains = get_gains("trolley", m_l, L_start)
    controller = LQIController(initial_gains, config.trolley.f_max)
    ref_gen = ReferenceGenerator(profile)
    
    # Data Logging
    history = {
        't': [], 'pos': [], 'vel': [], 'sway': [], 'L': [], 'K_theta': []
    }
    
    print(f"Starting Dynamic Hoist Test: Mass={m_l}lb, Hoisting {L_start}\" -> {L_end}\"")
    
    # 2. Simulation Loop
    for t in t_array:
        # --- A. Simulate Hoisting (L changes with time) ---
        # Simple linear hoist from t=2s to t=6s
        if t < 2.0:
            current_L = L_start
        elif t < 6.0:
            # Linear interpolation
            progress = (t - 2.0) / 4.0
            current_L = L_start + (L_end - L_start) * progress
        else:
            current_L = L_end
            
        # --- B. Update Physics (Plant) ---
        # The pendulum dynamics change as L changes
        plant.set_load_config(m_l, current_L)
        
        # --- C. GAIN SCHEDULING (The Key Part) ---
        # Ask config.py for new gains based on current L
        new_gains = get_gains("trolley", m_l, current_L)
        # Push new gains to controller
        controller.set_gains(new_gains)
        
        # --- D. Standard Control Step ---
        v_ref, _ = ref_gen.get_reference(t)
        state = plant.x_full # [x, v, theta, theta_dot]
        
        u, _ = controller.compute(
            v=state[1],
            theta=state[2],
            theta_dot=state[3],
            v_ref=v_ref,
            dt=dt
        )
        
        plant.step(u, v_ref, dt)
        
        # Log Data
        history['t'].append(t)
        history['pos'].append(state[0])
        history['vel'].append(state[1])
        history['sway'].append(np.degrees(state[2]))
        history['L'].append(current_L)
        history['K_theta'].append(new_gains.K_theta)

    # 3. Plotting
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    
    # Plot 1: Position & Cable Length
    ax1 = axes[0]
    ax1.plot(history['t'], history['pos'], 'b-', label='Trolley Pos [in]')
    ax1.set_ylabel('Position [in]', color='b')
    ax1.grid(True)
    
    ax1b = ax1.twinx()
    ax1b.plot(history['t'], history['L'], 'g--', label='Cable Length [in]')
    ax1b.set_ylabel('Cable Length [in]', color='g')
    
    # Plot 2: Gain Scheduling in Action
    ax2 = axes[1]
    ax2.plot(history['t'], history['K_theta'], 'r-', linewidth=2)
    ax2.set_ylabel('Sway Gain (K_theta)')
    ax2.set_title('Gain Scheduling: Adapting K_theta as Length Changes')
    ax2.grid(True)
    
    # Plot 3: Sway Performance
    ax3 = axes[2]
    ax3.plot(history['t'], history['sway'], 'k-')
    ax3.set_ylabel('Sway Angle [deg]')
    ax3.set_xlabel('Time [s]')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('dynamic_hoist_result.png')
    print("Test Complete. Plot saved to 'dynamic_hoist_result.png'")

if __name__ == "__main__":
    run_dynamic_test()