#!/usr/bin/env python3
"""
UHplift Crane — Post-Run Plot Generator
Team 11 Capstone II

Reads a CSV log file produced by main.py --log and generates three
publication-ready PNG plots for the validation milestone report:

    1. Velocity tracking:  v_ref and v_trolley vs time
    2. Sway angle:         theta_deg vs time with ±2° requirement band
    3. Loop timing:        histogram of loop_ms with 5ms deadline marker

Usage:
    python3 plot_run.py                          # Plot latest run
    python3 plot_run.py ~/crane_logs/run_*.csv   # Plot specific file

Outputs are saved alongside the CSV with matching filename prefix:
    run_20260315_143022_velocity.png
    run_20260315_143022_sway.png
    run_20260315_143022_timing.png

Dependencies:
    pip3 install matplotlib numpy --break-system-packages
"""

import sys
import os
import glob
import csv
import numpy as np

# matplotlib backend must be set before importing pyplot
# (Pi may not have a display)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def find_latest_csv() -> str:
    """Find the most recently modified CSV in ~/crane_logs/."""
    log_dir = os.path.expanduser("~/crane_logs")
    pattern = os.path.join(log_dir, "run_*.csv")
    files = glob.glob(pattern)
    if not files:
        print(f"[ERROR] No CSV files found in {log_dir}")
        print("  Run:  python3 main.py --sim --real-joystick --log")
        sys.exit(1)
    latest = max(files, key=os.path.getmtime)
    return latest


def load_csv(path: str) -> dict:
    """
    Load CSV into a dict of numpy arrays.
    
    Returns:
        dict with keys matching CSV column headers, values as float arrays
        (except 'mode' which stays as string list)
    """
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    
    if not rows:
        print(f"[ERROR] CSV file is empty: {path}")
        sys.exit(1)
    
    data = {}
    for key in rows[0].keys():
        if key == "mode":
            data[key] = [r[key] for r in rows]
        else:
            data[key] = np.array([float(r[key]) for r in rows])
    
    return data


def plot_velocity(data: dict, out_path: str):
    """Plot 1: Velocity reference vs measured velocity."""
    t = data["t_s"]
    v_ref = data["v_ref_trolley"]
    v_meas = data["v_trolley"]
    v_cmd = data["v_cmd_trolley"]
    
    fig, ax = plt.subplots(figsize=(10, 4))
    
    ax.plot(t, v_ref, label="v_ref (rate-limited)", color="#2196F3",
            linewidth=1.5)
    ax.plot(t, v_cmd, label="v_cmd (after LQI F→v)", color="#FF9800",
            linewidth=1.2, linestyle="--")
    ax.plot(t, v_meas, label="v_trolley (encoder)", color="#4CAF50",
            linewidth=1.0, alpha=0.8)
    
    # Shade mode regions
    _shade_modes(ax, data)
    
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity [in/s]")
    ax.set_title("Trolley Velocity Tracking")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  [SAVED] {out_path}")


def plot_sway(data: dict, out_path: str):
    """Plot 2: Sway angle with ±2° requirement band."""
    t = data["t_s"]
    theta = data["theta_deg"]
    
    fig, ax = plt.subplots(figsize=(10, 4))
    
    ax.plot(t, theta, label="theta (measured)", color="#E91E63",
            linewidth=1.0)
    
    # Requirement band: ±2°
    ax.axhline(y=2.0, color="#999", linestyle="--", linewidth=0.8,
               label="requirement (±2°)")
    ax.axhline(y=-2.0, color="#999", linestyle="--", linewidth=0.8)
    ax.fill_between(t, -2.0, 2.0, alpha=0.06, color="#4CAF50")
    
    # Shade mode regions
    _shade_modes(ax, data)
    
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Sway Angle [deg]")
    ax.set_title("Payload Sway Angle")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  [SAVED] {out_path}")


def plot_timing(data: dict, out_path: str):
    """Plot 3: Loop timing histogram with 5ms deadline."""
    loop_ms = data["loop_ms"]
    
    fig, ax = plt.subplots(figsize=(8, 4))
    
    # Histogram with reasonable bins
    max_ms = min(float(np.max(loop_ms)) * 1.2, 20.0)
    bins = np.linspace(0, max_ms, 60)
    ax.hist(loop_ms, bins=bins, color="#2196F3", alpha=0.7,
            edgecolor="white", linewidth=0.5)
    
    # 5ms deadline
    ax.axvline(x=5.0, color="#F44336", linestyle="--", linewidth=1.5,
               label="5ms deadline (200 Hz)")
    
    # Stats annotation
    mean_ms = float(np.mean(loop_ms))
    p99_ms = float(np.percentile(loop_ms, 99))
    max_ms_val = float(np.max(loop_ms))
    overruns = int(np.sum(loop_ms > 5.0))
    total = len(loop_ms)
    
    stats_text = (f"mean: {mean_ms:.2f} ms\n"
                  f"p99:  {p99_ms:.2f} ms\n"
                  f"max:  {max_ms_val:.2f} ms\n"
                  f"overruns: {overruns}/{total} "
                  f"({100*overruns/total:.1f}%)")
    ax.text(0.97, 0.95, stats_text, transform=ax.transAxes,
            fontsize=8, verticalalignment='top', horizontalalignment='right',
            fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                      alpha=0.8, edgecolor='#ccc'))
    
    ax.set_xlabel("Loop Time [ms]")
    ax.set_ylabel("Count")
    ax.set_title("Control Loop Timing Distribution")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  [SAVED] {out_path}")


def _shade_modes(ax, data: dict):
    """
    Add subtle colored background bands showing mode transitions.
    
    DISABLED = light gray, MANUAL = light blue, AUTO = light green
    """
    t = data["t_s"]
    modes = data["mode"]
    
    mode_colors = {
        "DISABLED": ("#9E9E9E", 0.08),
        "MANUAL":   ("#2196F3", 0.08),
        "AUTO":     ("#4CAF50", 0.08),
        "HOMING":   ("#FF9800", 0.08),
        "FAULT":    ("#F44336", 0.12),
    }
    
    if len(t) < 2:
        return
    
    # Walk through and find contiguous mode spans
    span_start = 0
    current_mode = modes[0]
    
    for i in range(1, len(modes)):
        if modes[i] != current_mode or i == len(modes) - 1:
            end_idx = i if modes[i] != current_mode else i
            color, alpha = mode_colors.get(current_mode, ("#9E9E9E", 0.05))
            ax.axvspan(float(t[span_start]), float(t[min(end_idx, len(t)-1)]),
                       alpha=alpha, color=color, zorder=0)
            span_start = i
            current_mode = modes[i]


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = find_latest_csv()
    
    if not os.path.isfile(csv_path):
        print(f"[ERROR] File not found: {csv_path}")
        sys.exit(1)
    
    print(f"[PLOT] Loading {csv_path}")
    data = load_csv(csv_path)
    
    n_ticks = len(data["t_s"])
    duration = float(data["t_s"][-1]) if n_ticks > 0 else 0
    print(f"  {n_ticks} ticks, {duration:.1f}s duration")
    
    # Output paths: same directory, same prefix as CSV
    base = csv_path.rsplit(".csv", 1)[0]
    
    plot_velocity(data, f"{base}_velocity.png")
    plot_sway(data, f"{base}_sway.png")
    plot_timing(data, f"{base}_timing.png")
    
    print(f"\n[DONE] 3 plots saved alongside {os.path.basename(csv_path)}")


if __name__ == "__main__":
    main()