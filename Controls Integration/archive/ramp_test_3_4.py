#!/usr/bin/env python3
"""
UHplift Crane - Bridge Motor 1 Validated Open Loop Test (with accel/decel ramp)
Hardware: CL42T + ULN2003AN + AM26LS32A + LS7366R + RPi4
Timing: pigpio DMA waveforms (hardware-precise, OS-independent)
References: CL42T V4.1 manual (sequence chart p.X), LS7366R datasheet
"""

import pigpio
import spidev
import time
import csv
import os
import math
from datetime import datetime

# ── Pin assignments (BCM) ─────────────────────────────────────────────────────
ENA_PIN = 17    # ULN2003AN channel 5: GPIO HIGH → ULN sinks → opto ON → driver ENABLED
DIR_PIN = 27    # ULN2003AN channel 6: GPIO HIGH → ULN sinks → opto ON → CL42T DIR active
PUL_PIN = 22    # ULN2003AN channel 7: pulses generated via DMA waveform

# ── Motion parameters ─────────────────────────────────────────────────────────
STEPS_PER_REV   = 200       # 1.8° motor, no microstepping factor here
MICROSTEP       = 4         # SW1=OFF,SW2=ON,SW3=ON,SW4=ON → 4x → 800 pulses/rev
PULSES_PER_REV  = STEPS_PER_REV * MICROSTEP   # = 800
COUNTS_PER_REV  = 4000      # 1000 PPR encoder × 4 quadrature
GEAR_RATIO      = 4.0
WHEEL_RADIUS_IN = 0.5       # Placeholder — measure actual wheel

# ── NEW: Ramp configuration (edit these freely) ───────────────────────────────
START_SPEED_HZ   = 150.0     # start frequency for ramps (pulses/sec)
TARGET_SPEED_HZ  = 2000.0    # cruise/peak frequency (pulses/sec)
ACCEL_HZ_PER_S   = 800.0    # accel slope (Hz per second)
DECEL_HZ_PER_S   = 800.0    # decel slope (Hz per second)

# Test parameters
TEST_PULSES     = int(PULSES_PER_REV * GEAR_RATIO)  # pulses = 1 output shaft revolution (per your assumption)

# Waveform chunk sizing:
# - Big chunks reduce CPU overhead but make ramps "coarser"
# - Smaller chunks make ramps smoother (recommended for ramping)
RAMP_CHUNK_PULSES = 100       # pulses per waveform chunk during ramp/cruise
MIN_HALF_PERIOD_US = 2        # safety clamp (1us min per CL42T, but keep margin)

# CL42T sequence timing (from manual sequence chart)
T1_ENA_DIR_MS   = 250       # ENA must precede DIR by ≥200ms — using 250ms margin
T2_DIR_PUL_US   = 10        # DIR must precede first PUL edge by ≥2µs — using 10µs margin

# ── LS7366R SPI opcodes ───────────────────────────────────────────────────────
CMD_WR_MDR0  = 0x88
CMD_WR_MDR1  = 0x90
CMD_CLR_CNTR = 0x20
CMD_LOAD_OTR = 0xE8
CMD_RD_OTR   = 0x68
CMD_RD_MDR0  = 0x48

MDR0_X4_FREE = 0x03
MDR1_4BYTE   = 0x00


# ── SPI / encoder ─────────────────────────────────────────────────────────────

def init_spi():
    spi = spidev.SpiDev()
    spi.open(0, 0)             # Bus 0, CE0 = GPIO8
    spi.max_speed_hz = 500_000 # Conservative 500kHz for first test
    spi.mode = 0b00            # CPOL=0, CPHA=0 — required by LS7366R
    return spi


def init_encoder(spi):
    """
    Configure LS7366R and verify SPI communication with readback.
    Returns True if comms confirmed, False if SPI appears dead.
    """
    spi.xfer2([CMD_WR_MDR0, MDR0_X4_FREE])
    time.sleep(0.001)
    spi.xfer2([CMD_WR_MDR1, MDR1_4BYTE])
    time.sleep(0.001)
    spi.xfer2([CMD_CLR_CNTR])
    time.sleep(0.001)

    # Readback MDR0 to confirm SPI is actually talking to the chip
    result = spi.xfer2([CMD_RD_MDR0, 0x00])
    readback = result[1]
    print(f"[SPI] MDR0 readback: {hex(readback)} (expect 0x03)")

    if readback == MDR0_X4_FREE:
        print("[SPI] LS7366R comms confirmed. Counter zeroed.")
        return True
    elif readback == 0x00 or readback == 0xFF:
        print("[SPI] WARNING: MDR0 readback is 0x00 or 0xFF.")
        print("      Likely causes: MISO divider wrong, CS not reaching chip,")
        print("      or oscillator not running at LS7366R fCKi pin.")
        return False
    else:
        print(f"[SPI] WARNING: Unexpected readback {hex(readback)}. Check wiring.")
        return False


def read_encoder(spi):
    """Latch counter to OTR then read. Returns signed 32-bit count."""
    spi.xfer2([CMD_LOAD_OTR])
    time.sleep(0.0001)                  # 100µs settling between commands
    raw = spi.xfer2([CMD_RD_OTR, 0x00, 0x00, 0x00, 0x00])
    count = (raw[1] << 24) | (raw[2] << 16) | (raw[3] << 8) | raw[4]
    if count >= 0x80000000:             # Convert to signed
        count -= 0x100000000
    return count


def counts_to_inches(counts):
    shaft_revs  = counts / COUNTS_PER_REV
    output_revs = shaft_revs / GEAR_RATIO
    return output_revs * 2 * 3.14159 * WHEEL_RADIUS_IN


# ── pigpio waveform helpers ───────────────────────────────────────────────────

def hz_to_half_period_us(hz: float) -> int:
    hz = max(float(hz), 1.0)
    half = int(1_000_000 / (2.0 * hz))
    return max(half, MIN_HALF_PERIOD_US)


def build_pulse_waveform(pi, n_pulses: int, half_period_us: int):
    """
    Build a DMA waveform of n_pulses step pulses on PUL_PIN.
    Each pulse: HIGH for half_period_us, LOW for half_period_us.
    """
    pi.wave_clear()
    pulse_list = []
    for _ in range(n_pulses):
        pulse_list.append(pigpio.pulse(1 << PUL_PIN, 0,            half_period_us))
        pulse_list.append(pigpio.pulse(0,            1 << PUL_PIN, half_period_us))
    pi.wave_add_generic(pulse_list)
    return pi.wave_create()


def send_waveform_blocking(pi, wid):
    """Transmit waveform and block until complete."""
    pi.wave_send_once(wid)
    while pi.wave_tx_busy():
        time.sleep(0.005)
    pi.wave_delete(wid)


# ── NEW: Ramped pulse sender ──────────────────────────────────────────────────

def ramp_pulses_needed(f0: float, f1: float, slope_hz_per_s: float) -> float:
    """
    Pulses needed to ramp from f0 -> f1 with slope a (Hz/s),
    using: pulses = (f1^2 - f0^2) / (2a)
    """
    a = max(float(slope_hz_per_s), 1e-6)
    return (f1 * f1 - f0 * f0) / (2.0 * a)


def solve_peak_frequency(total_pulses: int, f_start: float, accel: float, decel: float) -> float:
    """
    If there isn't enough distance (pulses) to reach TARGET_SPEED_HZ,
    solve triangular profile peak frequency f_peak such that:
      (f_peak^2 - f_start^2)/(2*accel) + (f_peak^2 - f_start^2)/(2*decel) = total_pulses
    """
    a = max(float(accel), 1e-6)
    d = max(float(decel), 1e-6)
    f0_sq = f_start * f_start
    denom = (1.0 / a) + (1.0 / d)
    fpeak_sq = f0_sq + (2.0 * float(total_pulses)) / denom
    return math.sqrt(max(fpeak_sq, f_start * f_start))


def send_pulses_ramped(pi,
                       total_pulses: int,
                       f_start: float,
                       f_target: float,
                       accel_hz_per_s: float,
                       decel_hz_per_s: float,
                       chunk_pulses: int):
    """
    Send total_pulses with an accel/cruise/decel profile.

    - If total_pulses is long enough: trapezoid (ramp up to f_target, cruise, ramp down)
    - If not: triangular (ramp up to f_peak < f_target, then ramp down)

    Ramping math is done in "pulses space" so it's stable and doesn't depend on sleep timing.
    """
    total_pulses = int(total_pulses)
    if total_pulses <= 0:
        return

    f0 = max(float(f_start), 1.0)
    fT = max(float(f_target), f0)
    a  = max(float(accel_hz_per_s), 1e-6)
    d  = max(float(decel_hz_per_s), 1e-6)
    chunk_pulses = max(int(chunk_pulses), 1)

    # Decide trapezoid vs triangular
    pulses_accel = ramp_pulses_needed(f0, fT, a)
    pulses_decel = ramp_pulses_needed(f0, fT, d)

    if pulses_accel + pulses_decel >= total_pulses:
        # Triangular: compute reachable peak
        f_peak = solve_peak_frequency(total_pulses, f0, a, d)
        cruise_pulses = 0
    else:
        # Trapezoid: reach target and cruise
        f_peak = fT
        cruise_pulses = int(round(total_pulses - pulses_accel - pulses_decel))

    # Convert accel/decel pulse budgets into integers for chunk loops
    accel_pulses = int(round(ramp_pulses_needed(f0, f_peak, a)))
    decel_pulses = int(round(ramp_pulses_needed(f0, f_peak, d)))

    # Fix any rounding drift so sum matches exactly
    used = accel_pulses + cruise_pulses + decel_pulses
    if used != total_pulses:
        cruise_pulses += (total_pulses - used)
        cruise_pulses = max(cruise_pulses, 0)

    # ---- Phase 1: Accel ----
    f = f0
    remaining = accel_pulses
    while remaining > 0:
        dp = min(chunk_pulses, remaining)

        half_us = hz_to_half_period_us(f)
        wid = build_pulse_waveform(pi, dp, half_us)
        send_waveform_blocking(pi, wid)

        # Update frequency after dp pulses: f_new = sqrt(f^2 + 2*a*dp)
        f = math.sqrt(max(f * f + 2.0 * a * float(dp), f0 * f0))
        f = min(f, f_peak)
        remaining -= dp

    # ---- Phase 2: Cruise ----
    remaining = cruise_pulses
    if remaining > 0:
        f = f_peak
        while remaining > 0:
            dp = min(chunk_pulses, remaining)
            half_us = hz_to_half_period_us(f)
            wid = build_pulse_waveform(pi, dp, half_us)
            send_waveform_blocking(pi, wid)
            remaining -= dp

    # ---- Phase 3: Decel ----
    remaining = decel_pulses
    f = f_peak
    while remaining > 0:
        dp = min(chunk_pulses, remaining)

        half_us = hz_to_half_period_us(f)
        wid = build_pulse_waveform(pi, dp, half_us)
        send_waveform_blocking(pi, wid)

        # Update frequency after dp pulses: f_new = sqrt(f^2 - 2*d*dp)
        f_sq_new = f * f - 2.0 * d * float(dp)
        f = math.sqrt(max(f_sq_new, f0 * f0))
        remaining -= dp


# ── Driver control ────────────────────────────────────────────────────────────

def init_gpio(pi):
    pi.set_mode(ENA_PIN, pigpio.OUTPUT)
    pi.set_mode(DIR_PIN, pigpio.OUTPUT)
    pi.set_mode(PUL_PIN, pigpio.OUTPUT)
    # Safe initial state
    pi.write(ENA_PIN, 1)   # ENA GPIO LOW → ULN off → opto off → driver DISABLED
    pi.write(DIR_PIN, 0)
    pi.write(PUL_PIN, 0)
    print("[GPIO] Initialized via pigpio. Driver disabled.")


def enable_driver(pi):
    """
    Assert ENA then wait T1 before DIR is touched.
    CL42T sequence chart: ENA must precede DIR by ≥200ms.
    """
    pi.write(ENA_PIN, 0)
    print(f"[GPIO] ENA asserted. Waiting {T1_ENA_DIR_MS}ms before DIR...")
    time.sleep(T1_ENA_DIR_MS / 1000)
    print("[GPIO] Driver ENABLED.")


def disable_driver(pi):
    pi.write(ENA_PIN, 1)
    print("[GPIO] Driver DISABLED.")


def set_direction(pi, forward: bool):
    """
    Set DIR and wait T2 before first pulse.
    CL42T sequence chart: DIR must precede PUL rising edge by ≥2µs.
    """
    pi.write(DIR_PIN, 1 if forward else 0)
    time.sleep(T2_DIR_PUL_US / 1_000_000)


# ── Test run ──────────────────────────────────────────────────────────────────

def run_test():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path  = os.path.join(os.path.expanduser("~"), f"crane_test_{timestamp}.csv")

    print("=" * 60)
    print("  UHplift Bridge Motor 1 — Open Loop Test (pigpio, ramped)")
    print(f"  Pulses/run: {TEST_PULSES}")
    print(f"  Speed: start={START_SPEED_HZ} Hz → target={TARGET_SPEED_HZ} Hz")
    print(f"  Accel: {ACCEL_HZ_PER_S} Hz/s | Decel: {DECEL_HZ_PER_S} Hz/s")
    print(f"  Chunk: {RAMP_CHUNK_PULSES} pulses/waveform")
    print(f"  Log: {log_path}")
    print("=" * 60)

    # ── Connect pigpio daemon (must be running: sudo pigpiod) ──────────────
    pi = pigpio.pi()
    if not pi.connected:
        print("[ERROR] pigpio daemon not running. Run: sudo pigpiod")
        return

    spi = init_spi()

    init_gpio(pi)
    spi_ok = init_encoder(spi)

    if not spi_ok:
        print("[WARNING] Proceeding with motor test, but encoder data will be unreliable.")
        print("          Diagnose SPI before trusting encoder readings.")

    commanded_total = 0

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_s", "run", "direction_fwd",
            "commanded_pulses", "encoder_counts",
            "cmd_position_in", "enc_position_in"
        ])

        try:
            input("\nPress Enter to enable driver and start (Ctrl+C to abort)...")
            enable_driver(pi)

            # ── Pre-run encoder snapshot ──────────────────────────────────
            enc_pre = read_encoder(spi)
            print(f"[INFO] Encoder before Run 1: {enc_pre} counts")

            # ── Run 1: Forward ────────────────────────────────────────────
            print(f"\n[RUN 1] Forward {TEST_PULSES} pulses (ramped)...")
            set_direction(pi, forward=True)

            send_pulses_ramped(
                pi,
                total_pulses=TEST_PULSES,
                f_start=START_SPEED_HZ,
                f_target=TARGET_SPEED_HZ,
                accel_hz_per_s=ACCEL_HZ_PER_S,
                decel_hz_per_s=DECEL_HZ_PER_S,
                chunk_pulses=RAMP_CHUNK_PULSES
            )
            commanded_total += TEST_PULSES

            enc1 = read_encoder(spi)
            pos_cmd = counts_to_inches(commanded_total * COUNTS_PER_REV / PULSES_PER_REV)
            pos_enc = counts_to_inches(enc1)
            print(f"  Commanded: {commanded_total} pulses ({pos_cmd:.4f} in)")
            print(f"  Encoder:   {enc1} counts ({pos_enc:.4f} in)")
            writer.writerow([f"{time.time():.4f}", 1, True,
                             commanded_total, enc1,
                             f"{pos_cmd:.4f}", f"{pos_enc:.4f}"])

            time.sleep(1.0)

            # ── Run 2: Reverse ────────────────────────────────────────────
            print(f"\n[RUN 2] Reverse {TEST_PULSES} pulses (ramped)...")
            set_direction(pi, forward=False)

            send_pulses_ramped(
                pi,
                total_pulses=TEST_PULSES,
                f_start=START_SPEED_HZ,
                f_target=TARGET_SPEED_HZ,
                accel_hz_per_s=ACCEL_HZ_PER_S,
                decel_hz_per_s=DECEL_HZ_PER_S,
                chunk_pulses=RAMP_CHUNK_PULSES
            )
            commanded_total -= TEST_PULSES

            enc2 = read_encoder(spi)
            pos_cmd = counts_to_inches(commanded_total * COUNTS_PER_REV / PULSES_PER_REV)
            pos_enc = counts_to_inches(enc2)
            print(f"  Commanded: {commanded_total} pulses ({pos_cmd:.4f} in)")
            print(f"  Encoder:   {enc2} counts ({pos_enc:.4f} in)")
            writer.writerow([f"{time.time():.4f}", 2, False,
                             commanded_total, enc2,
                             f"{pos_cmd:.4f}", f"{pos_enc:.4f}"])

            net = read_encoder(spi)
            print(f"\n  Net encoder count (target ~0): {net}")
            print(f"[DONE] Log saved to {log_path}")

        except KeyboardInterrupt:
            print("\n[ABORT] Ctrl+C — disabling driver.")

        finally:
            # If pigpiod crashed, these calls can throw. Keep cleanup best-effort.
            try:
                if pi.connected:
                    disable_driver(pi)
                    pi.write(PUL_PIN, 0)
                    pi.write(DIR_PIN, 0)
            except Exception:
                pass
            try:
                pi.stop()
            except Exception:
                pass
            try:
                spi.close()
            except Exception:
                pass
            print("[CLEANUP] pigpio and SPI released.")


if __name__ == "__main__":
    run_test()