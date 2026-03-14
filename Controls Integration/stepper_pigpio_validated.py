#!/usr/bin/env python3
"""
UHplift Crane - Bridge Motor 1 Validated Open Loop Test (with auto-tuned ramp)
Hardware: CL42T + ULN2003AN + AM26LS32A + LS7366R + RPi4
Timing: pigpio DMA waveforms (hardware-precise, OS-independent)

Update:
- User changes ONLY TARGET_SPEED_HZ (and optionally a few "policy" constants).
- START_SPEED_HZ, ACCEL_HZ_PER_S, DECEL_HZ_PER_S auto-compute from TARGET_SPEED_HZ.
"""

import pigpio
import spidev
import time
import csv
import os
import math
from datetime import datetime

# ── Pin assignments (BCM) ─────────────────────────────────────────────────────
ENA_PIN = 17
DIR_PIN = 27
PUL_PIN = 22

# ── Motion parameters ─────────────────────────────────────────────────────────
STEPS_PER_REV   = 200
MICROSTEP       = 4
PULSES_PER_REV  = STEPS_PER_REV * MICROSTEP      # 800 pulses/rev at 4x microstep
COUNTS_PER_REV  = 4000                           # 1000 PPR encoder × 4 quadrature
GEAR_RATIO      = 4.0
WHEEL_RADIUS_IN = 0.5                            # Placeholder — measure actual wheel

# ── USER INPUT: change ONLY this for speed tests ──────────────────────────────
TARGET_SPEED_HZ = 4000.0   # pulses/sec (step frequency)

# ── Auto-ramp "policy" knobs (usually leave these alone) ──────────────────────
START_RATIO          = 0.10   # start speed = START_RATIO * target (typical 0.05–0.20)
START_MIN_HZ         = 80.0   # never start below this (avoid ultra-slow stutter)
START_MAX_HZ         = 400.0  # never start above this (avoid harsh "kick" start)

ACCEL_SCALE          = 3.0    # accel (Hz/s) = ACCEL_SCALE * target (typical 2–6)
ACCEL_MIN_HZ_PER_S   = 300.0  # safety floor
ACCEL_MAX_HZ_PER_S   = 12000.0# safety cap (protect against silly settings)

DECEL_SCALE          = 3.0    # decel (Hz/s) = DECEL_SCALE * target
DECEL_MIN_HZ_PER_S   = 300.0
DECEL_MAX_HZ_PER_S   = 12000.0

# Waveform chunk sizing (smoothness vs overhead)
RAMP_CHUNK_PULSES = 200       # lower = smoother ramps, higher = fewer wave builds
MIN_HALF_PERIOD_US = 2        # safety clamp (>=1us required, keep margin)

# Test parameters
TEST_PULSES = int(PULSES_PER_REV * GEAR_RATIO)   # pulses for ~1 output rev (per your assumption)

# CL42T sequence timing
T1_ENA_DIR_MS  = 250
T2_DIR_PUL_US  = 10

# ── LS7366R SPI opcodes ───────────────────────────────────────────────────────
CMD_WR_MDR0  = 0x88
CMD_WR_MDR1  = 0x90
CMD_CLR_CNTR = 0x20
CMD_LOAD_OTR = 0xE8
CMD_RD_OTR   = 0x68
CMD_RD_MDR0  = 0x48

MDR0_X4_FREE = 0x03
MDR1_4BYTE   = 0x00


# ── Auto-tuned ramp parameters (computed from TARGET_SPEED_HZ) ────────────────
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def compute_ramp_params(target_hz: float):
    """
    Compute (start_hz, accel_hz_per_s, decel_hz_per_s) from target_hz using simple rules:
      - start_hz = clamp(target * START_RATIO, START_MIN_HZ, START_MAX_HZ)
      - accel = clamp(target * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)
      - decel = clamp(target * DECEL_SCALE, DECEL_MIN, DECEL_MAX)
    """
    t = max(float(target_hz), 1.0)
    start_hz = clamp(t * START_RATIO, START_MIN_HZ, START_MAX_HZ)
    accel = clamp(t * ACCEL_SCALE, ACCEL_MIN_HZ_PER_S, ACCEL_MAX_HZ_PER_S)
    decel = clamp(t * DECEL_SCALE, DECEL_MIN_HZ_PER_S, DECEL_MAX_HZ_PER_S)

    # Ensure target >= start
    if t < start_hz:
        start_hz = max(1.0, t * 0.5)

    return start_hz, accel, decel


START_SPEED_HZ, ACCEL_HZ_PER_S, DECEL_HZ_PER_S = compute_ramp_params(TARGET_SPEED_HZ)


# ── SPI / encoder ─────────────────────────────────────────────────────────────
def init_spi():
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 500_000
    spi.mode = 0b00
    return spi


def init_encoder(spi):
    spi.xfer2([CMD_WR_MDR0, MDR0_X4_FREE])
    time.sleep(0.001)
    spi.xfer2([CMD_WR_MDR1, MDR1_4BYTE])
    time.sleep(0.001)
    spi.xfer2([CMD_CLR_CNTR])
    time.sleep(0.001)

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
    spi.xfer2([CMD_LOAD_OTR])
    time.sleep(0.0001)
    raw = spi.xfer2([CMD_RD_OTR, 0x00, 0x00, 0x00, 0x00])
    count = (raw[1] << 24) | (raw[2] << 16) | (raw[3] << 8) | raw[4]
    if count >= 0x80000000:
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
    pi.wave_clear()
    pulse_list = []
    for _ in range(n_pulses):
        pulse_list.append(pigpio.pulse(1 << PUL_PIN, 0,            half_period_us))
        pulse_list.append(pigpio.pulse(0,            1 << PUL_PIN, half_period_us))
    pi.wave_add_generic(pulse_list)
    return pi.wave_create()


def send_waveform_blocking(pi, wid):
    pi.wave_send_once(wid)
    while pi.wave_tx_busy():
        time.sleep(0.005)
    pi.wave_delete(wid)


# ── Ramped pulse sender (trapezoid/triangular) ────────────────────────────────
def ramp_pulses_needed(f0: float, f1: float, slope_hz_per_s: float) -> float:
    a = max(float(slope_hz_per_s), 1e-6)
    return (f1 * f1 - f0 * f0) / (2.0 * a)


def solve_peak_frequency(total_pulses: int, f_start: float, accel: float, decel: float) -> float:
    a = max(float(accel), 1e-6)
    d = max(float(decel), 1e-6)
    f0_sq = f_start * f_start
    denom = (1.0 / a) + (1.0 / d)
    fpeak_sq = f0_sq + (2.0 * float(total_pulses)) / denom
    return math.sqrt(max(fpeak_sq, f0_sq))


def send_pulses_ramped(pi,
                       total_pulses: int,
                       f_start: float,
                       f_target: float,
                       accel_hz_per_s: float,
                       decel_hz_per_s: float,
                       chunk_pulses: int):
    total_pulses = int(total_pulses)
    if total_pulses <= 0:
        return

    f0 = max(float(f_start), 1.0)
    fT = max(float(f_target), f0)
    a  = max(float(accel_hz_per_s), 1e-6)
    d  = max(float(decel_hz_per_s), 1e-6)
    chunk_pulses = max(int(chunk_pulses), 1)

    pulses_accel = ramp_pulses_needed(f0, fT, a)
    pulses_decel = ramp_pulses_needed(f0, fT, d)

    if pulses_accel + pulses_decel >= total_pulses:
        # Triangular: can't reach target; compute peak
        f_peak = solve_peak_frequency(total_pulses, f0, a, d)
        cruise_pulses = 0
    else:
        f_peak = fT
        cruise_pulses = int(round(total_pulses - pulses_accel - pulses_decel))

    accel_pulses = int(round(ramp_pulses_needed(f0, f_peak, a)))
    decel_pulses = int(round(ramp_pulses_needed(f0, f_peak, d)))

    used = accel_pulses + cruise_pulses + decel_pulses
    if used != total_pulses:
        cruise_pulses += (total_pulses - used)
        cruise_pulses = max(cruise_pulses, 0)

    # Accel phase
    f = f0
    remaining = accel_pulses
    while remaining > 0:
        dp = min(chunk_pulses, remaining)
        half_us = hz_to_half_period_us(f)
        wid = build_pulse_waveform(pi, dp, half_us)
        send_waveform_blocking(pi, wid)

        f = math.sqrt(max(f * f + 2.0 * a * float(dp), f0 * f0))
        f = min(f, f_peak)
        remaining -= dp

    # Cruise phase
    remaining = cruise_pulses
    if remaining > 0:
        f = f_peak
        while remaining > 0:
            dp = min(chunk_pulses, remaining)
            half_us = hz_to_half_period_us(f)
            wid = build_pulse_waveform(pi, dp, half_us)
            send_waveform_blocking(pi, wid)
            remaining -= dp

    # Decel phase
    remaining = decel_pulses
    f = f_peak
    while remaining > 0:
        dp = min(chunk_pulses, remaining)
        half_us = hz_to_half_period_us(f)
        wid = build_pulse_waveform(pi, dp, half_us)
        send_waveform_blocking(pi, wid)

        f_sq_new = f * f - 2.0 * d * float(dp)
        f = math.sqrt(max(f_sq_new, f0 * f0))
        remaining -= dp


# ── Driver control ────────────────────────────────────────────────────────────
def init_gpio(pi):
    pi.set_mode(ENA_PIN, pigpio.OUTPUT)
    pi.set_mode(DIR_PIN, pigpio.OUTPUT)
    pi.set_mode(PUL_PIN, pigpio.OUTPUT)

    pi.write(ENA_PIN, 1)  # disabled (per your ULN logic)
    pi.write(DIR_PIN, 0)
    pi.write(PUL_PIN, 0)
    print("[GPIO] Initialized via pigpio. Driver disabled.")


def enable_driver(pi):
    pi.write(ENA_PIN, 0)
    print(f"[GPIO] ENA asserted. Waiting {T1_ENA_DIR_MS}ms before DIR...")
    time.sleep(T1_ENA_DIR_MS / 1000)
    print("[GPIO] Driver ENABLED.")


def disable_driver(pi):
    pi.write(ENA_PIN, 1)
    print("[GPIO] Driver DISABLED.")


def set_direction(pi, forward: bool):
    pi.write(DIR_PIN, 1 if forward else 0)
    time.sleep(T2_DIR_PUL_US / 1_000_000)


# ── Test run ──────────────────────────────────────────────────────────────────
def run_test():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path  = os.path.join(os.path.expanduser("~"), f"crane_test_{timestamp}.csv")

    # Recompute at runtime (so if user edits TARGET, everything updates)
    start_hz, accel, decel = compute_ramp_params(TARGET_SPEED_HZ)

    print("=" * 72)
    print("  UHplift Bridge Motor 1 — Open Loop Test (pigpio, auto-ramped)")
    print(f"  Pulses/run: {TEST_PULSES}")
    print(f"  TARGET_SPEED_HZ: {TARGET_SPEED_HZ:.1f} Hz")
    print(f"  Auto ramp: START={start_hz:.1f} Hz | ACCEL={accel:.1f} Hz/s | DECEL={decel:.1f} Hz/s")
    print(f"  Policy knobs: START_RATIO={START_RATIO}, ACCEL_SCALE={ACCEL_SCALE}, DECEL_SCALE={DECEL_SCALE}")
    print(f"  Chunk: {RAMP_CHUNK_PULSES} pulses/waveform")
    print(f"  Log: {log_path}")
    print("=" * 72)

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

            enc_pre = read_encoder(spi)
            print(f"[INFO] Encoder before Run 1: {enc_pre} counts")

            # Run 1: Forward
            print(f"\n[RUN 1] Forward {TEST_PULSES} pulses (auto-ramped)...")
            set_direction(pi, forward=True)

            send_pulses_ramped(
                pi,
                total_pulses=TEST_PULSES,
                f_start=start_hz,
                f_target=TARGET_SPEED_HZ,
                accel_hz_per_s=accel,
                decel_hz_per_s=decel,
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

            # Run 2: Reverse
            print(f"\n[RUN 2] Reverse {TEST_PULSES} pulses (auto-ramped)...")
            set_direction(pi, forward=False)

            send_pulses_ramped(
                pi,
                total_pulses=TEST_PULSES,
                f_start=start_hz,
                f_target=TARGET_SPEED_HZ,
                accel_hz_per_s=accel,
                decel_hz_per_s=decel,
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