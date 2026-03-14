#!/usr/bin/env python3
"""
UHplift Crane - Bridge Motor 1 Validated Open Loop Test
Hardware: CL42T + ULN2003AN + AM26LS32A + LS7366R + RPi4
Timing: pigpio DMA waveforms (hardware-precise, OS-independent)
References: CL42T V4.1 manual (sequence chart p.X), LS7366R datasheet
"""

import pigpio
import spidev
import time
import csv
import os
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

# Test parameters
TEST_SPEED_HZ   = 200       # Conservative for first validated run (200 pulses/sec)
HALF_PERIOD_US  = int(1_000_000 / (2 * TEST_SPEED_HZ))  # µs, for pigpio waveform
TEST_PULSES     = PULSES_PER_REV   # 800 pulses = 1 motor shaft revolution

# CL42T sequence timing (from manual sequence chart)
T1_ENA_DIR_MS   = 250       # ENA must precede DIR by ≥200ms — using 250ms margin
T2_DIR_PUL_US   = 10        # DIR must precede first PUL edge by ≥2µs — using 10µs margin
LOG_INTERVAL_S  = 0.1       # Encoder sample period between waveform runs

# ── LS7366R SPI opcodes ───────────────────────────────────────────────────────
# Instruction format: [action(2)][register(3)][000]
CMD_WR_MDR0  = 0x88   # 10 001 000 — Write Mode Register 0
CMD_WR_MDR1  = 0x90   # 10 010 000 — Write Mode Register 1
CMD_CLR_CNTR = 0x20   # 00 100 000 — Clear counter register
CMD_LOAD_OTR = 0xE8   # 11 101 000 — Latch CNTR → OTR (snapshot)
CMD_RD_OTR   = 0x68   # 01 101 000 — Read OTR
CMD_RD_MDR0  = 0x48   # 01 001 000 — Read MDR0 back (verification)

MDR0_X4_FREE = 0x03   # x4 quadrature count, free-running (no index reset yet)
MDR1_4BYTE   = 0x00   # 4-byte counter, counting enabled


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

def build_pulse_waveform(pi, n_pulses):
    """
    Build a DMA waveform of n_pulses step pulses on PUL_PIN.
    Each pulse: HIGH for HALF_PERIOD_US, LOW for HALF_PERIOD_US.
    Pulse width and low width both satisfy CL42T ≥1µs requirement.
    """
    pi.wave_clear()
    pulse_list = []
    for _ in range(n_pulses):
        pulse_list.append(pigpio.pulse(1 << PUL_PIN, 0,          HALF_PERIOD_US))
        pulse_list.append(pigpio.pulse(0,            1 << PUL_PIN, HALF_PERIOD_US))
    pi.wave_add_generic(pulse_list)
    return pi.wave_create()


def send_waveform_blocking(pi, wid):
    """Transmit waveform and block until complete."""
    pi.wave_send_once(wid)
    while pi.wave_tx_busy():
        time.sleep(0.005)
    pi.wave_delete(wid)


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
    print("  UHplift Bridge Motor 1 — Open Loop Test (pigpio)")
    print(f"  Speed: {TEST_SPEED_HZ} Hz | Pulses/run: {TEST_PULSES}")
    print(f"  Half-period: {HALF_PERIOD_US} µs")
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
            print(f"\n[RUN 1] Forward {TEST_PULSES} pulses...")
            set_direction(pi, forward=True)

            wid = build_pulse_waveform(pi, TEST_PULSES)
            send_waveform_blocking(pi, wid)
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
            print(f"\n[RUN 2] Reverse {TEST_PULSES} pulses...")
            set_direction(pi, forward=False)

            wid = build_pulse_waveform(pi, TEST_PULSES)
            send_waveform_blocking(pi, wid)
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
            disable_driver(pi)
            pi.write(PUL_PIN, 0)
            pi.write(DIR_PIN, 0)
            pi.stop()
            spi.close()
            print("[CLEANUP] pigpio and SPI released.")


if __name__ == "__main__":
    run_test()