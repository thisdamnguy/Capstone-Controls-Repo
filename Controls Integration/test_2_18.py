#!/usr/bin/env python3
"""
UHplift Crane - Bridge Motor 1 Open Loop Test
Tests: ENA/DIR/PUL motor control + LS7366R encoder SPI read
GPIO assignments match IO_LIST_AND_PINOUTS.xlsx
"""

import RPi.GPIO as GPIO
import spidev
import time
import csv
import os
from datetime import datetime

# ── Pin assignments (BCM numbering) ──────────────────────────────────────────
ENA_PIN = 17   # Blue  - Driver enable (LOW = enabled via ULN2003)
DIR_PIN = 27   # Green - Direction
PUL_PIN = 22   # Purple - Step pulse
# SPI: CE0=GPIO8(CS), SCLK=GPIO11, MISO=GPIO9, MOSI=GPIO10 (hardware, no define needed)

# ── Motion parameters ─────────────────────────────────────────────────────────
STEPS_PER_REV     = 200        # 1.8 deg motor
MICROSTEP         = 8          # Match CL42T SW switch setting
COUNTS_PER_REV    = 4000       # 1000 PPR encoder x4 quadrature
GEAR_RATIO        = 4.0        # Bridge motor gearbox
WHEEL_RADIUS_IN   = 0.5        # Estimate - update with actual value

TEST_SPEED_HZ     = 400        # Pulse frequency (steps/sec) - start slow
PULSE_HALF_PERIOD = 1.0 / (2 * TEST_SPEED_HZ)
TEST_STEPS        = 400        # Steps per direction run (~1 rev at 8x microstep)
LOG_INTERVAL      = 0.05       # Encoder read every 50ms

# ── LS7366R SPI commands ──────────────────────────────────────────────────────
CMD_WR_MDR0  = 0x88   # Write Mode Register 0
CMD_WR_MDR1  = 0x90   # Write Mode Register 1
CMD_CLR_CNTR = 0x20   # Clear counter
CMD_RD_CNTR  = 0x60   # Read counter (CNTR register)
MDR0_X4_FREE = 0x03   # x4 quadrature, free-running
MDR1_4BYTE   = 0x00   # 4-byte counter width, enable counting


def init_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(ENA_PIN, GPIO.OUT)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(PUL_PIN, GPIO.OUT)
    # Safe initial state: driver disabled, pulse low
    GPIO.output(ENA_PIN, GPIO.HIGH)   # HIGH = disabled through ULN2003
    GPIO.output(DIR_PIN, GPIO.LOW)
    GPIO.output(PUL_PIN, GPIO.LOW)
    print("[GPIO] Initialized. Driver disabled.")


def enable_driver():
    GPIO.output(ENA_PIN, GPIO.LOW)    # LOW = enabled through ULN2003
    time.sleep(0.1)                   # Allow driver to settle
    print("[GPIO] Driver ENABLED.")


def disable_driver():
    GPIO.output(ENA_PIN, GPIO.HIGH)
    print("[GPIO] Driver DISABLED.")


def init_encoder(spi):
    """Configure LS7366R for x4 quadrature, 4-byte counter."""
    spi.xfer2([CMD_WR_MDR0, MDR0_X4_FREE])
    time.sleep(0.001)
    spi.xfer2([CMD_WR_MDR1, MDR1_4BYTE])
    time.sleep(0.001)
    spi.xfer2([CMD_CLR_CNTR])         # Zero the counter
    time.sleep(0.001)
    print("[SPI] LS7366R initialized. Counter zeroed.")


def read_encoder(spi):
    """Read 32-bit counter. Returns signed count."""
    raw = spi.xfer2([CMD_RD_CNTR, 0x00, 0x00, 0x00, 0x00])
    # raw[0] is status byte from CMD, bytes 1-4 are counter
    count = (raw[1] << 24) | (raw[2] << 16) | (raw[3] << 8) | raw[4]
    # Convert to signed 32-bit
    if count >= 0x80000000:
        count -= 0x100000000
    return count


def counts_to_inches(counts):
    """Convert encoder counts to linear travel in inches."""
    shaft_revs = counts / COUNTS_PER_REV
    output_revs = shaft_revs / GEAR_RATIO
    # circumference = 2 * pi * r, but using wheel_radius for rack/pinion or wheel drive
    travel_in = output_revs * 2 * 3.14159 * WHEEL_RADIUS_IN
    return travel_in


def step_motor(steps, direction, spi, log_writer, commanded_total):
    """
    Step motor N steps in given direction with encoder logging.
    direction: GPIO.HIGH or GPIO.LOW
    Returns updated commanded_total
    """
    GPIO.output(DIR_PIN, direction)
    time.sleep(0.000002)   # DIR setup time >= 2us per CL42T spec

    last_log_time = time.time()
    dir_sign = 1 if direction == GPIO.HIGH else -1

    for i in range(steps):
        # Pulse
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(PULSE_HALF_PERIOD)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(PULSE_HALF_PERIOD)

        commanded_total += dir_sign

        # Log at interval
        now = time.time()
        if now - last_log_time >= LOG_INTERVAL:
            enc_count = read_encoder(spi)
            position_in = counts_to_inches(enc_count)
            cmd_pos_in = counts_to_inches(
                commanded_total * (COUNTS_PER_REV / (STEPS_PER_REV * MICROSTEP))
            )
            log_writer.writerow([
                f"{now:.4f}",
                direction == GPIO.HIGH,
                commanded_total,
                enc_count,
                f"{cmd_pos_in:.4f}",
                f"{position_in:.4f}"
            ])
            last_log_time = now

    return commanded_total


def run_test():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = f"/home/pi/crane_test_{timestamp}.csv"

    print("=" * 55)
    print("  UHplift Bridge Motor 1 - Open Loop Test")
    print(f"  Speed: {TEST_SPEED_HZ} Hz | Steps/run: {TEST_STEPS}")
    print(f"  Log: {log_path}")
    print("=" * 55)

    # Init SPI - hardware SPI0, CE0
    spi = spidev.SpiDev()
    spi.open(0, 0)                    # Bus 0, Device 0 (CE0)
    spi.max_speed_hz = 1000000        # 1 MHz - conservative for first test
    spi.mode = 0b00                   # SPI mode 0 (CPOL=0, CPHA=0)

    init_gpio()
    init_encoder(spi)

    commanded_total = 0

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_s",
            "direction_fwd",
            "commanded_steps",
            "encoder_counts",
            "cmd_position_in",
            "enc_position_in"
        ])

        try:
            input("\nPress Enter to enable driver and start test (Ctrl+C to abort)...")
            enable_driver()
            time.sleep(0.5)

            # ── Run 1: Forward ────────────────────────────────────────────
            print(f"\n[RUN 1] Forward {TEST_STEPS} steps...")
            commanded_total = step_motor(
                TEST_STEPS, GPIO.HIGH, spi, writer, commanded_total
            )
            enc = read_encoder(spi)
            print(f"  Commanded: {commanded_total} steps | Encoder: {enc} counts")

            time.sleep(1.0)   # Pause between directions

            # ── Run 2: Reverse ────────────────────────────────────────────
            print(f"\n[RUN 2] Reverse {TEST_STEPS} steps...")
            commanded_total = step_motor(
                TEST_STEPS, GPIO.LOW, spi, writer, commanded_total
            )
            enc = read_encoder(spi)
            print(f"  Commanded: {commanded_total} steps | Encoder: {enc} counts")

            time.sleep(1.0)

            # ── Run 3: Forward again ──────────────────────────────────────
            print(f"\n[RUN 3] Forward {TEST_STEPS * 2} steps...")
            commanded_total = step_motor(
                TEST_STEPS * 2, GPIO.HIGH, spi, writer, commanded_total
            )
            enc = read_encoder(spi)
            print(f"  Commanded: {commanded_total} steps | Encoder: {enc} counts")

            print(f"\n[DONE] Log saved to {log_path}")

        except KeyboardInterrupt:
            print("\n[ABORT] KeyboardInterrupt - disabling driver.")

        finally:
            disable_driver()
            GPIO.cleanup()
            spi.close()
            print("[CLEANUP] GPIO and SPI released.")


if __name__ == "__main__":
    run_test()
