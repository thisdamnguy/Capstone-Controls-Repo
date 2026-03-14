#!/usr/bin/env python3
"""
UHplift Crane - Bridge Motor 1 Open Loop Test (DEBUG VERSION)
- Adds SPI sanity checks (readback MDR0/MDR1/STR)
- Prints raw SPI bytes
- Verifies CNT_EN status via STR before running motor
- Live encoder monitor mode before/after motion
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
# SPI: CE0=GPIO8(CS), SCLK=GPIO11, MISO=GPIO9, MOSI=GPIO10

# ── Motion parameters ─────────────────────────────────────────────────────────
STEPS_PER_REV     = 200
MICROSTEP         = 4
COUNTS_PER_REV    = 4000
GEAR_RATIO        = 4.0
WHEEL_RADIUS_IN   = 0.5  # update later

TEST_SPEED_HZ     = 400
PULSE_HALF_PERIOD = 1.0 / (2 * TEST_SPEED_HZ)
TEST_STEPS        = 800
LOG_INTERVAL      = 0.05

# ── LS7366R SPI opcodes (per datasheet) ──────────────────────────────────────
CLR_MDR0   = 0x08
CLR_MDR1   = 0x10
CLR_CNTR   = 0x20
CLR_STR    = 0x30

RD_MDR0    = 0x48
RD_MDR1    = 0x50
RD_CNTR    = 0x60
RD_STR     = 0x70

WR_MDR0    = 0x88
WR_MDR1    = 0x90

# MDR0 bits
MDR0_X4_FREE = 0x03  # x4 quadrature (B1:B0=11), free-running (B3:B2=00), index disabled, async index, filter/1
# MDR1 bits
MDR1_4BYTE_ENABLE_COUNT = 0x00  # 4-byte (B1:B0=00) AND counting enabled (B2=0)

# STR bits (for decode)
# STR = [7]CY [6]BW [5]CMP [4]IDX [3]CEN [2]PLS [1]U/D [0]S

def init_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(ENA_PIN, GPIO.OUT)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(PUL_PIN, GPIO.OUT)

    GPIO.output(ENA_PIN, GPIO.HIGH)   # disabled
    GPIO.output(DIR_PIN, GPIO.LOW)
    GPIO.output(PUL_PIN, GPIO.LOW)
    print("[GPIO] Initialized. Driver disabled.")

def enable_driver():
    GPIO.output(ENA_PIN, GPIO.LOW)
    time.sleep(0.1)
    print("[GPIO] Driver ENABLED.")

def disable_driver():
    GPIO.output(ENA_PIN, GPIO.HIGH)
    print("[GPIO] Driver DISABLED.")

def spi_xfer(spi, out_bytes, label=""):
    """Wrapper that prints raw SPI I/O for debugging."""
    resp = spi.xfer2(out_bytes)
    if label:
        print(f"[SPI] {label} TX={out_bytes} RX={resp}")
    return resp

def ls_write_1(spi, opcode, data, label=""):
    spi_xfer(spi, [opcode, data], label=label)

def ls_read_1(spi, opcode, label=""):
    r = spi_xfer(spi, [opcode, 0x00], label=label)
    return r[1]

def ls_clear(spi, opcode, label=""):
    spi_xfer(spi, [opcode], label=label)

def init_encoder(spi):
    # Clear regs first (optional but nice for determinism)
    ls_clear(spi, CLR_MDR0, "CLR_MDR0")
    ls_clear(spi, CLR_MDR1, "CLR_MDR1")
    time.sleep(0.001)

    # Program MDR0/MDR1
    ls_write_1(spi, WR_MDR0, MDR0_X4_FREE, "WR_MDR0 (x4 free)")
    time.sleep(0.001)
    ls_write_1(spi, WR_MDR1, MDR1_4BYTE_ENABLE_COUNT, "WR_MDR1 (4-byte, count EN)")
    time.sleep(0.001)

    # Zero CNTR and clear STR
    ls_clear(spi, CLR_CNTR, "CLR_CNTR")
    time.sleep(0.001)
    ls_clear(spi, CLR_STR, "CLR_STR")
    time.sleep(0.001)

    # Read back to confirm SPI is real
    mdr0 = ls_read_1(spi, RD_MDR0, "RD_MDR0")
    mdr1 = ls_read_1(spi, RD_MDR1, "RD_MDR1")
    strv = ls_read_1(spi, RD_STR,  "RD_STR")

    print(f"[ENC] MDR0 readback: 0x{mdr0:02X} (expected ~0x{MDR0_X4_FREE:02X})")
    print(f"[ENC] MDR1 readback: 0x{mdr1:02X} (expected ~0x{MDR1_4BYTE_ENABLE_COUNT:02X})")
    print(f"[ENC] STR  readback: 0x{strv:02X}")
    print("[ENC] Initialized.")

def read_encoder(spi, verbose=False):
    raw = spi_xfer(spi, [RD_CNTR, 0x00, 0x00, 0x00, 0x00], "RD_CNTR" if verbose else "")
    count = (raw[1] << 24) | (raw[2] << 16) | (raw[3] << 8) | raw[4]
    if count >= 0x80000000:
        count -= 0x100000000
    return count

def read_status(spi):
    s = ls_read_1(spi, RD_STR, "RD_STR")
    return s

def decode_status(s):
    return {
        "CY":  (s >> 7) & 1,
        "BW":  (s >> 6) & 1,
        "CMP": (s >> 5) & 1,
        "IDX": (s >> 4) & 1,
        "CEN": (s >> 3) & 1,  # count enable status
        "PLS": (s >> 2) & 1,  # power loss indicator latch
        "UD":  (s >> 1) & 1,  # up/down
        "S":   (s >> 0) & 1,
    }

def counts_to_inches(counts):
    shaft_revs = counts / COUNTS_PER_REV
    output_revs = shaft_revs / GEAR_RATIO
    return output_revs * 2 * 3.14159 * WHEEL_RADIUS_IN

def live_encoder_monitor(spi, seconds=3.0):
    print(f"\n[MONITOR] Watching encoder for {seconds:.1f}s. (Try moving the carriage/shaft by hand if possible.)")
    t0 = time.time()
    last = None
    while time.time() - t0 < seconds:
        c = read_encoder(spi)
        if c != last:
            s = read_status(spi)
            d = decode_status(s)
            print(f"  CNTR={c:>10d}  STR=0x{s:02X}  CEN={d['CEN']} UD={d['UD']} PLS={d['PLS']}")
            last = c
        time.sleep(0.05)

def step_motor(steps, direction, spi, log_writer, commanded_total):
    GPIO.output(DIR_PIN, direction)
    time.sleep(0.01)

    last_log_time = time.time()
    dir_sign = 1 if direction == GPIO.HIGH else -1

    for _ in range(steps):
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(PULSE_HALF_PERIOD)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(PULSE_HALF_PERIOD)

        commanded_total += dir_sign

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
                f"{position_in:.4f}",
            ])
            last_log_time = now

    return commanded_total

def run_test():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(os.path.expanduser("~"), f"crane_test_{timestamp}.csv")

    print("=" * 60)
    print(" UHplift Bridge Motor 1 - Open Loop Test (ENCODER DEBUG)")
    print(f" Speed: {TEST_SPEED_HZ} Hz | Steps/run: {TEST_STEPS}")
    print(f" Log: {log_path}")
    print("=" * 60)

    spi = spidev.SpiDev()
    spi.open(0, 0)               # CE0
    spi.max_speed_hz = 200000    # slow first for reliability
    spi.mode = 0b00              # mode 0

    init_gpio()
    init_encoder(spi)

    # Sanity: status + counter should be stable and readable
    s = read_status(spi)
    d = decode_status(s)
    print(f"\n[STATUS] STR=0x{s:02X} decoded={d}")
    if d["CEN"] == 0:
        print("[WARN] CEN=0 (counting disabled). This is usually CNT_EN pin LOW or MDR1 B2 set to disable.")
        print("       If wiring is correct, CNT_EN should be HIGH (it has an internal pull-up, but your board may be pulling it down).")

    # Watch encoder before motion
    live_encoder_monitor(spi, seconds=3.0)

    commanded_total = 0
    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_s",
            "direction_fwd",
            "commanded_steps",
            "encoder_counts",
            "cmd_position_in",
            "enc_position_in",
        ])

        try:
            input("\nPress Enter to enable driver and start test (Ctrl+C abort)...")
            enable_driver()
            time.sleep(0.2)

            print(f"\n[RUN 1] Forward {TEST_STEPS} steps...")
            commanded_total = step_motor(TEST_STEPS, GPIO.HIGH, spi, writer, commanded_total)
            enc = read_encoder(spi, verbose=True)
            print(f"  Commanded: {commanded_total} steps | Encoder: {enc} counts")

            time.sleep(0.5)
            live_encoder_monitor(spi, seconds=2.0)

            print(f"\n[RUN 2] Reverse {TEST_STEPS} steps...")
            commanded_total = step_motor(TEST_STEPS, GPIO.LOW, spi, writer, commanded_total)
            enc = read_encoder(spi, verbose=True)
            print(f"  Commanded: {commanded_total} steps | Encoder: {enc} counts")

            net = read_encoder(spi, verbose=True)
            print(f"\n  Net encoder count (should be ~0 if perfect): {net}")
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
