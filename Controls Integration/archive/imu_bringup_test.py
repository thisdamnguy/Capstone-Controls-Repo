#Before running, enable SPI1 and reboot:

#echo "dtoverlay=spi1-3cs" | sudo tee -a /boot/config.txt
#sudo reboot

#Then verify:
#ls /dev/spidev1*
# should show /dev/spidev1.0

!/usr/bin/env python3
"""
UHplift - LSM6DS3 IMU Bring-up Test
SPI1, Mode 3
Pi Pin 17  → IMU VCC (3.3V)
Pi Pin 39  → IMU GND
GPIO18     → IMU DCS  (CS)
GPIO19     → IMU SAO  (MISO)
GPIO20     → IMU SDX  (MOSI)
GPIO21     → IMU SCX  (SCLK)
"""

import spidev
import time
import math

# SPI1, CE0 = GPIO18
SPI_BUS     = 1
SPI_DEVICE  = 0
SPI_SPEED   = 1_000_000   # 1 MHz conservative for bring-up
SPI_MODE    = 0b11        # Mode 3: CPOL=1, CPHA=1

# Register addresses
WHO_AM_I    = 0x0F
CTRL1_XL    = 0x10
CTRL2_G     = 0x11
CTRL3_C     = 0x12
OUTX_L_G    = 0x22   # Gyro output start (6 bytes)
OUTX_L_A    = 0x28   # Accel output start (6 bytes)

# Read bit
READ_BIT    = 0x80

# Sensitivity
# Accel: ±4g @ 0.122 mg/LSB
ACCEL_SENS  = 0.122 / 1000.0
# Gyro: ±500 dps @ 17.5 mdps/LSB
GYRO_SENS   = 17.5  / 1000.0


def spi_read(spi, reg, length=1):
    """Read one or more registers"""
    if length == 1:
        result = spi.xfer2([reg | READ_BIT, 0x00])
        return result[1]
    else:
        # Burst read with auto-increment (bit 6 set)
        cmd = [reg | READ_BIT | 0x40] + [0x00] * length
        result = spi.xfer2(cmd)
        return result[1:]


def spi_write(spi, reg, value):
    """Write single register"""
    spi.xfer2([reg & ~READ_BIT, value])


def bytes_to_int16(low, high):
    """Convert two bytes to signed 16-bit integer (little-endian)"""
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value


def init_imu(spi):
    """
    Initialize LSM6DS3 per control philosophy doc.
    Returns True if WHO_AM_I confirmed.
    """
    # Step 1: Verify communication
    who = spi_read(spi, WHO_AM_I)
    print(f"[IMU] WHO_AM_I: {hex(who)} (expect 0x69)")
    if who != 0x69:
        print("[IMU] ERROR: unexpected WHO_AM_I.")
        print("      Check: SPI1 enabled, Mode 3, wiring on GPIO18-21.")
        return False
    print("[IMU] Communication confirmed.")

    # Step 2: Configure accelerometer — 208Hz, ±4g
    spi_write(spi, CTRL1_XL, 0x58)
    time.sleep(0.001)

    # Step 3: Configure gyroscope — 208Hz, ±500dps
    spi_write(spi, CTRL2_G, 0x54)
    time.sleep(0.001)

    # Step 4: BDU=1, auto-increment=1, 4-wire SPI
    spi_write(spi, CTRL3_C, 0x44)
    time.sleep(0.001)

    # Step 5: Readback verification
    xl = spi_read(spi, CTRL1_XL)
    g  = spi_read(spi, CTRL2_G)
    c  = spi_read(spi, CTRL3_C)
    print(f"[IMU] CTRL1_XL readback: {hex(xl)} (expect 0x58)")
    print(f"[IMU] CTRL2_G  readback: {hex(g)}  (expect 0x54)")
    print(f"[IMU] CTRL3_C  readback: {hex(c)}  (expect 0x44)")

    if xl == 0x58 and g == 0x54:
        print("[IMU] Init confirmed. Ready to stream.")
        return True
    else:
        print("[IMU] WARNING: readback mismatch. Check configuration.")
        return False


def read_imu(spi):
    """
    Burst read gyro + accel (12 bytes from OUTX_L_G).
    Returns (ax, ay, az) in g and (gx, gy, gz) in deg/s.
    """
    raw = spi_read(spi, OUTX_L_G, 12)

    gx = bytes_to_int16(raw[0],  raw[1])  * GYRO_SENS
    gy = bytes_to_int16(raw[2],  raw[3])  * GYRO_SENS
    gz = bytes_to_int16(raw[4],  raw[5])  * GYRO_SENS
    ax = bytes_to_int16(raw[6],  raw[7])  * ACCEL_SENS
    ay = bytes_to_int16(raw[8],  raw[9])  * ACCEL_SENS
    az = bytes_to_int16(raw[10], raw[11]) * ACCEL_SENS

    return (ax, ay, az), (gx, gy, gz)


def compute_angle(ax, ay, az):
    """
    Estimate sway angles from accelerometer.
    Flat on table: ax~0, ay~0, az~1g, θx~0°, θy~0°
    """
    theta_x = math.degrees(math.atan2(ax, az))
    theta_y = math.degrees(math.atan2(ay, az))
    return theta_x, theta_y


def run_test():
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = SPI_MODE

    print("=" * 60)
    print("  UHplift LSM6DS3 IMU Bring-up Test (SPI1)")
    print("  CS=GPIO18 | MISO=GPIO19 | MOSI=GPIO20 | SCLK=GPIO21")
    print("=" * 60)

    ok = init_imu(spi)
    if not ok:
        spi.close()
        return

    print("\n[IMU] Streaming at ~100Hz. Ctrl+C to stop.")
    print(f"{'Time':>8} | {'ax':>7} {'ay':>7} {'az':>7} | "
          f"{'gx':>8} {'gy':>8} {'gz':>8} | "
          f"{'θx':>7} {'θy':>7}")
    print("-" * 80)

    t_start = time.time()
    try:
        while True:
            (ax, ay, az), (gx, gy, gz) = read_imu(spi)
            theta_x, theta_y = compute_angle(ax, ay, az)
            t = time.time() - t_start

            print(f"{t:8.2f} | "
                  f"{ax:7.3f} {ay:7.3f} {az:7.3f} | "
                  f"{gx:8.2f} {gy:8.2f} {gz:8.2f} | "
                  f"{theta_x:7.2f} {theta_y:7.2f}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[IMU] Stopped.")
    finally:
        spi.close()
        print("[CLEANUP] SPI1 released.")


if __name__ == "__main__":
    run_test()