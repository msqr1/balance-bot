import time
from math import degrees

import adafruit_mpu6050
import board

samples = 1000

i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

print("=== MPU6050 Gyroscope Calibration ===")
print("1. Place the sensor flat on a completely stable surface.")
print("2. Do NOT touch, move, or vibrate the sensor or table during testing.")
print("\nPress ENTER when ready to begin...")
input()

print(f"{samples} samples")
print("Calibrating... Keep perfectly still.")

# Storage variables for total accumulation
sum_gyro_x = sum_gyro_y = sum_gyro_z = 0.0

# Collect data points
i = 0
while i < samples:
    # mpu.gyro outputs readings in Radians per second
    try:
        gx, gy, gz = mpu.gyro
    except OSError as e:
        print(f"Ignoring: {e}")
        continue
    sum_gyro_x += gx
    sum_gyro_y += gy
    sum_gyro_z += gz

    # Visual feedback every 50 samples
    if (i + 1) % 50 == 0:
        print(f"  Progress: {((i + 1) / samples):.0%} Done...")

    time.sleep(0.01)  # 10ms delay between samples
    i += 1

# Calculate final averages (this is your zero-rate bias offset)
bias_x = sum_gyro_x / samples
bias_y = sum_gyro_y / samples
bias_z = sum_gyro_z / samples

print("\n=======================================================")
print("=== GYRO CALIBRATION COMPLETE ===")
print("=======================================================\n")

print("Calculated Bias Offsets (in Radians/sec):")
print(f"  X-Axis Bias: {bias_x:+.6f} rad/s ({degrees(bias_x):+.4f}°/s)")
print(f"  Y-Axis Bias: {bias_y:+.6f} rad/s ({degrees(bias_y):+.4f}°/s)")
print(f"  Z-Axis Bias: {bias_z:+.6f} rad/s ({degrees(bias_z):+.4f}°/s)")

print("\n=======================================================")
print(f"calibrated_gyro=({bias_x:.6f}, {bias_y:.6f}, {bias_z:.6f})")
