import time

import board
from adafruit_mpu6050 import MPU6050, STANDARD_GRAVITY

i2c = board.I2C()
mpu = MPU6050(i2c)


# Storage for max/min values (+1g / -1g)
peak_data: dict[str, float] = {}

# Storage for zero-readings (4 per axis across the 6 orientations)
zero_readings: dict[str, list[float]] = {"x": [], "y": [], "z": []}


def wait_for_user(prompt: str, samples: int = 100) -> tuple[float, float, float]:
    print(f"\n[ACTION] {prompt}")
    print("Press ENTER when steady...")
    input()
    print(f"Sampling {samples} points... Keep it completely still.")
    sum_x = sum_y = sum_z = 0.0
    i = 0
    while i < samples:
        try:
            x, y, z = mpu.acceleration
        except OSError as e:
            print(f"Ignoring: {e}")
            continue
        sum_x += x
        sum_y += y
        sum_z += z
        i += 1
        time.sleep(0.01)
    return sum_x / samples, sum_y / samples, sum_z / samples


print("=== Enhanced MPU6050 Calibration Routine ===")
print("Keep the sensor level and still for each of the 6 orientations.\n")

ax, ay, az = wait_for_user("Place sensor FLAT and FACE UP (Z pointing UP)")
peak_data["z_max"] = az
zero_readings["x"].append(ax)
zero_readings["y"].append(ay)

ax, ay, az = wait_for_user("Place sensor FLAT and FACE DOWN (Z pointing DOWN)")
peak_data["z_min"] = az
zero_readings["x"].append(ax)
zero_readings["y"].append(ay)

ax, ay, az = wait_for_user("Stand sensor on edge (X pointing UP)")
peak_data["x_max"] = ax
zero_readings["y"].append(ay)
zero_readings["z"].append(az)

ax, ay, az = wait_for_user("Stand sensor on edge (X pointing DOWN)")
peak_data["x_min"] = ax
zero_readings["y"].append(ay)
zero_readings["z"].append(az)

ax, ay, az = wait_for_user("Stand sensor on edge (Y pointing UP)")
peak_data["y_max"] = ay
zero_readings["x"].append(ax)
zero_readings["z"].append(az)

ax, ay, az = wait_for_user("Stand sensor on edge (Y pointing DOWN)")
peak_data["y_min"] = ay
zero_readings["x"].append(ax)
zero_readings["z"].append(az)

print("\n=======================================================")
print("=== CALIBRATION RESULTS & ZERO ANALYSIS ===")
print("=======================================================\n")

calibrated_centers: list[float] = []
calibrated_scales: list[float] = []

for axis in ("x", "y", "z"):
    v_min = peak_data[f"{axis}_min"]
    v_max = peak_data[f"{axis}_max"]

    # Midpoint derived purely from +1g and -1g span
    span_center = (v_min + v_max) / 2.0
    # Average of the 4 independent perpendicular zero-gravity poses
    zeroes = zero_readings[axis]
    zero_avg = sum(zeroes) / len(zeroes)
    # Combined center: weighted between span-midpoint and orthogonal zeros
    refined_center = span_center * 0.5 + zero_avg * 0.5

    scale = (2.0 * STANDARD_GRAVITY) / (v_max - v_min)

    calibrated_centers.append(refined_center)
    calibrated_scales.append(scale)

    print(f"--- Axis {axis.upper()} ---")
    print(f"  +1g peak: {v_max:8.4f} m/s² | -1g peak: {v_min:8.4f} m/s²")
    print(f"  Span Center [(max+min)/2]: {span_center:8.4f} m/s²")
    print(f"  Orthogonal Zeros: {[round(z, 4) for z in zeroes]}")
    print(f"  Average Zero:              {zero_avg:8.4f} m/s²")
    print(f"  Final Refined Center:      {refined_center:8.4f} m/s²")
    print(f"  Scale Factor:              {scale:8.4f}\n")

print("=======================================================")
print(f"calibrated_centers={tuple(calibrated_centers)}")
print(f"calibrated_scales={tuple(calibrated_scales)}")
