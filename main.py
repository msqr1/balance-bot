from collections import defaultdict
from math import atan, cos, degrees, hypot, sin
from pathlib import Path
from time import monotonic

import board

from constants import (
    abort_angle,
    enable_motors,
    kalman_r_measure,
    kd,
    ki,
    kp,
    ks,
    left_multiplier,
    pid_tau,
    pitch_rate_tau,
    right_multiplier,
    setpoint,
    speed_tau,
    tick_rate,
    velocity_correction,
)
from filtered_mpu6050 import Bandwidth, FilteredMPU6050
from h_bridge_motor import HBridgeMotor
from independent_ema import IndependentEMA
from kalman_filter import KalmanFilter
from pid_controller import PIDController, sign

i2c = board.I2C()

digital_filter = KalmanFilter(R_measure=kalman_r_measure)

mpu = FilteredMPU6050(
    i2c,
    digital_filter,
    filter_bandwidth=Bandwidth.BAND_21_HZ,
    axes=(2, 0, 1),
    signs=(-1, 1, -1),
    calibrated_centers=(0.4767, -0.0322, 0.1800),
    calibrated_scales=(1.0012, 0.9939, 0.9802),
)


motor_r = HBridgeMotor(  # Motor A
    in1=25,
    in2=18,
    pwm=23,
)
motor_l = HBridgeMotor(  # Motor B
    in1=12,
    in2=16,
    pwm=24,
)

pid = PIDController(kp, ki, kd, ks, pid_tau, setpoint=setpoint)


def main() -> None:
    def get_time() -> float:
        return current_time - start_time

    grouped_loggers = {
        "core": (
            get_time,
            lambda: pitch,  # Pitch
            lambda: pid.value_ema.value,  # Filtered Pitch
            lambda: clamped_speed,  # Speed
            lambda: pid.setpoint,  # Setpoint
        ),
        "contribution": (
            get_time,
            lambda: speed,
            lambda: (pid.setpoint - pid.value_ema.value) * kp,  # kp
            lambda: pid.error_integral * ki,  # ki
            lambda: pitch_rate * kd,  # kd
            lambda: sign(pid.setpoint - pid.value_ema.value) * ks,  # ks
        ),
        "acceleration": (
            get_time,
            lambda: velocity,
            lambda: acceleration[0],
            lambda: acceleration[1],
            lambda: acceleration[2],
        ),
    }
    start_time = last_time = last_print = monotonic()
    speed_ema = IndependentEMA(speed_tau)
    pitch_rate_ema = IndependentEMA(pitch_rate_tau)
    velocity = 0.0
    while True:
        current_time = monotonic()
        dt = current_time - last_time
        if dt < 1.0 / tick_rate:
            continue
        if dt > 2.0 / tick_rate:
            print(f"LAG: {dt:.4f}")
        last_time = current_time
        mpu.update(dt)
        pitch_rad = mpu.oriented_pitch
        try:
            acceleration = mpu.oriented_acceleration
        except OSError:
            acceleration = 0.0, 0.0, 0.0
        else:
            acceleration_linear = acceleration[0] * cos(pitch_rad) + hypot(
                0.0 * acceleration[1], acceleration[2]
            ) * sin(pitch_rad)
            velocity += acceleration_linear * dt
        pid.setpoint = setpoint - atan(velocity * velocity_correction)
        pitch = degrees(pitch_rad)
        pitch_rate = pitch_rate_ema.update(dt, degrees(mpu.oriented_gyro[1]))
        if abs(pitch) > abort_angle:
            print(f"Robot fell at {get_time():.3f}s. Aborting.")
            motor_r.stop()
            motor_l.stop()
            break
        speed = -pid.calculate(dt, pitch, pitch_rate)
        clamped_speed = enable_motors * min(max(speed_ema.update(dt, speed), -1.0), 1.0)
        motor_r.move(clamped_speed * right_multiplier)
        motor_l.move(clamped_speed * left_multiplier)
        if current_time - last_print > 0.2:
            # print(f"Pitch (deg): {pitch:.2f}")
            # print(mpu)
            last_print = current_time
        for group, loggers in grouped_loggers.items():
            logged_values[group].append(tuple(logger() for logger in loggers))


logs_directory = Path(".logs")
logs_directory.mkdir(exist_ok=True)

logged_values: defaultdict[str, list[tuple[float, ...]]] = defaultdict(list)


if __name__ == "__main__":
    try:
        main()
    finally:
        for group, values in logged_values.items():
            with (logs_directory / f"{group}.txt").open("w") as log_file:
                log_file.writelines(
                    ",".join(f"{value:.10f}" for value in row) + "\n" for row in values
                )
