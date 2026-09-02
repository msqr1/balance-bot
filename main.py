from math import atan, cos, degrees, hypot, sin
from pathlib import Path
from time import monotonic

import board

from constants import (
    abort_angle,
    complementary_filter_weight,
    enable_motors,
    kd,
    ki,
    kp,
    ks,
    left_multiplier,
    pitch_tau,
    right_multiplier,
    setpoint,
    speed_tau,
    tick_rate,
    velocity_correction,
)
from filtered_mpu6050 import Bandwidth, FilteredMPU6050
from h_bridge_motor import HBridgeMotor
from independent_ema import IndependentEMA
from pid_controller import PIDController

i2c = board.I2C()

mpu = FilteredMPU6050(
    i2c,
    filter_weight=complementary_filter_weight,
    filter_bandwidth=Bandwidth.BAND_5_HZ,
    axes=(2, 0, 1),
    signs=(-1, 1, -1),
    calibration_data=(
        (-9.3741, 10.1954),
        (-9.9044, 9.8148),
        (-9.8029, 10.2050),
    ),
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

pid = PIDController(kp, ki, kd, pitch_tau, ks, setpoint=setpoint)


def main() -> None:
    loggers = (
        lambda: current_time - start_time,  # Time
        lambda: pitch,  # Pitch
        lambda: pid.value_ema.value,  # Filtered Pitch
        lambda: speed,  # Speed
        lambda: (pid.setpoint - pid.value_ema.value - _last_error) / dt,  # Derivative
        lambda: pid.error_integral,  # Integral
        lambda: velocity,  # Velocity
        lambda: pid.setpoint,  # Setpoint
        # lambda: acceleration[0],
        # lambda: acceleration[1],
        # lambda: acceleration[2],
    )
    start_time = monotonic()
    last_time = monotonic()
    last_print = monotonic()
    speed_ema = IndependentEMA(speed_tau)
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
        if abs(pitch) > abort_angle:
            print("Robot fell. Aborting.")
            motor_r.stop()
            motor_l.stop()
            break
        _last_error = pid.last_error
        speed = enable_motors * min(
            max(speed_ema.update(dt, -pid.calculate(dt, pitch)), -1.0), 1.0
        )
        motor_r.move(speed * right_multiplier)
        motor_l.move(speed * left_multiplier)
        if current_time - last_print > 0.2:
            # print(f"Pitch (deg): {pitch:.2f}")
            # print(mpu)
            last_print = current_time
        logged_values.append(tuple(logger() for logger in loggers))


log_path = Path(".log")

logged_values: list[tuple[float, ...]] = []


if __name__ == "__main__":
    try:
        main()
    finally:
        with log_path.open("w") as log_file:
            log_file.writelines(
                ",".join(map(str, values)) + "\n" for values in logged_values
            )
