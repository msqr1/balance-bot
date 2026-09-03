from collections import defaultdict
from collections.abc import Callable
from math import atan, degrees
from pathlib import Path
from time import monotonic

import board

from constants import (
    abort_angle,
    distance_correction,
    distance_max_error,
    distance_setpoint,
    distance_tau,
    enable_motors,
    kalman_q_angle,
    kalman_q_bias,
    kalman_r_measure,
    kd,
    ki,
    kp,
    ks,
    kv,
    left_multiplier,
    pid_tau,
    ping_interval,
    pitch_rate_tau,
    right_multiplier,
    setpoint,
    speed_tau,
    tick_rate,
    velocity_correction,
    velocity_tau,
)
from filtered_mpu6050 import Bandwidth, FilteredMPU6050
from h_bridge_motor import HBridgeMotor
from hcsr04 import NonblockingHCSR04
from independent_ema import IndependentEMA
from kalman_filter import KalmanFilter
from pid_controller import PIDController

i2c = board.I2C()

digital_filter = KalmanFilter(
    Q_angle=kalman_q_angle, Q_bias=kalman_q_bias, R_measure=kalman_r_measure
)

mpu = FilteredMPU6050(
    i2c,
    digital_filter,
    filter_bandwidth=Bandwidth.BAND_21_HZ,
    axes=(2, 0, 1),
    signs=(-1, 1, -1),
    calibrated_centers=(0.4767, -0.0322, 0.1800),
    calibrated_scales=(1.0012, 0.9939, 0.9802),
    calibrated_gyro_offsets=(-0.050926, 0.021809, 0.007153),
)

sonar = NonblockingHCSR04(
    trigger_pin=board.D6,  # type: ignore[attr-defined]
    echo_pin=board.D5,  # type: ignore[attr-defined]
)


motor_r = HBridgeMotor(  # Motor A
    in1=13,
    in2=19,
    pwm=26,
)
motor_l = HBridgeMotor(  # Motor B
    in1=21,
    in2=20,
    pwm=16,
)

pid = PIDController(kp, ki, kd, ks, pid_tau, setpoint=setpoint)


def main() -> None:
    def get_time() -> float:
        return current_time - start_time

    grouped_loggers: dict[str, Callable[[], float]] = {
        "core": (
            get_time,
            lambda: pitch,  # Pitch
            lambda: pid.value_ema.value,  # Filtered Pitch
            lambda: clamped_speed,  # Speed
            lambda: pid.setpoint,  # Setpoint
        ),
        "contribution": (
            get_time,
            lambda: -speed,  # Speed was inverted after calculating it from gains
            lambda: (pid.setpoint - pid.value_ema.value) * kp,  # kp
            lambda: pid.error_integral * ki,  # ki
            lambda: pitch_rate * kd,  # kd
            # lambda: sign(pid.setpoint - pid.value_ema.value) * ks,  # ks
            lambda: -kv * velocity,  # kv
        ),
        "setpoint": (
            get_time,
            lambda: pid.setpoint,  # Setpoint
            lambda: -velocity * velocity_correction,  # Velocity correction
            lambda: -distance * distance_correction,  # Distance correction
        ),
    }
    speed_ema = IndependentEMA(speed_tau)
    pitch_rate_ema = IndependentEMA(pitch_rate_tau)
    clamped_speed = speed = 0.0
    velocity_ema = IndependentEMA(velocity_tau)

    distance_ema = IndependentEMA(distance_tau, distance_setpoint)

    start_time = last_time = last_print = last_ping = monotonic()
    while True:
        current_time = monotonic()
        dt = current_time - last_time
        if dt < 1.0 / tick_rate:
            continue
        if dt > 1.5 / tick_rate:
            print(f"LAG at {get_time():.4f}s: {dt:.4f} ({dt * tick_rate - 1.0:+.0%})")
        last_time = current_time
        mpu.update(dt)
        pitch_rad = mpu.oriented_pitch
        velocity = velocity_ema.update(dt, speed)

        if current_time - last_ping >= ping_interval:
            sonar.trigger()
            last_ping = current_time
        sonar.update()

        raw_distance = sonar.distance_cm
        distance = (
            distance_ema.update(dt, raw_distance)
            if raw_distance is not None
            else distance_ema.value
        )
        distance_error = min(
            max(distance_setpoint - distance, distance_max_error), -distance_max_error
        )

        pid.setpoint = setpoint - degrees(
            atan(velocity * velocity_correction + distance_error * distance_correction)
        )
        pitch = degrees(pitch_rad)
        pitch_rate = pitch_rate_ema.update(dt, degrees(mpu.oriented_gyro[1]))
        if abs(pitch) > abort_angle:
            print(f"Robot fell at {get_time():.3f}s. Aborting.")
            motor_r.stop()
            motor_l.stop()
            break
        speed = -(pid.calculate(dt, pitch, pitch_rate) - kv * velocity)
        clamped_speed = enable_motors * min(max(speed_ema.update(dt, speed), -1.0), 1.0)
        motor_r.move(clamped_speed * right_multiplier)
        motor_l.move(clamped_speed * left_multiplier)
        if current_time - last_print > 0.2:
            # print(f"Pitch (deg): {pitch:.2f}")
            # print(mpu)
            print(
                f"Setpoint: {pid.setpoint:.3f}, Distance error: {distance_error:.2f} cm"
                f", Raw distance: {distance:.2f} cm"
            )
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
