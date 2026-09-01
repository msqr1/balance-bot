from math import degrees
from pathlib import Path
from time import monotonic

import board

from constants import (
    abort_angle,
    complementary_filter_weight,
    kd,
    ki,
    kp,
    ks,
    setpoint,
    tau,
    tick_rate,
)
from filtered_mpu6050 import Bandwidth, FilteredMPU6050
from h_bridge_motor import HBridgeMotor
from pid_controller import PIDController

i2c = board.I2C()

mpu = FilteredMPU6050(
    i2c,
    filter_weight=complementary_filter_weight,
    filter_bandwidth=Bandwidth.BAND_5_HZ,
    axes=(2, 0, 1),
    signs=(-1, 1, -1),
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

pid = PIDController(kp, ki, kd, tau, ks, setpoint=setpoint)


def main() -> None:
    logged_values = {
        "raw_pitch": lambda: mpu.oriented_pitch,
        "filtered_pitch": lambda: pid.value_ema.value,
        "speed": lambda: speed,
    }
    start_time = monotonic()
    last_time = monotonic()
    last_print = monotonic()
    try:
        for name in logged_values:
            with (logs_directory / name).open("w+"):
                pass  # Clear files
        log_files = {name: (logs_directory / name).open("a") for name in logged_values}
        while True:
            current_time = monotonic()
            dt = current_time - last_time
            if dt < 1.0 / tick_rate:
                continue
            if dt > 2.0 / tick_rate:
                print(f"LAG: {dt:.4f}")
            last_time = current_time
            mpu.update(dt)
            pitch = degrees(mpu.oriented_pitch)
            if abs(pitch) > abort_angle:
                print("Robot fell. Aborting.")
                motor_r.stop()
                motor_l.stop()
                break
            speed = -pid.calculate(dt, pitch)
            motor_r.move(speed)
            motor_l.move(speed)
            if monotonic() - last_print > 1.0:
                print(f"Pitch (deg): {pitch:.2f}")
                print(mpu)
                last_print = monotonic()
            elapsed = current_time - start_time
            for name, get_value in logged_values.items():
                log_files[name].write(f"{elapsed},{get_value()}")
    finally:
        for log_file in log_files.values():
            log_file.close()


logs_directory = Path(".logs")
logs_directory.mkdir(exist_ok=True)


if __name__ == "__main__":
    main()
