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
    start_time = monotonic()
    last_time = monotonic()
    last_print = monotonic()
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
        filtered_pitch_log.write(f"{current_time - start_time},{pid.value_ema.value}\n")


filtered_pitch_path = Path("filtered_pitch.txt")


if __name__ == "__main__":
    with filtered_pitch_path.open("w") as filtered_pitch_log:
        pass  # Clear file
    with filtered_pitch_path.open("a") as filtered_pitch_log:
        main()
