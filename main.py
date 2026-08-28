from math import radians
from time import monotonic

import board

from filtered_mpu6050 import FilteredMPU6050
from h_bridge_motor import HBridgeMotor
from pid_controller import PIDController

i2c = board.I2C()

filter_weight = 0.95
mpu = FilteredMPU6050(
    i2c,
    filter_weight=filter_weight,
    axes=(2, 0, 1),
    signs=(-1, 1, -1),
)

motor = HBridgeMotor(
    in1=12,
    in2=23,
    pwm=16,
)

kp = 0.0
ki = 0.0
kd = 0.0

pid = PIDController(kp, ki, kd)


tick_rate = 60.0

abort_angle = radians(45.0)


def main() -> None:
    last_time = monotonic()
    while True:
        current_time = monotonic()
        dt = current_time - last_time
        if dt < 1.0 / tick_rate:
            continue
        last_time = current_time
        mpu.update(dt)
        pitch = mpu.pitch
        if abs(pitch) > abort_angle:
            print("Robot fell. Aborting.")
            motor.stop()
            break
        speed = pid.calculate(dt, pitch)
        motor.move(speed)


if __name__ == "__main__":
    main()
