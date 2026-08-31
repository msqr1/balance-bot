from math import radians
from time import monotonic

import board

from filtered_mpu6050 import Bandwidth, FilteredMPU6050
from h_bridge_motor import HBridgeMotor
from pid_controller import PIDController

i2c = board.I2C()

filter_weight = 0.95
mpu = FilteredMPU6050(
    i2c,
    filter_weight=filter_weight,
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

kp = 1.0
ki = 0.0
kd = 0.0
kd_tau = 0.1
feedforward = 0.0

pid = PIDController(kp, ki, kd, kd_tau, feedforward, setpoint=0.0)


tick_rate = 60.0

abort_angle = radians(45.0)


def main() -> None:
    last_time = monotonic()
    last_print = monotonic()
    while True:
        current_time = monotonic()
        dt = current_time - last_time
        if dt < 1.0 / tick_rate:
            continue
        last_time = current_time
        mpu.update(dt)
        pitch = mpu.oriented_pitch
        if abs(pitch) > abort_angle:
            print("Robot fell. Aborting.")
            motor_r.stop()
            motor_l.stop()
            break
        speed = -pid.calculate(dt, pitch)
        motor_r.move(speed)
        motor_l.move(speed)
        if monotonic() - last_print > 1.0:
            print(mpu)
            last_print = monotonic()


if __name__ == "__main__":
    main()
