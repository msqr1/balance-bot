from time import sleep

import adafruit_mpu6050
import board

i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

while True:
    acceleration = mpu.acceleration
    gyro = mpu.gyro
    temperature = mpu.temperature
    print(f"Accel X: {acceleration[0]:.2f}")
    print(f"Accel Y: {acceleration[1]:.2f}")
    print(f"Accel Z: {acceleration[2]:.2f}")
    print(f"Gyro X:  {gyro[0]:.2f}")
    print(f"Gyro Y:  {gyro[1]:.2f}")
    print(f"Gyro Z:  {gyro[2]:.2f}")
    print(f"Temp C:  {temperature:.2f}")
    print()
    sleep(1)
