# Balance Bot
Balancing robot for UCSD SPIS 2026 final project

# Calibration
Before actually doing anything, we must correct any offset and/or scaling biases of the MPU (calibrate it).

## Acceleration
The script to calibrate acceleration is in [calibrate_acceleration.py](calibrate_acceleration.py).
Following the instruction of the script, we place each axis (±x, ±y, ±z) of the MPU in the direction of gravity (try our best). Once ready, we collect 100 samples of values
and calculate the center (bias): $(\max(up) + \min(down)) / 2$, and scale: $(2g) / (\max(up) - \min(down))$.

For usage, we re-scale the collected acceleration data with the formula $(accel - center) * scale$

## Gyro (angular velocity)
Script to calibrate the gyro is in [calibrate_gyro.py](calibrate_gyro.py). This time, we don't have scaling, because we don't have any other absolute reference aside from 0 (with acceleration we have g).
We're only going to calibrate for bias. Following the instruction, we keeping the MPU perfectly still, and average 1000 samples for the x, y and z bias: $\sum(gyro) / samples$.

For usage, we subtract bias from the measured gyro: $gyro - bias$

# Pitch (tilt)
Before calculating the pitch of the bot, we're going to establish that, regardless of the axes of the MPU (converted to bot-axes), and with the right hand rule:
- The bot can only along the x-axis, front of the robot is +x (and back is -x).
- Left & right correspond to +y and -y
- Up & down correspond to +z and -z
- Bot standing straight up has 0 pitch, positive pitch is toward +x, and negative pitch is toward -x.

Because the wheels is at the bottom of base plate, and symmetric along the z axis, it can only tilt front & back (into ±x).
Using some ~~meth~~ math, we figure out the pitch in 2 ways:
- Using acceleration (good absolute reference, but very noisy): $\tan^{-1}(-accel_x, \sqrt(accel_y^2+accel_z^2))$
- Using gyro (track movements very well, but accumulate error over time due to integration): $\int_{0}^{t} \omega \, dt$
 
We need a way to get the best of both worlds, which we also currently have 2 ways:
- Complementary filter: simple weighted sum of acceleration and gyro
- Kalman filter: What?

We started with complementary filter, but ended up using Kalman filter.

# Tuning
Most painful part of the project, similar to machine learning hyperparameter tuning, except it's manual. All parameters resides in [constants.py](constants.py).

## First basic idea (PID)
The typical approach to getting started is a simple PID control system. This is very widely used in using throttle or applying force to get to a target (in this case, set-point). Also $error = setpoint - pitch$
- **P**roportion: $kp * error$ for some constant $kp$. This acts like a spring, throttle proportional to how far the pitch is from the set-point.
- **I**ntegral: $ki * \int_{0}^{t} error \, dt$ for some constant $ki$. This acts on the accumulation of error (bot being consistently away from the set point will enlarge the integral, throttling it toward the set point).
- **D**erivative: $-kd * \frac{d}{dx}(pitch)$, for some constant $kd$. This prevent overshooting the set-point by reducing throttle as the bot approach it.

This works quite well, but
