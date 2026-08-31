from math import radians

kp = 0.0
ki = 0.0
kd = 0.0
kd_tau = 0.15
feedforward = 0.0

tick_rate = 240.0

abort_angle = radians(45.0)

complementary_filter_weight = 0.95
