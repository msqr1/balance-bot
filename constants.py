setpoint = -5.40

kp = 0.50
ki = 0.00
kd = 0.006
ks = 0.10
kv = 0.8
pid_tau = 0.00
pitch_rate_tau = 0.010
speed_tau = 0.00

velocity_tau = 0.50
velocity_correction = 0.10  # for m/s

distance_tau = 1.00
distance_correction = 0.003  # for centimeters
distance_setpoint = 40.0  # centimeters
distance_max_error = 20.0

ping_interval = 0.060

tick_rate = 200.0

abort_angle = 45.0

kalman_q_angle = 0.0001
kalman_q_bias = 0.003
kalman_r_measure = 5.0

# complementary_filter_weight = 0.98

left_multiplier = 0.90
right_multiplier = 1.0

enable_motors = True
