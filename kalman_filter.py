from filtered_mpu6050 import DigitalFilter


class KalmanFilter(DigitalFilter):
    """Kalman filter implementation via Google Gemini."""

    def __init__(self, *, angle: float = 0.0) -> None:
        # Filter tuning parameters (Covariances)
        self.Q_angle = 0.001  # Process noise variance for the accelerometer
        self.Q_bias = 0.003  # Process noise variance for the gyro bias
        self.R_measure = 0.03  # Measurement noise variance

        self.angle = angle  # Reset estimated angle
        self.bias = 0.0  # Reset estimated gyro bias

        # Error covariance matrix
        self.P = [
            [0.0, 0.0],
            [0.0, 0.0],
        ]

    def update(self, dt: float, new_angle: float, new_rate: float) -> float:
        # Step 1: Predict state ahead
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Step 2: Predict error covariance ahead
        self.P[0][0] += dt * (
            dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle
        )
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Step 3: Calculate Kalman Gain
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        # Step 4: Update estimate with measurement
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Step 5: Update error covariance matrix
        p00_temp = self.P[0][0]
        p01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * p00_temp
        self.P[0][1] -= K[0] * p01_temp
        self.P[1][0] -= K[1] * p00_temp
        self.P[1][1] -= K[1] * p01_temp

        return self.angle
