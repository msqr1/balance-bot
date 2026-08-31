from math import nan


class PIDController:
    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        feedforward: float = 0.0,
        setpoint: float = 0.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.feedforward = feedforward
        self.setpoint = setpoint
        self.error_integral = 0.0
        self.last_error = 0.0

    def calculate(self, dt: float, measurement: float) -> float:
        error = self.setpoint - measurement
        self.error_integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return (
            self.kp * error
            + self.ki * self.error_integral
            + self.kd * derivative
            + self.feedforward * sign(error)
        )


def sign(x: float) -> float:
    return -1.0 if x < 0.0 else 1.0 if x > 0.0 else 0.0 if x == 0.0 else nan
