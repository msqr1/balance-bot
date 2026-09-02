from math import nan

from independent_ema import IndependentEMA


class PIDController:
    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        tau: float = 0.1,
        ks: float = 0.0,
        setpoint: float = 0.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ks = ks
        self.setpoint = setpoint
        self.error_integral = 0.0
        self.last_error = 0.0
        self.value_ema = IndependentEMA(tau)

    def calculate(
        self, dt: float, measurement: float, measurement_rate: float
    ) -> float:
        value = self.value_ema.update(dt, measurement)
        error = self.setpoint - value
        self.error_integral += error * dt
        self.last_error = error
        return (
            self.kp * error
            + self.ki * self.error_integral
            + self.kd * measurement_rate
            + self.ks * sign(error)
        )


def sign(x: float) -> float:
    return -1.0 if x < 0.0 else 1.0 if x > 0.0 else 0.0 if x == 0.0 else nan
