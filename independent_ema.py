import math


class IndependentEMA:
    """Time independent exponential moving average filter."""

    def __init__(self, tau: float, initial_value: float = 0.0) -> None:
        self.tau = tau
        self.value = initial_value

    def update(self, dt: float, measurement: float) -> float:
        if dt <= 0.0:
            return self.value
        alpha = 1.0 - math.exp(-dt / self.tau) if self.tau != 0.0 else 1.0
        self.value += alpha * (measurement - self.value)
        return self.value
