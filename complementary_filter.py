from filtered_mpu6050 import DigitalFilter


class ComplementaryFilter(DigitalFilter):
    def __init__(self, weight: float, *, angle: float = 0.0) -> None:
        self.weight = weight
        self.angle = angle

    def update(
        self, dt: float, acceleration_angle: float, angular_velocity: float
    ) -> float:
        self.angle = (
            self.weight * (self.angle + angular_velocity * dt)
            + (1.0 - self.weight) * acceleration_angle
        )
        return self.angle
