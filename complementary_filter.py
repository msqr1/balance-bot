class ComplementaryFilter:
    def __init__(self, weight: float, *, angle: float = 0.0) -> None:
        self.weight = weight
        self.last_angle = angle

    def update(
        self, dt: float, acceleration_angle: float, angular_velocity: float
    ) -> float:
        angle = (
            self.weight * (self.last_angle + angular_velocity * dt)
            + (1.0 - self.weight) * acceleration_angle
        )
        self.last_angle = angle
        return angle
