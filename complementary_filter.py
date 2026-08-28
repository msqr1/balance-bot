from math import atan2, hypot


class ComplementaryFilter:
    def __init__(self, weight: float, *, last_angle: float = 0.0) -> None:
        self.weight = weight
        self.last_angle = 0.0

    def calculate(
        self, dt: float, angular_velocity: float, acceleration_angle: float
    ) -> float:
        angle = (
            self.weight * (self.last_angle + angular_velocity * dt)
            + (1.0 - self.weight) * acceleration_angle
        )
        self.last_angle = angle
        return angle

    @staticmethod
    def get_acceleration_pitch(accel_x: float, accel_y: float, accel_z: float) -> float:
        """Get pitch (Y) in radians, where 0.0 is no tilt and forward is positive."""
        return atan2(accel_x, hypot(accel_y, accel_z))
