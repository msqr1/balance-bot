from adafruit_mpu6050 import _MPU6050_DEFAULT_ADDRESS, MPU6050, Bandwidth
from busio import I2C

from complementary_filter import ComplementaryFilter


class FilteredMPU6050(MPU6050):
    def __init__(
        self,
        i2c_bus: I2C,
        address: int = _MPU6050_DEFAULT_ADDRESS,
        filter_weight: float = 0.95,
        filter_bandwidth: int = Bandwidth.BAND_260_HZ,
        axes: tuple[int, int, int] = (0, 1, 2),
        signs: tuple[int, int, int] = (1, 1, 1),
    ) -> None:
        super().__init__(i2c_bus, address)
        self._oriented_pitch = 0.0
        self.complementary_filter = ComplementaryFilter(
            filter_weight, last_angle=self._oriented_pitch
        )
        self.filter_bandwidth = filter_bandwidth
        if len(axes) != 3 or set(axes) != set(range(3)):
            raise ValueError(axes)
        self.axes = axes
        if len(signs) != 3 or not set(signs) <= {-1, 1}:
            raise ValueError(signs)
        self.signs = signs

    @property
    def oriented_acceleration(self) -> tuple[float, float, float]:
        acceleration = self.acceleration
        return tuple(acceleration[self.axes[i]] * self.signs[i] for i in range(3))

    @property
    def oriented_gyro(self) -> tuple[float, float, float]:
        gyro = self.gyro
        return tuple(gyro[self.axes[i]] * self.signs[i] for i in range(3))

    def update(self, dt: float) -> None:
        """Update the filtered pitch, each frame."""
        try:
            pitch_angular_velocity = self.oriented_gyro[1]
            oriented_acceleration_pitch = self.oriented_acceleration_pitch
        except OSError:
            return
        self._oriented_pitch = self.complementary_filter.calculate(
            dt, pitch_angular_velocity, oriented_acceleration_pitch
        )

    @property
    def oriented_acceleration_pitch(self) -> float:
        """Get oriented pitch calculated purely from linear acceleration, in radians.

        0.0 is no tilt and forward pitch is positive.
        """
        return self.complementary_filter.get_acceleration_pitch(
            *self.oriented_acceleration
        )

    @property
    def oriented_pitch(self) -> float:
        """Get filtered oriented pitch in radians.

        0.0 is no tilt and forward pitch is positive.
        """
        return self._oriented_pitch

    def __repr__(self) -> str:
        try:
            return (
                f"<{type(self).__name__}{{acceleration={self.acceleration}, "
                f"gyro={self.gyro}, temperature={self.temperature}, "
                f"oriented_acceleration={self.oriented_acceleration}, "
                f"oriented_gyro={self.oriented_gyro}, "
                f"oriented_pitch={self.oriented_pitch}, "
                f"oriented_acceleration_pitch={self.oriented_acceleration_pitch}}}>"
            )
        except OSError as e:
            return f"<{type(self).__name__}{e}>"
