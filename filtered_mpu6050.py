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
        self._pitch = 0.0
        self.complementary_filter = ComplementaryFilter(
            filter_weight, last_angle=self._pitch
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
        angular_velocity = self.oriented_gyro[1]
        acceleration_pitch = self.acceleration_pitch
        self._pitch = self.complementary_filter.calculate(
            dt, angular_velocity, acceleration_pitch
        )

    @property
    def acceleration_pitch(self) -> float:
        return self.complementary_filter.get_acceleration_pitch(
            *self.oriented_acceleration
        )

    @property
    def pitch(self) -> float:
        """Get pitch in radians, where 0.0 is no tilt and forward pitch is positive."""
        return self._pitch

    def __repr__(self) -> str:
        return (
            f"<{type(self).__name__}{{acceleration={self.acceleration}, "
            f"gyro={self.gyro}, pitch={self.pitch}, temperature={self.temperature}, "
            f"oriented_acceleration={self.oriented_acceleration}, "
            f"oriented_gyro={self.oriented_gyro}, "
            f"acceleration_pitch={self.acceleration_pitch}}}>"
        )
