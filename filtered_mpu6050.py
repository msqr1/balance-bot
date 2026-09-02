from math import atan2, hypot

from adafruit_mpu6050 import _MPU6050_DEFAULT_ADDRESS, MPU6050, STANDARD_GRAVITY
from adafruit_mpu6050 import Bandwidth as Bandwidth  # noqa: PLC0414
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
        calibration_data: tuple[
            tuple[float, float],
            tuple[float, float],
            tuple[float, float],
        ] = ((-STANDARD_GRAVITY, STANDARD_GRAVITY),) * 3,
    ) -> None:
        super().__init__(i2c_bus, address)
        self.calibration_data = calibration_data
        self.filter_bandwidth = filter_bandwidth
        if len(axes) != 3 or set(axes) != set(range(3)):
            raise ValueError(axes)
        self.axes = axes
        if len(signs) != 3 or not set(signs) <= {-1, 1}:
            raise ValueError(signs)
        self.signs = signs
        self.complementary_filter = ComplementaryFilter(filter_weight)
        try:
            self._oriented_acceleration = self._get_oriented_acceleration()
        except OSError:
            self._oriented_acceleration = 0.0, 0.0, 0.0
        self._oriented_pitch = self.complementary_filter.last_angle = (
            self.oriented_acceleration_pitch
        )

    @property
    def oriented_acceleration(self) -> tuple[float, float, float]:
        return self._oriented_acceleration

    @property
    def oriented_gyro(self) -> tuple[float, float, float]:
        gyro = self.gyro
        return tuple(gyro[self.axes[i]] * self.signs[i] for i in range(3))

    def _get_oriented_acceleration(self) -> tuple[float, float, float]:
        acceleration = self.acceleration
        return tuple(
            self._get_calibrated(acceleration[axis], *self.calibration_data[axis])
            * sign
            for axis, sign in zip(self.axes, self.signs, strict=True)
        )  # type: ignore[return-value]

    def update(self, dt: float) -> None:
        """Update the filtered pitch, each frame."""
        try:
            self._oriented_acceleration = self._get_oriented_acceleration()
        except OSError:
            pass
        try:
            pitch_angular_velocity = self.oriented_gyro[1]
            oriented_acceleration_pitch = self.oriented_acceleration_pitch
        except OSError:
            return
        self._oriented_pitch = self.complementary_filter.update(
            dt, oriented_acceleration_pitch, pitch_angular_velocity
        )

    @property
    def oriented_acceleration_pitch(self) -> float:
        """Get oriented pitch calculated purely from linear acceleration, in radians.

        0.0 is no tilt and forward pitch is positive.
        """
        return get_acceleration_pitch(*self.oriented_acceleration)

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

    @staticmethod
    def _get_calibrated(raw: float, min_grav: float, max_grav: float) -> float:
        center = (min_grav + max_grav) * 0.5
        scale = (2.0 * STANDARD_GRAVITY) / (max_grav - min_grav)
        return scale * (raw - center)


def get_acceleration_pitch(accel_x: float, accel_y: float, accel_z: float) -> float:
    """Get pitch (Y) in radians, where 0.0 is no tilt and forward is positive."""
    return atan2(-accel_x, hypot(accel_y, accel_z))
