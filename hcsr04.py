from time import monotonic, sleep
from typing import cast

from digitalio import DigitalInOut
from microcontroller import Pin
from pulseio import PulseIn


class NonblockingHCSR04:
    def __init__(self, trigger_pin: Pin, echo_pin: Pin, timeout: float = 0.050) -> None:
        self._trig = DigitalInOut(trigger_pin)
        self._trig.switch_to_output(value=False)

        self._echo = PulseIn(echo_pin, maxlen=1, idle_state=False)
        self._timeout = timeout

        self._last_trigger_time = 0.0
        self._waiting_for_echo = False
        self.distance_cm: float | None = None
        """None indicates no reading yet, or a timeout"""

    def trigger(self) -> None:
        """Fires the ultrasonic pulse and returns immediately."""
        if self._waiting_for_echo:
            return  # Already waiting for a ping to return
        self._echo.clear()
        # Send a 10-microsecond (0.01ms) pulse
        self._trig.value = True
        sleep(0.00001)
        self._trig.value = False
        self._last_trigger_time = monotonic()
        self._waiting_for_echo = True

    def update(self) -> float | None:
        """Checks if a distance reading is ready without blocking."""
        if not self._waiting_for_echo:
            return self.distance_cm
        if self._echo:
            # Check if hardware interrupt captured a pulse
            pulse_length_us = cast(int, self._echo[0])
            self._echo.clear()
            self._waiting_for_echo = False
            # Sound travels at ~343m/s (29.1 us/cm). Distance is round-trip.
            self.distance_cm = pulse_length_us * 0.01715
            return self.distance_cm
        if monotonic() - self._last_trigger_time > self._timeout:
            # Handle timeout (object out of range)
            self._waiting_for_echo = False
            self._echo.clear()
            self.distance_cm = None
        return self.distance_cm
