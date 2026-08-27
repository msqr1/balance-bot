from gpiozero import OutputDevice, PWMOutputDevice


class HBridgeMotor:
    def __init__(self, in1: int, in2: int, pwm: int) -> None:
        self.in1 = OutputDevice(in1)
        self.in2 = OutputDevice(in2)
        self.pwm = PWMOutputDevice(pwm)

    def move(self, speed: float) -> None:
        speed = min(max(speed, -1.0), 1.0)
        if speed > 0.0:
            self.in1.on()
            self.in2.off()
        elif speed < 0.0:
            self.in1.off()
            self.in2.on()
        else:
            self.in1.off()
            self.in2.off()
        self.pwm.value = abs(speed)
