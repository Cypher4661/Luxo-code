from commands2 import Subsystem
import wpilib
from Constants import led


class ledSubsys(Subsystem):
    def __init__(self) -> None:
        self.m_led = wpilib.AddressableLED(led.led_port)
        self.m_led.setLength(led.led_length)
        super().__init__()

    def change_color(self, color: list[int]) -> None:
        m_led_bufffer = [
            wpilib.AddressableLED.LEDData(color[0], color[1], color[2])
        ] * led.led_length
        self.m_led.setData(m_led_bufffer)
        self.m_led.start()

    def stop_LED(self) -> None:
        self.m_led.stop()
