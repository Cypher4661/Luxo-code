from wpilib import AddressableLED


class LED:
    def __init__(self) -> None:
        led = AddressableLED(0)
        led.LEDData().setRGB(255, 0, 0)
        led.start()
