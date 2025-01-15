from commands2 import Command
from Subsytem.LEDSubsys import ledSubsys
from Constants import led


class LEDAnimationCommand(Command):
    def __init__(self, subsys: ledSubsys, start_color: list[int], end_color: list[int]):
        super().__init__()
        self.subsys = subsys
        self.start_color_const = start_color.copy()  # Prevent shared reference
        self.start_color = start_color.copy()
        self.end_color = end_color
        self.addRequirements(subsys)

        # Calculate rates
        rates = [
            (self.end_color[0] - self.start_color[0]) / led.led_length,
            (self.end_color[1] - self.start_color[1]) / led.led_length,
            (self.end_color[2] - self.start_color[2]) / led.led_length,
        ]
        self.rates = [
            max(1, int(rate)) if rate > 0 else max(-1, int(rate)) for rate in rates
        ]

        self.temp = True

    def initialize(self):
        print("LEDAnimationCommand initialized.")
        return super().initialize()

    def execute(self):
        self.subsys.change_color(self.start_color)

        if self.temp:
            # Increment colors and clamp
            self.start_color[0] = min(
                self.start_color[0] + self.rates[0], self.end_color[0]
            )
            self.start_color[1] = min(
                self.start_color[1] + self.rates[1], self.end_color[1]
            )
            self.start_color[2] = min(
                self.start_color[2] + self.rates[2], self.end_color[2]
            )

            if self.start_color[2] >= self.end_color[2]:
                self.temp = False
        else:
            # Decrement colors and clamp
            self.start_color[0] = max(
                self.start_color[0] - self.rates[0], self.start_color_const[0]
            )
            self.start_color[1] = max(
                self.start_color[1] - self.rates[1], self.start_color_const[1]
            )
            self.start_color[2] = max(
                self.start_color[2] - self.rates[2], self.start_color_const[2]
            )

            if self.start_color[2] <= self.start_color_const[2]:
                self.temp = True

    def isFinished(self):
        return False
