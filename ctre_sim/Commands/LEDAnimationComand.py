from commands2 import Command
from Subsytem.LEDSubsys import ledSubsys
from Constants import led
from wpilib import Timer


class LEDAnimationCommand(Command):
    def __init__(
        self,
        subsys: ledSubsys,
        start_color: list[int],
        end_color: list[int],
        rate: float,
    ):
        super().__init__()
        self.subsys = subsys
        self.start_color = start_color.copy()
        self.end_color = end_color
        self.rate = rate
        self.addRequirements(subsys)
        self.temp = True

    def initialize(self):
        self.subsys.change_color(self.start_color)
        self.time = Timer()
        self.time.start()
        return super().initialize()

    def execute(self):
        if self.time.get() >= self.rate:
            self.temp = not self.temp
            self.time.restart()
            if self.temp:
                self.subsys.change_color(self.start_color)
            else:
                self.subsys.change_color(self.end_color)

    def isFinished(self):
        return False
