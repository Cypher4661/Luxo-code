from commands2 import Command
from Subsytem.LEDSubsys import ledSubsys


class ledCommand(Command):
    def __init__(self, subsys: ledSubsys, color: list[int]):
        self.subsys = subsys
        self.color = color
        self.run = False
        self.addRequirements(self.subsys)
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        self.subsys.change_color(self.color)
        self.run = True

    def isFinished(self) -> bool:
        return self.run

    def runsWhenDisabled(self) -> bool:
        return True
