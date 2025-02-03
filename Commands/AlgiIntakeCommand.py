from commands2 import Command
from Subsytem.AlgiIntake import algiIntake


class algiIntakeCommand(Command):
    def __init__(self, subsys: algiIntake, power: float):
        self.power = power
        self.subsys = subsys
        self.addRequirements(self.subsys)
        super().__init__()

    def initialize(self):
        self.subsys.duty_motor(self.power)
        return super().initialize()

    def execute(self):
        return super().execute()

    def isFinished(self) -> bool:
        return super().isFinished()

    def end(self, interrupted: bool):
        self.subsys.duty_motor(0)
        return super().end(interrupted)
