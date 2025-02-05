from commands2 import Command
from Subsytem.CorralIntake import corralIntake


class corralIntakeCommand(Command):
    def __init__(self, subsys: corralIntake, power: float):
        self.power = power
        self.subsys = subsys
        self.addRequirements(subsys)
        super().__init__()

    def initialize(self):
        self.subsys.duty_motor(self.power)
        return super().initialize()

    def execute(self):
        return super().execute()

    def isFinished(self) -> bool:
        return self.subsys.at_limit()
        return super().isFinished()

    def end(self, interrupted: bool):
        self.subsys.duty_motor(0)
        return super().end(interrupted)
