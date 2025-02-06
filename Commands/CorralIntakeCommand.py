from commands2 import Command
from Subsytem.CorralIntake import corralIntake
from Constants import CorralIntake


class corralIntakeCommand(Command):
    def __init__(self, subsys: corralIntake, power: float):
        self.power = power
        self.subsys = subsys
        self.addRequirements(subsys)
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        if self.subsys.get_motor_velocity() <= CorralIntake.min_velocity:
            self.subsys.duty_motor(0)
        else:
            self.subsys.duty_motor(self.power)
        return super().execute()
