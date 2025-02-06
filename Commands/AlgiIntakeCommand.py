from commands2 import Command
from Subsytem.AlgiIntake import algiIntake
from Constants import AlgiIntake


class algiIntakeCommand(Command):
    def __init__(self, subsys: algiIntake, power: float):
        self.power = power
        self.subsys = subsys
        self.addRequirements(self.subsys)
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        if self.subsys.get_motor_velocity() <= AlgiIntake.min_velocity:
            self.subsys.duty_motor(0)
        else:
            self.subsys.duty_motor(self.power)
        return super().execute()
