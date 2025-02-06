from commands2 import Command
from wpilib import SmartDashboard
from Subsytem.AlgiArmSubsys import algiArmSubsys
from Constants import AlgiSubsys


class algiArmCommand(Command):
    def __init__(self, subsys: algiArmSubsys, angle: float):
        self.subsys: algiArmSubsys = subsys
        self.addRequirements(subsys)
        self.angle = angle
        super().__init__()

    def initialize(self):
        self.subsys.rest_encoder()
        return super().initialize()

    def execute(self):
        if (
            self.subsys.at_limit()
            or abs(self.subsys.get_current_degree() - self.angle) <= AlgiSubsys.deadband
        ):
            self.subsys.stop()
        else:
            self.subsys.motor_to_position(self.angle)
        return super().execute()
