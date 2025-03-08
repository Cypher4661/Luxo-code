from commands2 import Command
from wpilib import SmartDashboard
from Subsytem.AlgiArmSubsys import algiArmSubsys
from Constants import AlgiSubsys


class algiArmCalibrate(Command):
    def __init__(
        self, subsys: algiArmSubsys
    ):
        self.subsys: algiArmSubsys = subsys
        self.addRequirements(subsys)
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        self.subsys.setPower(-0.7)
        return super().execute()

    def isFinished(self):
        return (self.subsys.at_limit())
        return super().isFinished()

    def end(self, interrupted):
        self.subsys.rest_encoder()
        self.subsys.stop()
        return super().end(interrupted)
