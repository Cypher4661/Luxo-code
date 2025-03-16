from commands2 import Command
from wpilib import SmartDashboard
from Subsytem.CorallArmSubsy import corralArmSubsys
from Constants import CoralSubsys

class CorralArmCalibrate(Command):
    def __init__(
        self, subsys: corralArmSubsys
    ):
        self.subsys: corralArmSubsys = subsys
        self.addRequirements(subsys)
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        self.subsys.setPower(-0.3)
        return super().execute()

    def isFinished(self):
        return (self.subsys.get_motor_current() >= 25)
        return super().isFinished()

    def end(self, interrupted):
        self.subsys.rest_encoder()
        self.subsys.stop()
        return super().end(interrupted)

