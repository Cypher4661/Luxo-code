from commands2 import Command
from Subsytem.CorallArmSubsy import corralArmSubsys
from Constants import CoralSubsys


class corralArmCommand(Command):
    def __init__(self, subsys: corralArmSubsys, angle: float):
        self.angle = angle
        self.subsys = subsys
        self.addRequirements(subsys)
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        self.subsys.reset_encoder()
        self.subsys.motor_to_position(self.angle)
        return super().execute()

    def isFinished(self) -> bool:
        return abs(self.angle - self.subsys.get_current_angle()) <= CoralSubsys.deadBand
        return super().isFinished()

    def end(self, interrupted: bool):
        self.subsys.stop()
        return super().end(interrupted)
