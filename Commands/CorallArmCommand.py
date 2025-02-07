from commands2 import Command
from Subsytem.CorallArmSubsy import corralArmSubsys
from Constants import CoralSubsys


class corralArmCommand(Command):
    def __init__(self, subsys: corralArmSubsys, angle: float, isDeafultCommand:bool = False):
        self.angle = angle
        self.subsys = subsys
        self.addRequirements(subsys)
        self.isDeafultCommand = isDeafultCommand
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        if (
            self.subsys.at_limit()
            or abs(self.subsys.get_current_angle() - self.angle) <= CoralSubsys.deadBand
        ):
            self.subsys.stop()
        else:
            self.subsys.motor_to_position(self.angle)
        return super().execute()

    def isFinished(self):
        return not self.isDeafultCommand and abs(self.subsys.get_current_angle() - self.angle) <= CoralSubsys.deadBand
        return super().isFinished()
    
    def end(self, interrupted):
        return super().end(interrupted)