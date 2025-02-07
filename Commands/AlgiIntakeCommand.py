from commands2 import Command
from Subsytem.AlgiIntake import algiIntake
from Constants import AlgiIntake


class algiIntakeCommand(Command):
    def __init__(self, subsys: algiIntake, power: float, isDeafultCommand:bool = False):
        self.power = power
        self.subsys = subsys
        self.addRequirements(self.subsys)
        self.isDeafultCommand = isDeafultCommand
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self): 
        self.subsys.duty_motor(self.power)
        return super().execute()
    
    def isFinished(self):
        return not self.isDeafultCommand and self.subsys.get_motor_current() >= 10.5
        return super().isFinished()
    
    def end(self, interrupted):
        self.subsys.stop()
        return super().end(interrupted)