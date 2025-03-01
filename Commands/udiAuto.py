from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem


class UdiAuto(Command):
    def __init__(self, subsys: SwerveSubsystem):
        super().__init__()
        self.subsys = subsys
        self.count = 0
        self.addRequirements(self.subsys)
        self.rotation = subsys.getRotation2d()
        self.x = 0
        self.y = 0

    def initialize(self):
        self.count = 0
        self.rotation = self.subsys.getRotation2d()
        self.x = self.rotation.cos()
        self.y = self.rotation.sin()
        self.subsys.drive(self.x,self.y,0)

    def execute(self):
        self.subsys.drive(self.x,self.y,0)
        self.count += 1
  
    def end(self, interrupted: bool):
        self.subsys.drive(0,0,0)

    def isFinished(self) -> bool:
        return self.count > 100
