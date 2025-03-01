from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem
from commands2.button import CommandXboxController
from Constants import LimeLightConstants
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from ntcore import NetworkTableInstance


class AutoAlign(Command):
    def __init__(self, subsys: SwerveSubsystem, controller: CommandXboxController):
        super().__init__()
        self.subsys = subsys
        self.addRequirements(subsys)
        self.controller = controller
        self.pix = 800
        self.pov = 56.2
        self.onepix = 56.2/800
        self.tagSize = 0.1651
        self.halfTagSize = self.tagSize/2
        self.light = NetworkTableInstance.getDefault().getTable(
            LimeLightConstants.limelight_name
        )
        self.TAG_ANGLE = [
            Rotation2d.fromDegrees(0),  # 0
            Rotation2d.fromDegrees(126),  # 1
            Rotation2d.fromDegrees(234),  # 2
            Rotation2d.fromDegrees(270),  # 3
            Rotation2d.fromDegrees(0),  # 4
            Rotation2d.fromDegrees(0),  # 5
            Rotation2d.fromDegrees(300),  # 6
            Rotation2d.fromDegrees(0),  # 7
            Rotation2d.fromDegrees(60),  # 8
            Rotation2d.fromDegrees(120),  # 9
            Rotation2d.fromDegrees(180),  # 10
            Rotation2d.fromDegrees(240),  # 11
            Rotation2d.fromDegrees(54),  # 12
            Rotation2d.fromDegrees(306),  # 13
            Rotation2d.fromDegrees(180),  # 14
            Rotation2d.fromDegrees(180), # 15
            Rotation2d.fromDegrees(90),  # 16
            Rotation2d.fromDegrees(240),  # 17
            Rotation2d.fromDegrees(180),  # 18
            Rotation2d.fromDegrees(120),  # 19
            Rotation2d.fromDegrees(120),  # 20
            Rotation2d.fromDegrees(0),  # 21
            Rotation2d.fromDegrees(300)  # 22
        ]


        self.l3LeftOffset = Translation2d(0.5, 0.5)

    def execute(self):
        if int(self.light.getNumber("tid", 0)) != 0:
            robotToTag = self.getRobotToTag().rotateBy(self.subsys.getRotation2d())
            offset = self.l3LeftOffset.rotateBy(self.TAG_ANGLE[int(self.light.getNumber("tid", 0))])
            robotToTarget:Translation2d = robotToTag + offset
            if robotToTarget is not None:
                print(self.getDist())
                # self.subsys.drive(
                #     robotToTarget.X(),
                #     robotToTarget.Y(),
                #     self.controller.getRightX(),
                # )


    def getDist(self) -> float | None:
        cornecs = self.light.getNumberArray("tcornxy", [0, 0, 0, 0])
        if cornecs[2] != 0 and cornecs[3] != 0:
            return self.halfTagSize/((abs(cornecs[2]-cornecs[3])/2)*self.onepix)
        return None
    
    def getRobotToTag(self) -> Translation2d | None:
        yaw = self.light.getNumber("ty", 0)
        if(self.getDist() is not None):
            return Translation2d(self.getDist(), Rotation2d.fromDegrees(yaw))
        else:
            return None

    def end(self, interrupted: bool):
        self.subsys.drive(0, 0, 0) 

    def isFinished(self) -> bool:
        return abs(self.controller.getLeftY()) > 0.4 