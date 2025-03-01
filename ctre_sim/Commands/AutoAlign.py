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
            Rotation2d.fromDegrees(180),  # 15
            Rotation2d.fromDegrees(90),  # 16
            Rotation2d.fromDegrees(240),  # 17
            Rotation2d.fromDegrees(180),  # 18
            Rotation2d.fromDegrees(120),  # 19
            Rotation2d.fromDegrees(120),  # 20
            Rotation2d.fromDegrees(0),  # 21
            Rotation2d.fromDegrees(300)  # 22
        ]


        self.l3LeftOffset = Translation2d(0.1, 0.13)

    def execute(self):
        robot_pose = self.getRobot()
        if int(self.light.getNumber("tid", 7)) != 0 and robot_pose is not None:
            robotToTag = robot_pose.translation().rotateBy(Rotation2d.fromDegrees(self.subsys.getHeading()))
            offset = self.l3LeftOffset.rotateBy(self.TAG_ANGLE[int(self.light.getNumber("tid", 0))])
            print("Tag angle:",self.TAG_ANGLE)
            robotToTarget:Translation2d = robotToTag + offset
            if robotToTarget is not None:
                self.subsys.drive(
                    robotToTarget.X(),
                    robotToTarget.Y(),
                    self.controller.getRightX(),
                )


    def getRobot(self) -> Pose2d | None:
        april_pose = self.light.getNumberArray("botpose_targetspace", [0, 0, 0, 0, 0, 0])
        if april_pose[0] != 0:
            return Pose2d(-april_pose[0], april_pose[1], Rotation2d.fromDegrees(april_pose[4]))
        return None

    def end(self, interrupted: bool):
        self.subsys.drive(0, 0, 0) 

    def isFinished(self) -> bool:
        return abs(self.controller.getLeftY()) > 0.4 