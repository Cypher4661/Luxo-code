from ntcore import NetworkTableInstance
from commands2 import Subsystem
from wpiutil import SendableBuilder


from Constants import LimeLightConstants
import math
from ntcore import NetworkTable
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
import wpilib
from collections.abc import Callable
from wpilib import SmartDashboard


class limelight(Subsystem):
    def __init__(self, poseEstimator: SwerveDrive4PoseEstimator, getVelocity: Callable[[], float]) -> None:
        super().__init__()
        self.ntTable = NetworkTableInstance.getDefault().getTable(LimeLightConstants.limelight_name)
        self.field2d = wpilib._wpilib.Field2d()
        self.poseEstimator = poseEstimator
        self.getVelocity = getVelocity
        self.validCount = 0
        SmartDashboard.putData('Limelight/field', self.field2d)


    def id(self) -> int:
        return self.ntTable.getNumber("tid", -1)


    def inView(self) -> bool:
        return bool(self.ntTable.getNumber("tv", 0))


    def getTagId(self):
        if self.inView() and self.validCount > 2:
            return self.id()
        return -1


    def getPose(self):
        if self.inView() and abs(self.getVelocity() < 0.2):
            # data - x,y,x,roll,pitch,yaw,latency,tag count, tag span, avg tag distance, avg tag area
            data = self.ntTable.getNumberArray("botpose_wpiblue", [999] * 11)
            if data[0] != 999:
                return Pose2d(Translation2d(data[0], data[1]), Rotation2d.fromDegrees(data[5])), data[6]
        return None, 0


    def periodic(self) -> None:
        pose, latency = self.getPose()
        if pose:
            self.field2d.setRobotPose(pose)
            self.validCount = self.validCount + 1
            if self.validCount > 2:
                self.poseEstimator.addVisionMeasurement(pose, latency)
        else:
            self.validCount = 0


    def initSendable1(self, builder: SendableBuilder) -> None:
        builder.addBooleanProperty('InView', self.inView, lambda x : None)
        builder.addDoubleProperty('valid count', lambda : self.validCount, lambda x : None)
        builder.addDoubleProperty('tag', lambda : self.getTagId, lambda x : None)
