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
from Subsytem.LEDSubsys import ledSubsys
from Subsytem.SwerveSubsystem import SwerveSubsystem

class limelight(Subsystem):
    def __init__(self, swerve: SwerveSubsystem, led:ledSubsys, getVelocity: Callable[[], float],
                 isDisabled: Callable[[], bool]) -> None:
        super().__init__()
        self.isDisabled = isDisabled
        self.led = led
        self.ntTable = NetworkTableInstance.getDefault().getTable(LimeLightConstants.limelight_name)
        self.field2d = wpilib._wpilib.Field2d()
        self.swerve = swerve
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
        if(self.inView()):
            # data - x,y,z,roll,pitch,yaw,latency,tag count, tag span, avg tag distance, avg tag area
            data = self.ntTable.getNumberArray("botpose_wpiblue", [999] * 11)
            if data[0] >0:
                #rotation = Rotation2d.fromDegrees(5) if self.isDisabled() else self.swerve.getRotation2d()
                rotation = data[5]
                print("ANGLE FROM VISION: ", rotation)
                return Pose2d(Translation2d(data[0], data[1]), Rotation2d.fromDegrees(rotation)), data[6]
        return None, 0


    def periodic(self) -> None:
        pose, latency = self.getPose()
        SmartDashboard.putBoolean('Vision/inView', self.inView())
        SmartDashboard.putBoolean('Vision/Pose', pose != None)
        SmartDashboard.putNumber('Vision/tid', self.getTagId())
        SmartDashboard.putNumber('Vision/count', self.validCount)
        
        if pose is not None:
            self.led.change_color([0, 250, 0])
            self.field2d.setRobotPose(pose)
            self.validCount += 1
            if self.validCount > 2:
                #print("UPDATED VISION")
                self.swerve.addVisionMeasurment(pose, wpilib.Timer.getFPGATimestamp() - 0.05) #wpilib.Timer.getFPGATimestamp() - latency
        else:

            self.led.change_color([255, 0, 255])
            self.validCount = 0


    def initSendable1(self, builder: SendableBuilder) -> None:
        builder.addBooleanProperty('InView', self.inView, lambda x : None)
        builder.addDoubleProperty('valid count', lambda : self.validCount, lambda x : None)
        builder.addDoubleProperty('tag', lambda : self.getTagId, lambda x : None)
