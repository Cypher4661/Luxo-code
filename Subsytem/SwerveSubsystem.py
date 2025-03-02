import math
import RobotContainer

from wpimath import filter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from commands2 import Subsystem

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpiutil import SendableBuilder

from Constants import DriveConstants, OIConstants, ModuleConstants
from Subsytem.SwerveModule import SwerveModule
from navx import AHRS
import wpilib


class SwerveSubsystem(Subsystem):
    def __init__(self) -> None:
        self.gyro = AHRS.create_spi()
        self.gyro.configureVelocity(True, False, False, False)

        self.modules = [
            SwerveModule('FronLeft',
                DriveConstants.kFrontLeftDriveMotorPort,
                DriveConstants.kFrontLeftTurningMotorPort,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
            ),
            SwerveModule('FrontRight',
                DriveConstants.kFrontRightDriveMotorPort,
                DriveConstants.kFrontRightTurningMotorPort,
                DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            ),
            SwerveModule('BackLeft',
                DriveConstants.kBackLeftDriveMotorPort,
                DriveConstants.kBackLeftTurningMotorPort,
                DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            ),
            SwerveModule('BackRight',
                DriveConstants.kBackRightDriveMotorPort,
                DriveConstants.kBackRightTurningMotorPort,
                DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            )]
        self.odometer = SwerveDrive4PoseEstimator(
            DriveConstants.kDriveKinematics,
            self.getRotation2d(),
            self.getModulesPosition(),
            Pose2d(0, 0, Rotation2d(0)),
        )
        self.field2d = wpilib._wpilib.Field2d()
        super().__init__()

    def zeroHeading(self) -> None:
        self.gyro.reset()
        self.gyro.setAngleAdjustment(0)
        self.resetOdometry(self.getPose())

    def autoHeading(self, angle: float) -> None:
        self.gyro.reset()
        self.gyro.setAngleAdjustment(angle)
        self.resetOdometry(self.getPose())

    def getHeading(self) -> float:
        return self.gyro.getFusedHeading()

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self) -> Pose2d:
        return self.odometer.getEstimatedPosition()

    def resetOdometry(self, pose: Pose2d) -> None:
        self.odometer.resetPosition(self.getRotation2d(), self.getModulesPosition(), pose)

    def getModulesPosition(self) :
        return (self.modules[0].get_position(),
                self.modules[1].get_position(),
                self.modules[2].get_position(),
                self.modules[3].get_position())

    def getModulesState(self) :
        return (self.modules[0].getState(),
                self.modules[1].getState(),
                self.modules[2].getState(),
                self.modules[3].getState())

    def periodic(self) -> None:
        self.odometer.update(self.getRotation2d(), self.getModulesPosition())
        self.field2d.setRobotPose(self.getPose())

    def setModuleStates(self, desiredStates) -> None:
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.swerve_max_speed
        )
        for i in range(len(self.modules)):
            self.modules[i].setDesiredState(desiredStates[i], True)

    def drive(
        self, xSpeed: float, ySpeed: float, tSpeed: float) -> None:
        xSpeed = xSpeed if abs(xSpeed) > OIConstants.kStickDriftLX else 0.0
        ySpeed = ySpeed if abs(ySpeed) > OIConstants.kStickDriftLY else 0.0
        tSpeed = tSpeed if abs(tSpeed) > OIConstants.kStickDriftRX else 0.0
        cSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)
        self.setSpeeds(cSpeed)

    def setSpeeds(self, speed: ChassisSpeeds, correctColor: bool = True) -> None:
        if correctColor and RobotContainer.isRed:
            speed.vy = -speed.vy
            speed.vx = -speed.vx
        temp = ChassisSpeeds.fromRobotRelativeSpeeds(speed, self.getRotation2d())
        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            temp, self.getRotation2d())
        self.setModuleStates(moduleState)

    def getCSpeed(self) -> ChassisSpeeds:
        module_states = self.getModulesState()
        return DriveConstants.kDriveKinematics.toChassisSpeeds(module_states)

    def getVelocity(self):
        speeds = self.getCSpeed()
        return math.sqrt(speeds.vx*speeds.vx + speeds.vy*speeds.vy)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty('Gyro', self.getHeading, lambda x: None)
        wpilib.SmartDashboard.putData('Field', self.field2d)

