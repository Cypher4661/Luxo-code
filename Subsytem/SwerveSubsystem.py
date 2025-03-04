import math
import RobotContainer
from wpimath import filter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from wpimath.estimator import SwerveDrive4PoseEstimator
from Constants import DriveConstants, OIConstants
from Subsytem.SwerveModule import SwerveModule
from navx import AHRS
import wpilib
from commands2 import Subsystem

class SwerveSubsystem(Subsystem):
    def __init__(self,) -> None:
        super().__init__()
        self._isRed = False
        self.gyro = AHRS.create_spi()
        self.gyroOffset = 0

        self.xLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
        )
        self.yLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
        )
        self.tLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
        )

        self.frontLeft: SwerveModule = SwerveModule('FrontLeft',
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        )

        self.frontRight: SwerveModule = SwerveModule('FrontRight',
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        )

        self.backLeft: SwerveModule = SwerveModule('BackLeft',
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        )

        self.backRight: SwerveModule = SwerveModule('BackRight',
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        )
        wpilib.SmartDashboard.putData('FrontLeft',self.frontLeft)
        wpilib.SmartDashboard.putData('FrontRight',self.frontRight)
        wpilib.SmartDashboard.putData('BackLeft',self.backLeft)
        wpilib.SmartDashboard.putData('BackRight',self.backRight)
        self.odometer = SwerveDrive4PoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.getGyroHeading()),
            (
                self.frontLeft.get_position(),
                self.frontRight.get_position(),
                self.backLeft.get_position(),
                self.backRight.get_position(),
            ),
            Pose2d(0, 0, Rotation2d(0)),
        )
        self.field = wpilib._wpilib.Field2d()
        self.brake = True
        wpilib.SmartDashboard.putData('Field Pos', self.field)
        wpilib.SmartDashboard.putData('Swerve', self)

    def getVelocity(self):
        speeds = self.getCSpeed()
        return math.sqrt(speeds.vx*speeds.vx + speeds.vy*speeds.vy)

    def zeroHeading(self) -> None:
        self.autoHeading(0)

    def autoHeading(self, angle: float) -> None:
        pose = Pose2d(self.getPose().translation(), Rotation2d.fromDegrees(angle))
        self.resetOdometry(pose)
        wpilib.SmartDashboard.putNumber('Reset Gyro to', angle)
        wpilib.SmartDashboard.putNumber('Reset Gyro updated', self.getHeading())
        
        

    def getGyroHeading(self) -> float:
        angle = self.gyro.getYaw()
        return 360- angle
    
    def getGyroRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getGyroHeading())
    
    def getHeading(self):
        return self.getRotation2d().degrees()

    def getRotation2d(self) -> Rotation2d:
        return self.getPose().rotation()

    def getPose(self) -> Pose2d:
        return self.odometer.getEstimatedPosition()
    
    def getTranslation2d(self):
        return self.getPose().translation

    def resetOdometry(self, pose: Pose2d) -> None:
        module_positions = (
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backLeft.get_position(),
            self.backRight.get_position(),
        )
        self.odometer.resetPosition(self.getGyroRotation2d(), module_positions, pose)

    def periodic(self) -> None:
        # print(self.getHeading())
        module_positions = (
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backLeft.get_position(),
            self.backRight.get_position(),
        )
        self.odometer.update(Rotation2d.fromDegrees(self.getGyroHeading()), module_positions)
        self.field.setRobotPose(self.odometer.getEstimatedPosition())

    def setModuleStates(self, desiredStates) -> None:
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.swerve_max_speed
        )
        self.frontLeft.setDesiredState(desiredStates[0], False)
        self.frontRight.setDesiredState(desiredStates[1], False)
        self.backLeft.setDesiredState(desiredStates[2], False)
        self.backRight.setDesiredState(desiredStates[3], False)

    def drive(
        self, xSpeed: float, ySpeed: float, tSpeed: float) -> None:
        xSpeed = xSpeed if abs(xSpeed) > OIConstants.kStickDriftLX else 0.0
        ySpeed = ySpeed if abs(ySpeed) > OIConstants.kStickDriftLY else 0.0
        tSpeed = tSpeed if abs(tSpeed) > OIConstants.kStickDriftRX else 0.0
        xSpeed = xSpeed * DriveConstants.swerve_max_speed
        ySpeed = ySpeed * DriveConstants.swerve_max_speed
        tSpeed = tSpeed * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond
        cSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)
        self.setSpeeds(cSpeed, True)

    def setSpeeds(self, speed: ChassisSpeeds, correctColor: bool = False) -> None:
        if correctColor and self._isRed:
            speed.vy = -speed.vy
            speed.vx = -speed.vx
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(speed, self.getRotation2d())
        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(temp)
        self.setModuleStates(moduleState)

    def autoDrive(self, speed: ChassisSpeeds, feedforward=None) -> None:
        temp = ChassisSpeeds.fromRobotRelativeSpeeds(speed, self.getRotation2d())
        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            temp, Translation2d()
        )
        self.setModuleStates(moduleState)

    def getCSpeed(self) -> ChassisSpeeds:
        module_states = (
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backRight.getState(),
            self.backLeft.getState(),
        )
        return DriveConstants.kDriveKinematics.toChassisSpeeds(module_states)

    def change_drive(self, switch: bool):
        self.special_drive = switch

    def check_module_angle(self) -> None:
        self.frontLeft.reset_to_absolute()
        self.frontRight.reset_to_absolute()
        self.backLeft.reset_to_absolute()
        self.backRight.reset_to_absolute()

    def setBrake(self,brake:bool):
        self.brake = brake
        self.frontLeft.setBrake(brake)
        self.frontRight.setBrake(brake)
        self.backLeft.setBrake(brake)
        self.backRight.setBrake(brake)

    def getBrake(self):
        return self.brake

    def setRed(self,isRed:bool):
        self._isRed = isRed,
    def isRed(self) -> bool:
        return self._isRed
    
    def initSendable(self, builder):
        builder.addDoubleProperty('Gyro', self.getGyroHeading, lambda x: None)
        builder.addDoubleProperty('Heading', self.getHeading, lambda x: None)
        builder.addBooleanProperty('Brake', self.getBrake, self.setBrake)
#        builder.addBooleanProperty('Is Red', self.isRed, self.setRed), 
        return super().initSendable(builder)