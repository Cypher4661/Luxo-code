from wpimath import filter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from commands2 import Subsystem

# from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import SwerveDrive4Odometry
from Constants import DriveConstants, OIConstants, ModuleConstants
from Subsytem.SwerveModule import SwerveModule
from navx import AHRS
from pathplannerlib.config import (
    RobotConfig,
    PIDConstants,
    ModuleConfig,
)
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder
import wpilib
from wpimath.system.plant import DCMotor


class SwerveSubsystem(Subsystem):
    def __init__(self) -> None:
        self.autoCSpeed: ChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            0, 0, 0, Rotation2d.fromDegrees(0)
        )
        self.special_drive = False
        self.gyro = AHRS.create_spi()

        self.xLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
        )
        self.yLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
        )
        self.tLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
        )

        self.frontLeft: SwerveModule = SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        )

        self.frontRight: SwerveModule = SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        )

        self.backLeft: SwerveModule = SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        )

        self.backRight: SwerveModule = SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        )
        self.odometer = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            self.getRotation2d(),
            (
                self.frontLeft.get_position(),
                self.frontRight.get_position(),
                self.backLeft.get_position(),
                self.backRight.get_position(),
            ),
            Pose2d(0, 0, Rotation2d(0)),
        )
        config = RobotConfig(
            30.0,
            15.0,
            ModuleConfig(
                ModuleConstants.kWheelDiameterMeters,
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                1.0,
                DCMotor(
                    12,
                    22.68,
                    53.25,
                    1.5,
                    DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond,
                ),
                40.0,
                1
            ),
            [
                Translation2d(75.5 / 2, 75.5 / 2),
                Translation2d(75.5 / 2, 75.5 / 2),
                Translation2d(75.5 / 2, 75.5 / 2),
                Translation2d(75.5 / 2, 75.5 / 2),
            ],
            DriveConstants.kTrackWidth,
        )
        AutoBuilder.configure(
            self.getPose,  # Robot pose supplier
            self.resetOdometry,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getCSpeed,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speed, feedforward: self.autoDrive(speed),
            PPHolonomicDriveController(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(5, 0.0, 0.0),  # Translation PID constants
                PIDConstants(5, 0.0, 0.0),  # Rotation PID constants
            ),
            config,
            self.shouldFlipPath,
            self
        )

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return False
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def zeroHeading(self) -> None:
        self.gyro.reset()
        self.gyro.setAngleAdjustment(0)

    def autoHeading(self, angle: float) -> None:
        self.zeroHeading()
        self.gyro.setAngleAdjustment(angle)

    def getHeading(self) -> float:
        angle = self.gyro.getYaw() % 360
        return 360 - angle

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self) -> Pose2d:
        return self.odometer.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        module_positions = (
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backLeft.get_position(),
            self.backRight.get_position(),
        )
        self.odometer.resetPosition(self.getRotation2d(), module_positions, pose)

    def periodic(self) -> None:
        module_positions = (
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backLeft.get_position(),
            self.backRight.get_position(),
        )
        self.odometer.update(self.getRotation2d(), module_positions)

    def setModuleStates(self, desiredStates) -> None:
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.swerve_max_speed
        )
        self.frontLeft.setDesiredState(desiredStates[3], True)
        self.frontRight.setDesiredState(desiredStates[2], True)
        self.backLeft.setDesiredState(desiredStates[1], True)
        self.backRight.setDesiredState(desiredStates[0], True)

    def drive(
        self, xSpeed: float, ySpeed: float, tSpeed: float, fieldOriented: bool = True
    ) -> None:
        xSpeed = xSpeed if abs(xSpeed) > OIConstants.kStickDriftLX else 0.0
        ySpeed = ySpeed if abs(ySpeed) > OIConstants.kStickDriftLY else 0.0
        tSpeed = tSpeed if abs(tSpeed) > OIConstants.kStickDriftRX else 0.0

        cSpeed: ChassisSpeeds
        if fieldOriented:
            cSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, tSpeed, self.getRotation2d()
            )
        else:
            cSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)

        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            cSpeed, Translation2d()
        )
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
