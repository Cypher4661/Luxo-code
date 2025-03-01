import wpimath
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import math
from wpimath.trajectory import TrapezoidProfileRadians
from rev import SparkMax


class SystemValues:
    l2ArmAngle = 52.5
    l3ArmAngle = 135
    intakeAlgiPower = 0.35
    specialCorralIntakePower = -0.35
    outputAlgiPower = -1
    intakeCorralPower = 0.5
    outputCorralPower = -0.2
    intakeCorralArmAngle = 45.5
    specialCorralIntakeArmAngle = 68.5
    pickAlgiArmAngle = 71
    ouputAlgiArmAngle = 20

# class putOffsets:
#     # 0 - L1, 1 - L2Right, 2 - L2Left, 3 - AlgeaBottom, 4 - L3Right, 5 - L3Left, 6 - AlgeaTop
#     {Translation2d(0,0),Translation2d(0,0),Translation2d(0,0),Translation2d(0,0),Translation2d(0,0),Translation2d(0,0),Translation2d(0,0),Translation2d(0,0)}


class AlgaeIntake:
    motor_id = 33
    maxAmper = 30
    maxVolts = 6
    rampUp = 0.1
    limit_id = 2
    min_velocity = 10
    intakePower = 0.5
    processorPower = -1
    dropL2Power = -0.5
    keepPower = 0.05
    algaeCollectedAmper = 15.5


class CorralIntake:
    motor_id = 22
    maxAmper = 30
    maxVolts = 6
    rampUp = 0.1
    limit_id = 3
    min_velocity = 10


class CoralSubsys:
    motor_id = 21
    deadBand = 0.5  # degrees
    maxVolts = 12
    maxAmper = 40
    rampUp = 0.1
    gearRatio = 80
    limit_id = 1
    kp = 4.8
    ki = 0
    kd = 0.1
    kf = 0
    ks = 0.25
    kv = 0.12
    ka = 0.01
    velocity = 30
    acceleration = 60
    jerk = 600
    revcoder = 7
    encoder_offset = 0.582


class AlgaeSubsys:
    deadband = 1.25  # degrees
    motor1_id = 31
    motor2_id = 32
    limit_id = 5
    motor1_type = SparkMax.MotorType.kBrushless
    motor2_type = SparkMax.MotorType.kBrushless
    gear_ratio = 300
    convertion_factor = 360 / gear_ratio
    kp = 0.02
    ki = 0.0
    kd = 0.01
    kf = 0.0
    maxVelocity = 20
    minVelocity = 0
    maxAcceleration = 40
    revcoder = 2
    encoder_offset = 0.404
    maxPower = 1


class led:
    led_port = 4
    led_length = 120


def inchToMeter(inch):
    return inch * 0.0254

class LimeLightConstants:
    limelight_name = "limelight-luxo"
    BARGE_TAG_HEIGHT = inchToMeter(73.54)
    REEF_TAG_HEIGHT = inchToMeter(12.13)
    STATION_TAG_HEIGHT = inchToMeter(58.50)
    SIDE_TAG_HEIGHT = inchToMeter(51.25)
    # april tag data - id, x, y, direction, height
    april_tag_data = [(1,inchToMeter(657.37), inchToMeter(25.80),126,STATION_TAG_HEIGHT),
                      (2,inchToMeter(657.37), inchToMeter(291.20),234,STATION_TAG_HEIGHT),
                      (3,inchToMeter(455.15), inchToMeter(317.15),270,SIDE_TAG_HEIGHT),
                      (4,inchToMeter(365.20), inchToMeter(241.64),0,BARGE_TAG_HEIGHT),
                      (5,inchToMeter(365.20), inchToMeter(75.39),0,BARGE_TAG_HEIGHT),
                      (6,inchToMeter(530.49), inchToMeter(130.17),300,REEF_TAG_HEIGHT),
                      (7,inchToMeter(546.87), inchToMeter(158.50),0,REEF_TAG_HEIGHT),
                      (8,inchToMeter(530.49), inchToMeter(186.83),60,REEF_TAG_HEIGHT),
                      (9,inchToMeter(497.77), inchToMeter(186.83),120,REEF_TAG_HEIGHT),
                      (10,inchToMeter(481.39), inchToMeter(158.50),180,REEF_TAG_HEIGHT),
                      (11,inchToMeter(497.77), inchToMeter(130.17),240,REEF_TAG_HEIGHT),
                      (12,inchToMeter(33.51), inchToMeter(25.80),54,STATION_TAG_HEIGHT),
                      (13,inchToMeter(33.51), inchToMeter(291.20),306,STATION_TAG_HEIGHT),
                      (14,inchToMeter(325.68), inchToMeter(241.64),180,BARGE_TAG_HEIGHT),
                      (15,inchToMeter(325.68), inchToMeter(75.39),180,BARGE_TAG_HEIGHT),
                      (16,inchToMeter(235.73), inchToMeter(-0.15),90,SIDE_TAG_HEIGHT),
                      (17,inchToMeter(160.39), inchToMeter(130.17),240,REEF_TAG_HEIGHT),
                      (18,inchToMeter(144.00), inchToMeter(158.50),180,REEF_TAG_HEIGHT),
                      (19,inchToMeter(160.39), inchToMeter(186.83),120,REEF_TAG_HEIGHT),
                      (20,inchToMeter(193.10), inchToMeter(186.83),60,REEF_TAG_HEIGHT),
                      (21,inchToMeter(209.49), inchToMeter(158.50),0,REEF_TAG_HEIGHT),
                      (22,inchToMeter(193.10), inchToMeter(130.17),300,REEF_TAG_HEIGHT)]

    LEFT_L3_OFFSET = 0.2
    RIGHT_L3_OFFSET = 0.33 - LEFT_L3_OFFSET
    BACK_L3_OFFSET = -0.43

    def getTagTranslation(self, tagId : int) -> Translation2d:
        return Translation2d(LimeLightConstants.april_tag_data[tagId - 1][1],
                      LimeLightConstants.april_tag_data[tagId - 1][2])
    def getTagAngle(self, tagId: int)->float:
        return LimeLightConstants.april_tag_data[tagId-1][3]
    def getTagPose(self, tagId: int) -> Pose2d:
        return Pose2d(self.getTagTranslation(tagId),Rotation2d.fromDegrees(self.getTagAngle(tagId)))
    def getTagRelativePositon(self,tagId: int,x: float,y: float) -> Pose2d:
        tagTranslation = self.getTagTranslation(tagId)
        angle = self.getTagAngle(tagId)
        r = Translation2d(x,y).rotateBy(Rotation2d.fromDegrees(angle))
        return Pose2d(tagTranslation + r, Rotation2d.fromDegrees(wpimath.inputModulus(180+angle,-180,180)))

    def getLeftL3Position(self, tagId:int) -> Pose2d:
        return self.getTagRelativePositon(tagId,self.BACK_L3_OFFSET, self.LEFT_L3_OFFSET)
    def getRightL3Position(self, tagId:int) -> Pose2d:
        return self.getTagRelativePositon(tagId,self.BACK_L3_OFFSET, self.RIGHT_L3_OFFSET)


class ModuleConstants:
    driveKS = 0.56548
    driveKV = 3.7091
    driveKA = 0.85702
    # Angle Motor PID Values
    angle_kp = 0.01
    angle_ki = 0.0
    angle_kd = 0.0
    angle_kf = 0.0
    # Drive Motor PID Values
    drive_kp = 1
    drive_ki = 0.0
    drive_kd = 0.0
    drive_kf = 1

    kWheelDiameterMeters = 0.095
    kDriveMotorGearRatio = 6.75
    kTurningMotorGearRatio = 150 / 7
    kDriveEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60
    # Drive Motor Conversion Factors #
    driveConversionPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDriveMotorGearRatio
    driveConversionVelocityFactor = driveConversionPositionFactor / 60.0
    angleConversionFactor = 360.0 / kTurningMotorGearRatio
    kPTurning = 0.2


class DriveConstants:
    slowDriveMultiplier = 0.4
    # Chassis
    kTrackWidth = 0.75
    kWheelBase = 0.75
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    )

    swerve_max_speed = 4

    kTeleDriveMaxAccelerationUnitsPerSecond = 3
    kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3

    # FrontLeft
    kFrontLeftDriveMotorPort = 2
    kFrontLeftTurningMotorPort = 1
    kFrontLeftTurningEncoderReversed = True
    kFrontLeftDriveEncoderReversed = True
    kFrontLeftDriveAbsoluteEncoderPort = 9
    kFrontLeftDriveAbsoluteEncoderReversed = False
    kFrontLeftDriveAbsoluteEncoderOffset = 356.22072

    # FrontRight
    kFrontRightDriveMotorPort = 7
    kFrontRightTurningMotorPort = 8
    kFrontRightTurningEncoderReversed = True
    kFrontRightDriveEncoderReversed = True
    kFrontRightDriveAbsoluteEncoderPort = 10
    kFrontRightDriveAbsoluteEncoderReversed = False
    kFrontRightDriveAbsoluteEncoderOffset = 6.6798

    # BackLeft
    kBackLeftDriveMotorPort = 4
    kBackLeftTurningMotorPort = 3
    kBackLeftTurningEncoderReversed = True
    kBackLeftDriveEncoderReversed = False
    kBackLeftDriveAbsoluteEncoderPort = 12
    kBackLeftDriveAbsoluteEncoderReversed = False
    kBackLeftDriveAbsoluteEncoderOffset = 239.854

    # BackRight
    kBackRightDriveMotorPort = 5
    kBackRightTurningMotorPort = 6
    kBackRightTurningEncoderReversed = True
    kBackRightDriveEncoderReversed = False
    kBackRightDriveAbsoluteEncoderPort = 11
    kBackRightDriveAbsoluteEncoderReversed = False
    kBackRightDriveAbsoluteEncoderOffset = 62.9298

    # Constants
    kPhysicalMaxSpeedMetersPerSecond = 4.6
    kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * math.pi

    kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
    kTeleDriveMaxAngularSpeedRadiansPerSecond = (
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4
    )
    kTeleDriveMaxAccelerationUnitsPerSecond = 3
    kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3


class AutoConstants:
    kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4
    kMaxAngularSpeedRadiansPerSecond = (
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10
    )
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 4
    kPXController = 1.5
    kPYController = 1.5
    kPThetaController = 3
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared
    )


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1

    # stick drift values
    kStickDriftLX = 0.1
    kStickDriftLY = 0.1
    kStickDriftRX = 0.1
