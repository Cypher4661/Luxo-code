from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ClosedLoopConfig
from phoenix6.hardware.cancoder import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import SimpleMotorFeedforwardMeters
from Constants import (
    ModuleConstants,
    DriveConstants,
)
from util.onboard_module_state import OnboardModuleState


class SwerveModule:
    def __init__(
        self,
        driveMotorId: int,
        turningMotorId: int,
        driveMotorReversed: bool,
        turningMotorReversed: bool,
        absoluteEncoderId: int,
        absoluteEncoderOffset: float,
        absoluteEncoderReversed: bool,
    ) -> None:
        self.feed_forward = SimpleMotorFeedforwardMeters(
            ModuleConstants.driveKS, ModuleConstants.driveKV, ModuleConstants.driveKA
        )

        self.absoluteEncoderOffset = absoluteEncoderOffset
        self.absoluteEncoderReversed = absoluteEncoderReversed
        self.absoluteEncoder = CANcoder(absoluteEncoderId)

        # Drive motor configuration
        self.driveMotor = SparkMax(driveMotorId, SparkLowLevel.MotorType.kBrushless)
        self.driveMotorReversed = driveMotorReversed

        # Turning motor configuration
        self.turningMotor = SparkMax(turningMotorId, SparkLowLevel.MotorType.kBrushless)
        self.turningMotorReversed = turningMotorReversed

        # Configure encoders and motors
        self.config_driveMotor()
        self.config_turningMotor()

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()
        self.reset_to_absolute()

        self.drive_controller = self.driveMotor.getClosedLoopController()
        self.turning_controller = self.turningMotor.getClosedLoopController()

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driveEncoder.getPosition(), self.get_angle())

    # Sync the internal motor encoder with the absolute encoder
    def reset_to_absolute(self):
        absolute_position = (
            self.absoluteEncoder.get_absolute_position().value * 360
            - self.absoluteEncoderOffset
        )
        desired_state = OnboardModuleState.optimize(
            SwerveModuleState(0, Rotation2d.fromDegrees(absolute_position)),
            self.getState().angle,
            False,
        )
        self.turningEncoder.setPosition(desired_state.angle.degrees())

    # Setting up the turning motor and configuring it
    def config_turningMotor(self):
        config = SparkMaxConfig()
        config.inverted(self.turningMotorReversed)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        config.closedLoop.pidf(
            ModuleConstants.angle_kp,
            ModuleConstants.angle_ki,
            ModuleConstants.angle_kd,
            ModuleConstants.angle_kf,
        )
        config.closedLoop.setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        )
        config.encoder.positionConversionFactor(ModuleConstants.angleConversionFactor)

        self.turningMotor.configure(
            config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    # Setting up the driving motor and configuring it
    def config_driveMotor(self):
        config = SparkMaxConfig()
        config.inverted(self.driveMotorReversed)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        config.closedLoop.pidf(
            ModuleConstants.drive_kp,
            ModuleConstants.drive_ki,
            ModuleConstants.drive_kd,
            ModuleConstants.drive_kf,
        )
        config.closedLoop.setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        )
        config.encoder.velocityConversionFactor(
            ModuleConstants.driveConversionVelocityFactor
        )
        config.encoder.positionConversionFactor(
            ModuleConstants.driveConversionPositionFactor
        )

        self.driveMotor.configure(
            config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.driveEncoder.getVelocity(), self.get_angle())

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.turningEncoder.getPosition())

    def setDesiredState(self, state: SwerveModuleState, is_open_loop: bool) -> None:
        desired_state = OnboardModuleState.optimize(state, self.getState().angle)
        if abs(desired_state.angle.degrees() - self.turningEncoder.getPosition()) > 0.6:
            self.turning_controller.setReference(
                desired_state.angle.degrees(),
                SparkLowLevel.ControlType.kPosition,
            )
        else:
            self.turningMotor.set(0)
        self.set_speed(desired_state, is_open_loop)

    def set_speed(self, desired_state: SwerveModuleState, is_open_loop: bool) -> None:
        if (
            is_open_loop
            and abs(desired_state.speed) <= DriveConstants.swerve_max_speed * 0.05
        ):
            self.driveMotor.set(0)
        elif is_open_loop:
            percent_output = desired_state.speed
            percent_output = pow(percent_output, 2)*(abs(percent_output)/percent_output)
            if abs(percent_output) >= 1:
                percent_output = abs(percent_output)/percent_output
            self.driveMotor.set(percent_output)
        else:
            self.drive_controller.setReference(
                desired_state.speed,
                SparkLowLevel.ControlType.kVelocity,
            )
