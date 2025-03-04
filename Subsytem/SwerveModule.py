from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ClosedLoopConfig
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.configs import CANcoderConfiguration
from phoenix6.signals import SensorDirectionValue
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import SimpleMotorFeedforwardMeters
from Constants import (
    ModuleConstants,
    DriveConstants,
)
from util.onboard_module_state import OnboardModuleState
import wpiutil
from wpilib import SmartDashboard


class SwerveModule(wpiutil._wpiutil.Sendable):
    def __init__(
        self,
        name,
        driveMotorId: int,
        turningMotorId: int,
        driveMotorReversed: bool,
        turningMotorReversed: bool,
        absoluteEncoderId: int,
        absoluteEncoderOffset: float,
        absoluteEncoderReversed: bool,
    ) -> None:
        super().__init__()
        self.name = name
        self.feed_forward = SimpleMotorFeedforwardMeters(
            ModuleConstants.driveKS, ModuleConstants.driveKV, ModuleConstants.driveKA
        )

        self.absoluteEncoderOffset = absoluteEncoderOffset
        self.absoluteEncoderReversed = absoluteEncoderReversed
        self.absoluteEncoder = CANcoder(absoluteEncoderId)
        cfg = CANcoderConfiguration()
        
        cfg.magnet_sensor.with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE 
                                                if absoluteEncoderReversed else 
                                                SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        self.absoluteEncoder.configurator.apply(cfg)

        # Drive motor configuration
        self.driveMotor = SparkMax(driveMotorId, SparkLowLevel.MotorType.kBrushless)
        self.driveConfig = SparkMaxConfig()
        self.driveMotorReversed = driveMotorReversed

        # Turning motor configuration
        self.turningMotor = SparkMax(turningMotorId, SparkLowLevel.MotorType.kBrushless)
        self.turningMotorReversed = turningMotorReversed
        self.turningConfig = SparkMaxConfig()

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

    def getAbsEncoder(self):
        return self.absoluteEncoder.get_absolute_position().value * 360 
    
    # Sync the internal motor encoder with the absolute encoder
    def reset_to_absolute(self):
        absolute_position = self.getAbsEncoder()- self.absoluteEncoderOffset
        self.turningEncoder.setPosition(absolute_position)

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
        self.turningConfig = config

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
        self.driveConfig = config

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.driveEncoder.getVelocity(), self.get_angle())

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.turningEncoder.getPosition())

    def get_neg_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.turningEncoder.getPosition()).__neg__()

    def setDesiredState(self, state: SwerveModuleState, is_open_loop: bool) -> None:
#        state.angle = state.angle.__neg__()
        desired_state = OnboardModuleState.optimize(state, self.getState().angle)
        if abs(desired_state.angle.degrees() - self.turningEncoder.getPosition()) > 0.6:
            self.turning_controller.setReference(
                desired_state.angle.degrees(),
                SparkLowLevel.ControlType.kPosition,
            )
        else:
            self.turningMotor.set(0)
        self.set_speed(desired_state.speed, is_open_loop)

    def set_speed(self, speed: float, is_open_loop: bool) -> None:
        if is_open_loop:
            if abs(speed) <= DriveConstants.swerve_max_speed * 0.05:
                self.driveMotor.set(0)
            else:
                percent_output = speed / DriveConstants.swerve_max_speed
                percent_output = speed * speed if speed > 0 else -speed * speed
                percent_output = 1 if percent_output > 1 else -1 if percent_output < -1 else percent_output
                self.driveMotor.set(percent_output)
        else:
            if abs(speed) < 0.05:
                self.driveMotor.set(0)
            else:
                volts = ModuleConstants.driveKS if speed > 0 else -ModuleConstants.driveKS
                volts = volts + speed * ModuleConstants.driveKV
                self.driveMotor.set(volts/12)

    def setBrake(self, brake:bool):
        self.driveConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake if brake else SparkMaxConfig.IdleMode.kCoast)
        self.turningConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake if brake else SparkMaxConfig.IdleMode.kCoast)
        self.turningMotor.configure(
            self.turningConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.driveMotor.configure(
            self.driveConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )


    def initSendable(self, builder):
        builder.addDoubleProperty('angle', self.turningEncoder.getPosition, lambda  x: None)
        builder.addDoubleProperty('abs', self.getAbsEncoder, lambda  x: None)
        builder.addDoubleProperty('velocity', self.driveEncoder.getVelocity, lambda  x: None)