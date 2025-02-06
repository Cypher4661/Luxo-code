from commands2 import Subsystem
import rev
import wpilib
from wpiutil import SendableBuilder
from Constants import AlgiSubsys
from wpilib import SmartDashboard


class algiArmSubsys(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # add constants
        self.motor1 = self.motor_config(
            rev.SparkMax(AlgiSubsys.motor1_id, AlgiSubsys.motor1_type), True
        )
        self.motor2 = self.motor_config(
            rev.SparkMax(AlgiSubsys.motor2_id, AlgiSubsys.motor2_type), False
        )

        self.motor1_controller = self.motor1.getClosedLoopController()
        self.motor2_controller = self.motor2.getClosedLoopController()

        self.encoder = self.motor1.getEncoder()

        self.limit = wpilib.DigitalInput(AlgiSubsys.limit_id)
        self.input = wpilib.DigitalInput(AlgiSubsys.revcoder)
        self.absolute_encoder = wpilib.DutyCycleEncoder(self.input)
        self.rest_encoder()
        SmartDashboard.putData("Algi Arm Subsystem", self)

    def motor_config(self, motor: rev.SparkMax, direction: bool) -> rev.SparkMax:
        config = rev.SparkMaxConfig()
        config.inverted(direction)
        config.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        config.closedLoop.pidf(
            AlgiSubsys.kp, AlgiSubsys.ki, AlgiSubsys.kd, AlgiSubsys.kf
        )

        config.closedLoop.smartMotion.maxAcceleration(AlgiSubsys.maxAcceleration)
        config.closedLoop.smartMotion.maxVelocity(AlgiSubsys.maxVelocity)
        config.closedLoop.smartMotion.minOutputVelocity(AlgiSubsys.minVelocity)

        config.closedLoop.setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        )
        config.encoder.positionConversionFactor(AlgiSubsys.convertion_factor)
        motor.configure(
            config,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )
        return motor

    def rest_encoder(self) -> None:
        pose = self.absolute_encoder.get() - AlgiSubsys.encoder_offset
        self.encoder.setPosition(pose)
        encoder = self.motor2.getEncoder()
        encoder.setPosition(pose)

    def at_limit(self) -> bool:
        return not self.limit.get()

    def motor_to_position(self, angle: float) -> None:
        self.motor1_controller.setReference(
            self.degrees_to_rotation(angle), rev.SparkLowLevel.ControlType.kPosition
        )
        self.motor2_controller.setReference(
            self.degrees_to_rotation(angle), rev.SparkLowLevel.ControlType.kPosition
        )

    def stop(self) -> None:
        self.motor1.set(0)
        self.motor2.set(0)

    def rotation_to_degrees(self, rotation: float) -> float:
        return rotation * 360

    def degrees_to_rotation(self, angle) -> float:
        return angle / 360

    def get_current_degree(self) -> float:
        return self.rotation_to_degrees(self.encoder.getPosition())

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty(
            "System Position", self.get_current_degree, lambda x: None
        )
        return super().initSendable(builder)
