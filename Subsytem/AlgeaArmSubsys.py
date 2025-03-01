from commands2 import Subsystem, InstantCommand
from rev import SparkLowLevel
import rev
import wpilib
from wpiutil import SendableBuilder
from Constants import AlgaeSubsys
from enum import Enum

class AlgaeArmPosition(Enum):
    UP = 0
    COLLECT = 70
    PROCESSOR = 30
    DROP_L2 = 25


class AlgaeArmSubsys(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # add constants
        self.motor1 = self.motor_config(rev.SparkMax(AlgaeSubsys.motor1_id, AlgaeSubsys.motor1_type), True)
        self.motor2 = self.motor_config(rev.SparkMax(AlgaeSubsys.motor2_id, AlgaeSubsys.motor2_type), False)

        self.motor1_controller = self.motor1.getClosedLoopController()
        self.motor2_controller = self.motor2.getClosedLoopController()
        self.encoder = self.motor1.getEncoder()

        self.limit = wpilib.DigitalInput(AlgaeSubsys.limit_id)
        self.input = wpilib.DigitalInput(AlgaeSubsys.revcoder)
        self.absolute_encoder = wpilib.DutyCycleEncoder(self.input)
        self.armPosition = AlgaeArmPosition.UP
        self.rest_encoder()

    def motor_config(self, motor: rev.SparkMax, direction: bool) -> rev.SparkMax:
        config = rev.SparkMaxConfig()
        config.inverted(direction)
        config.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        config.closedLoop.pidf(AlgaeSubsys.kp, AlgaeSubsys.ki, AlgaeSubsys.kd, AlgaeSubsys.kf)
        config.closedLoop.smartMotion.maxAcceleration(AlgaeSubsys.maxAcceleration)
        config.closedLoop.smartMotion.maxVelocity(AlgaeSubsys.maxVelocity)
        config.closedLoop.smartMotion.minOutputVelocity(AlgaeSubsys.minVelocity)
        config.closedLoop.maxMotion.maxAcceleration(AlgaeSubsys.maxAcceleration)
        config.closedLoop.maxMotion.maxVelocity(AlgaeSubsys.maxVelocity)
        config.closedLoop.maxOutput(AlgaeSubsys.maxPower)
        config.closedLoop.minOutput(-AlgaeSubsys.maxPower)
        config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        config.encoder.positionConversionFactor(AlgaeSubsys.convertion_factor)
        motor.configure(config,rev.SparkMax.ResetMode.kResetSafeParameters,rev.SparkMax.PersistMode.kPersistParameters)
        return motor

    def periodic(self):
        pass

    def rest_encoder(self) -> None:
        pos = 360*(1 - self.absolute_encoder.get() - AlgaeSubsys.encoder_offset)
        self.encoder.setPosition(pos)
        self.motor2.getEncoder().setPosition(pos)

    def at_limit(self) -> bool:
        return not self.limit.get()

    def motor_to_position(self, angle: float) -> None:
        self.motor1_controller.setReference(angle, SparkLowLevel.ControlType.kPosition)
        self.motor2_controller.setReference(angle, SparkLowLevel.ControlType.kPosition)

    def setPosition(self, position:AlgaeArmPosition):
        self.armPosition = position
        self.motor_to_position(position.value)

    def atPosition(self):
        return abs(self.get_current_degree() - self.armPosition.value) < AlgaeSubsys.deadband

    def stop(self) -> None:
        self.motor1.set(0)
        self.motor2.set(0)

    def get_current_degree(self) -> float:
        return self.encoder.getPosition()

    def setUpPosition(self):
        self.setPosition(AlgaeArmPosition.UP)

    def setCollectPosition(self):
        self.setPosition(AlgaeArmPosition.COLLECT)

    def setProcessorPosition(self):
        self.setPosition(AlgaeArmPosition.PROCESSOR)

    def setDropL2Position(self):
        self.setPosition(AlgaeArmPosition.DROP_L2)

    def setPositionValue(self, value):
        a = AlgaeArmPosition(value)
        if a:
            self.setPosition(a)

    def getCommand(self,toPosition:AlgaeArmPosition):
        match toPosition:
            case AlgaeArmPosition.UP:
                return InstantCommand(self.setUpPosition)
            case AlgaeArmPosition.PROCESSOR:
                return InstantCommand(self.setProcessorPosition)
            case AlgaeArmPosition.COLLECT:
                return InstantCommand(self.setCollectPosition)
            case AlgaeArmPosition.DROP_L2:
                return InstantCommand(self.setDropL2Position)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty("Algae Arm Angle", self.get_current_degree, lambda x: None)
        builder.addBooleanProperty("Algae Limit", self.at_limit, lambda x: None)
        builder.addDoubleProperty("Algae Arm Target", lambda: self.armPosition.value, self.setPositionValue)
        builder.addBooleanProperty("Algae at Position", self.atPosition, lambda x: None)
