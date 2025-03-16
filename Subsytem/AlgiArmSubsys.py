from commands2 import Subsystem
import rev
import wpilib
from wpiutil import SendableBuilder
from Constants import AlgiSubsys
from wpilib import SmartDashboard
from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
from phoenix6.configs import TalonFXConfiguration
from commands2 import Subsystem
import phoenix6
from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue
from wpilib import SmartDashboard
import wpilib
from wpiutil import SendableBuilder
from Constants import CorralIntake


class algiArmSubsys(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # add constants
        self.motor1 = self.motor_config(
            rev.SparkMax(AlgiSubsys.motor1_id, AlgiSubsys.motor1_type), False
        )
        self.motor2 = self.motor_config(
            rev.SparkMax(AlgiSubsys.motor2_id, AlgiSubsys.motor2_type), True
        )

        self.motor1_controller = self.motor1.getClosedLoopController()
        self.motor2_controller = self.motor2.getClosedLoopController()

        self.encoder = self.motor1.getEncoder()

        self.limit = wpilib.DigitalInput(AlgiSubsys.limit_id)
        self.input = wpilib.DigitalInput(AlgiSubsys.revcoder)
        self.absolute_encoder = wpilib.DutyCycleEncoder(self.input)
        SmartDashboard.putData(self)
        SmartDashboard.putNumber('curent angle in degree', self.get_current_degree())
        self.rest_encoder()

    def motor_config(self, motor: rev.SparkMax, direction: bool) -> rev.SparkMax:
        config = rev.SparkMaxConfig()
        config.inverted(direction)
        config.setIdleMode(rev.SparkMaxConfig.IdleMode.kCoast)
        config.closedLoop.pidf(
            AlgiSubsys.kp, AlgiSubsys.ki, AlgiSubsys.kd, AlgiSubsys.kf
        )

        config.closedLoop.smartMotion.maxAcceleration(AlgiSubsys.maxAcceleration)
        config.closedLoop.smartMotion.maxVelocity(AlgiSubsys.maxVelocity)
        config.closedLoop.smartMotion.minOutputVelocity(AlgiSubsys.minVelocity)

        config.closedLoop.maxMotion.maxAcceleration(AlgiSubsys.maxAcceleration)
        config.closedLoop.maxMotion.maxVelocity(AlgiSubsys.maxVelocity)

        config.closedLoop.maxOutput(AlgiSubsys.maxPower)
        config.closedLoop.minOutput(-AlgiSubsys.maxPower)

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

    def periodic(self):
        SmartDashboard.putNumber('curent angle in degree', self.get_current_degree())
        SmartDashboard.putBoolean('algi at limit', self.at_limit())
        SmartDashboard.putNumber('algi abs', self.absolute_encoder.get())
        return super().periodic()

    def rest_encoder(self) -> None:
        self.encoder.setPosition(0)
        encoder = self.motor2.getEncoder()
        encoder.setPosition(0)
        # SmartDashboard.putNumber("alge arm init angle", pose)

    def at_limit(self) -> bool:
        return not self.limit.get()

    def motor_to_position(self, angle: float) -> None:
        self.motor1_controller.setReference(
            angle, rev.SparkLowLevel.ControlType.kPosition
        )
        self.motor2_controller.setReference(
            angle, rev.SparkLowLevel.ControlType.kPosition
        )

    def stop(self) -> None:
        self.motor1.set(0)
        self.motor2.set(0)

    def rotation_to_degrees(self, rotation: float) -> float:
        return rotation * 360

    def degrees_to_rotation(self, angle) -> float:
        return angle / 360

    def get_current_degree(self) -> float:
        return self.encoder.getPosition()
    
    def setPower(self, power: float) -> None:
        self.motor1.set(power)
        self.motor2.set(power)

    def get_motor_current(self) -> float:   
        return self.motor1.get_stator_current().value_as_double
    # def initSendable(self, builder: SendableBuilder) -> None:
    #     builder.addDoubleProperty(
    #         "Alge Arm Angle", self.get_current_degree, lambda x: None
    #     )
    #     builder.addDoubleProperty(
    #         "absolute_encoder ", self.absolute_encoder.get, lambda x: None
    #     )

    #     builder.addBooleanProperty(
    #         "absolute_encoder is con", self.absolute_encoder.isConnected, lambda x: None
    #     )

    #     return super().initSendable(builder)
