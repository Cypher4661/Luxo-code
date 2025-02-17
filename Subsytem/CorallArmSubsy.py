from commands2 import Subsystem
from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue
from phoenix6.hardware import TalonFX
import phoenix6.utils
from Constants import CoralSubsys
import phoenix6
import wpilib
from wpilib import SmartDashboard
from wpiutil import SendableBuilder
from wpilib import DutyCycleEncoder
from rev import AbsoluteEncoder


class corralArmSubsys(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.motor = self.config_motor(TalonFX(CoralSubsys.motor_id, ""), True)
        self.controller = phoenix6.controls.MotionMagicVoltage(0)
        self.stopController = phoenix6.controls.VoltageOut(0)
        self.limit = wpilib.DigitalInput(CoralSubsys.limit_id)
        self.encoder_input = wpilib.DigitalInput(CoralSubsys.revcoder)
        self.absolute_encoder = DutyCycleEncoder(self.encoder_input)
        self.reset_encoder()

    def config_motor(
        self, motor: phoenix6.hardware.TalonFX, inverted: bool
    ) -> phoenix6.hardware.TalonFX:
        # base talon config
        talonConfig = phoenix6.configs.TalonFXConfiguration()
        talonConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if inverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        talonConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        talonConfig.motor_output.peak_forward_duty_cycle = CoralSubsys.maxVolts / 12
        talonConfig.motor_output.peak_reverse_duty_cycle = CoralSubsys.maxVolts / 12
        talonConfig.voltage.peak_forward_voltage = CoralSubsys.maxVolts
        talonConfig.voltage.peak_reverse_voltage = CoralSubsys.maxVolts
        talonConfig.current_limits.supply_current_limit = CoralSubsys.maxAmper + 1
        talonConfig.current_limits.supply_current_limit_enable = True
        talonConfig.open_loop_ramps.duty_cycle_open_loop_ramp_period = (
            CoralSubsys.rampUp
        )
        talonConfig.open_loop_ramps.voltage_open_loop_ramp_period = CoralSubsys.rampUp
        talonConfig.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
            CoralSubsys.rampUp
        )
        talonConfig.closed_loop_ramps.voltage_closed_loop_ramp_period = (
            CoralSubsys.rampUp
        )
        talonConfig.feedback.rotor_to_sensor_ratio = CoralSubsys.gearRatio

        # smart motion config
        slot = talonConfig.slot0
        slot.k_s = CoralSubsys.ks
        slot.k_v = CoralSubsys.kv
        slot.k_a = CoralSubsys.ka
        slot.k_p = CoralSubsys.kp
        slot.k_i = CoralSubsys.ki
        slot.k_d = CoralSubsys.kd

        magic = talonConfig.motion_magic
        magic.motion_magic_cruise_velocity = CoralSubsys.velocity
        magic.motion_magic_acceleration = CoralSubsys.acceleration
        magic.motion_magic_jerk = CoralSubsys.jerk

        motor.configurator.apply(talonConfig)
        return motor

    def periodic(self):
        return super().periodic()

    def reset_encoder(self) -> None:
        pose = self.absolute_encoder.get() - CoralSubsys.encoder_offset
        self.motor.set_position(0)

    def at_limit(self) -> bool:
        return not self.limit.get()

    def motor_to_position(self, angle: float) -> None:
        rot = self.angle_to_rotation(angle) * CoralSubsys.gearRatio
        self.motor.set_control(self.controller.with_position(rot))

    def stop(self) -> None:
        self.motor.set_control(self.stopController)

    def rotation_to_angle(self, rotation: float) -> float:
        return rotation * 360

    def angle_to_rotation(self, angle: float) -> float:
        return angle / 360

    def get_current_angle(self) -> float:
        angle = self.rotation_to_angle(self.motor.get_rotor_position().value_as_double)
        return angle / CoralSubsys.gearRatio

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty(
            "Current Arm Angle", self.get_current_angle, lambda x: None
        )
        return super().initSendable(builder)
