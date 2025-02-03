from commands2 import Subsystem
import phoenix6
from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue
from Constants import AlgiIntake
from wpilib import SmartDashboard
from wpiutil import SendableBuilder


class algiIntake(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.motor: phoenix6.hardware.TalonFX = self.config_motor(
            phoenix6.hardware.TalonFX(AlgiIntake.motor_id, ""), True
        )
        self.controller = phoenix6.controls.DutyCycleOut(0)
        SmartDashboard.putData("Algi Intake", self)

    def config_motor(
        self, motor: phoenix6.hardware.TalonFX, inverted: bool
    ) -> phoenix6.hardware.TalonFX:
        talonConfig = phoenix6.configs.TalonFXConfiguration()
        talonConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if inverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        talonConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        talonConfig.motor_output.peak_forward_duty_cycle = AlgiIntake.maxVolts / 12
        talonConfig.motor_output.peak_reverse_duty_cycle = AlgiIntake.maxVolts / 12
        talonConfig.voltage.peak_forward_voltage = AlgiIntake.maxVolts
        talonConfig.voltage.peak_reverse_voltage = AlgiIntake.maxVolts
        talonConfig.current_limits.supply_current_limit = AlgiIntake.maxAmper + 1
        talonConfig.current_limits.supply_current_limit_enable = True
        talonConfig.open_loop_ramps.duty_cycle_open_loop_ramp_period = AlgiIntake.rampUp
        talonConfig.open_loop_ramps.voltage_open_loop_ramp_period = AlgiIntake.rampUp
        talonConfig.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
            AlgiIntake.rampUp
        )
        talonConfig.closed_loop_ramps.voltage_closed_loop_ramp_period = (
            AlgiIntake.rampUp
        )

        motor.configurator.apply(talonConfig)
        return motor

    def duty_motor(self, power) -> None:
        self.motor.set_control(self.controller.with_output(power))

    def initSendable(self, builder: SendableBuilder) -> None:
        return super().initSendable(builder)
