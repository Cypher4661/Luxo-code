from commands2 import Subsystem
import phoenix6
from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue
from Constants import AlgaeIntake
from wpilib import SmartDashboard
from wpiutil import SendableBuilder


class AlgaeIntakeSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.motor: phoenix6.hardware.TalonFX = self.config_motor(
            phoenix6.hardware.TalonFX(AlgaeIntake.motor_id, ""), False
        )
        self.controller = phoenix6.controls.DutyCycleOut(0)

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
        talonConfig.motor_output.peak_forward_duty_cycle = AlgaeIntake.maxVolts / 12
        talonConfig.motor_output.peak_reverse_duty_cycle = -AlgaeIntake.maxVolts / 12
        talonConfig.voltage.peak_forward_voltage = AlgaeIntake.maxVolts
        talonConfig.voltage.peak_reverse_voltage = -AlgaeIntake.maxVolts
        talonConfig.current_limits.supply_current_limit = AlgaeIntake.maxAmper + 1
        talonConfig.current_limits.supply_current_limit_enable = True
        talonConfig.open_loop_ramps.duty_cycle_open_loop_ramp_period = AlgaeIntake.rampUp
        talonConfig.open_loop_ramps.voltage_open_loop_ramp_period = AlgaeIntake.rampUp
        talonConfig.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
            AlgaeIntake.rampUp
        )
        talonConfig.closed_loop_ramps.voltage_closed_loop_ramp_period = (
            AlgaeIntake.rampUp
        )

        motor.configurator.apply(talonConfig)
        return motor

    def setPower(self, power: float) -> None:
        self.motor.set_control(self.controller.with_output(power))

    def stop(self) -> None:
        self.setPower(0)

    def get_motor_velocity(self) -> float:
        return self.motor.get_rotor_velocity().value_as_double

    def get_motor_current(self) -> float:
        return self.motor.get_stator_current().value_as_double

    def initSendable(self, builder: SendableBuilder) -> None:
        pass
