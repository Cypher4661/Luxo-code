from commands2 import Subsystem
import phoenix6
from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue
from wpilib import SmartDashboard
import wpilib
from wpiutil import SendableBuilder
from Constants import CorralIntake

class corralIntake(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.motor:phoenix6.hardware.TalonFX = self.config_motor(phoenix6.hardware.TalonFX(CorralIntake.motor_id, ""), True)
        self.controller = phoenix6.controls.DutyCycleOut(0)
        self.limit = wpilib.DigitalInput(0)
    
    def config_motor(self, motor:phoenix6.hardware.TalonFX, inverted:bool) -> phoenix6.hardware.TalonFX:
        
        talonConfig = phoenix6.configs.TalonFXConfiguration()
        talonConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if inverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        talonConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        talonConfig.motor_output.peak_forward_duty_cycle = CorralIntake.maxVolts / 12
        talonConfig.motor_output.peak_reverse_duty_cycle = CorralIntake.maxVolts / 12
        talonConfig.voltage.peak_forward_voltage = CorralIntake.maxVolts
        talonConfig.voltage.peak_reverse_voltage = CorralIntake.maxVolts
        talonConfig.current_limits.supply_current_limit = CorralIntake.maxAmper + 1
        talonConfig.current_limits.supply_current_limit_enable = True
        talonConfig.open_loop_ramps.duty_cycle_open_loop_ramp_period = CorralIntake.rampUp
        talonConfig.open_loop_ramps.voltage_open_loop_ramp_period = CorralIntake.rampUp
        talonConfig.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
            CorralIntake.rampUp
        )
        talonConfig.closed_loop_ramps.voltage_closed_loop_ramp_period = (
            CorralIntake.rampUp
        )

        motor.configurator.apply(talonConfig)
        return motor

    def at_limit(self) -> bool:
        return self.limit.get()

    def duty_motor(self, power:float) -> None:
        self.motor.set_control(self.controller.with_output(power))

    def initSendable(self, builder: SendableBuilder) -> None:
        return super().initSendable(builder)
