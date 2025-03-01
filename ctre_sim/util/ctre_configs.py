from phoenix6.configs import (
    CANcoderConfiguration,
    SensorTimeBase,
    AbsoluteSensorRange,
    SensorInitializationStrategy,
)


class CTREConfigs:
    swerveCanCoderConfig = CANcoderConfiguration()

    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360
    swerveCanCoderConfig.sensorDirection = False
    swerveCanCoderConfig.initializationStrategy = (
        SensorInitializationStrategy.BootToAbsolutePosition
    )
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond
