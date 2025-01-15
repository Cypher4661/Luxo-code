from rev import CANSparkMax
from enum import Enum


class Usage(Enum):
    kAll = 1
    kPositionOnly = 2
    kVelocityOnly = 3
    kMinimal = 4


# Sets motor usage for a Spark Max motor controller #
class CANSparkMaxUtil:
    @staticmethod
    def set_spark_max_bus_usage(
        motor: CANSparkMax, usage: Usage, enable_following: bool = False
    ):
        if enable_following:
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 10)
        else:
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 500)

        if usage == Usage.kAll:
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20)
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20)
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 50)
        elif usage == Usage.kPositionOnly:
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 500)
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20)
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 500)
        elif usage == Usage.kVelocityOnly:
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20)
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 500)
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 500)
        elif usage == Usage.kMinimal:
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 500)
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 500)
            motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 500)
