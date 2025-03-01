from phoenix5.sensors import CANCoder, CANCoderStatusFrame
from enum import Enum


class CCUsage(Enum):
    kAll = 1
    kSensorDataOnly = 2
    kFaultsOnly = 3
    kMinimal = 4


# Sets status frames for the CTRE CANCoder. #
class CANCoderUtil:
    @staticmethod
    def set_cancoder_bus_usage(cancoder: CANCoder, usage: CCUsage):
        if usage == CCUsage.kAll:
            cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10)
            cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10)
        elif usage == CCUsage.kSensorDataOnly:
            cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10)
            cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100)
        elif usage == CCUsage.kFaultsOnly:
            cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100)
            cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10)
        elif usage == CCUsage.kMinimal:
            cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100)
            cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100)
