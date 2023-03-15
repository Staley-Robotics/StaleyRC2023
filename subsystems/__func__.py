import math
from wpimath.geometry import Rotation2d

def getTicksFromRotation( rotation:Rotation2d, ticksPerRotation:float ) -> int:
    radians:float = rotation.radians()
    ticks:int = int( radians * ticksPerRotation / ( 2 * math.pi ) )
    return ticks

def getRotationFromTicks( ticks:float, ticksPerRotation:float ) -> Rotation2d:
    radians:float = ticks * ( 2 * math.pi ) / ticksPerRotation
    rotation = Rotation2d( value=radians )
    return rotation

def getVelocityTp100msToMps( ticksPer100ms:float, ticksPerRotation:float, radius:float, gearRatio: float = 1.0 ) -> float:
    ticksPerSec:float = ticksPer100ms * 10
    rotationsPerSec:float = ticksPerSec / ticksPerRotation
    metersPerSec:float = rotationsPerSec * ( 2 * math.pi * radius * gearRatio)
    return metersPerSec

def getVelocityMpsToTp100ms( metersPerSec:float, ticksPerRotation:float, radius:float, gearRatio: float = 1.0 ) -> float:
    rotationsPerSec:float = metersPerSec / ( 2 * math.pi * radius * gearRatio )
    ticksPerSec:float = rotationsPerSec * ticksPerRotation
    ticksPer100ms:float = ticksPerSec / 10
    return ticksPer100ms

def getDistanceTicksToMeters( ticks:float, ticksPerRotation:float, radius:float, gearRatio: float = 1.0 ) -> float:
    rotations:float = ticks / ticksPerRotation
    meters:float = rotations * ( 2 * math.pi * radius * gearRatio )
    return meters

def getDistanceMetersToTicks( meters:float, ticksPerRotation:float, radius:float, gearRatio: float = 1.0 ) -> float:
    rotations:float = meters / ( 2 * math.pi * radius * gearRatio )
    ticks:float = rotations * ticksPerRotation
    return ticks

def getContinuousInputMeasurement( currentSetpoint:float, targetSetpoint:float, measurement:float ) -> float:
    targetSetpoint = math.remainder(targetSetpoint, measurement)
    remainder:float = currentSetpoint % measurement
    adjustedAngleSetpoint:float = targetSetpoint + (currentSetpoint - remainder)

    if (adjustedAngleSetpoint - currentSetpoint > (measurement/2)):
        adjustedAngleSetpoint -= measurement
    elif (adjustedAngleSetpoint - currentSetpoint < (-measurement/2)):
        adjustedAngleSetpoint += measurement

    return adjustedAngleSetpoint
