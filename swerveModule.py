import math

from wpilib import PWMSparkMax, Encoder
from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

import drivetrain


kMaxSpeed = 2
kMaxSpeedX = 2
kMaxSpeedY = 2
kMaxSpeedR = 2 * math.pi

kWheelRadius:float = 0.05 #In Meters
kEncoderResolution = 2048
kModuleAngularVelocity = 2 * math.pi
kModuleMaxAngularAcceleration = 2 * math.pi


class SwerveModule:
  kWheelRadius = 0.0508
  kEncoderResolution = 4096

  kModuleMaxAngularVelocity = drivetrain.kMaxAngularSpeed
  kModuleMaxAngularAcceleration = 2 * math.pi #radians per second squared

  m_driveMotor = None
  m_turningMotor = None

  m_driveEncoder = None
  m_turningEncoder = None

  # Gains are for example purposes only - must be determined for your own robot!
  m_drivePIDController = PIDController(1, 0, 0)

  # Gains are for example purposes only - must be determined for your own robot!
  m_turningPIDController = ProfiledPIDController(1, 0, 0,TrapezoidProfile.Constraints(
  kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration))

  # Gains are for example purposes only - must be determined for your own robot!
  m_driveFeedforward = SimpleMotorFeedforwardMeters(1, 3)
  m_turnFeedforward = SimpleMotorFeedforwardMeters(1, 0.5)

  """
  * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
  *
  * @param driveMotorChannel PWM output for the drive motor.
  * @param turningMotorChannel PWM output for the turning motor.
  * @param driveEncoderChannelA DIO input for the drive encoder channel A
  * @param driveEncoderChannelB DIO input for the drive encoder channel B
  * @param turningEncoderChannelA DIO input for the turning encoder channel A
  * @param turningEncoderChannelB DIO input for the turning encoder channel B
  """
  def __init__ (self,
    driveMotorChannel,
    turningMotorChannel,
    driveEncoderChannelA,
    driveEncoderChannelB,
    turningEncoderChannelA,
    turningEncoderChannelB):
    m_driveMotor = PWMSparkMax(driveMotorChannel)
    m_turningMotor = PWMSparkMax(turningMotorChannel)

    m_driveEncoder = Encoder(driveEncoderChannelA, driveEncoderChannelB)
    m_turningEncoder = Encoder(turningEncoderChannelA, turningEncoderChannelB)

    # Set the distance per pulse for the drive encoder. We can simply use the
    # distance traveled for one rotation of the wheel divided by the encoder
    # resolution.
    m_driveEncoder.setDistancePerPulse(2 * math.pi * kWheelRadius / kEncoderResolution)

    # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    # This is the the angle through an entire rotation (2 * pi) divided by the
    # encoder resolution.
    m_turningEncoder.setDistancePerPulse(2 * math.pi / kEncoderResolution)

    # Limit the PID Controller's input range between -pi and pi and set the input
    # to be continuous.
    self.m_turningPIDController.enableContinuousInput(-math.pi, math.pi)

  """
  * Returns the current state of the module.
  *
  * @return The current state of the module.
  """
  def getState(self):
    SwerveModuleState(self.m_driveEncoder.getRate(), Rotation2d(self.m_turningEncoder.getDistance()))

  """
  * Returns the current position of the module.
  *
  * @return The current position of the module.
  """
  def getPosition(self):
    return SwerveModulePosition(self.m_driveEncoder.getDistance(), Rotation2d(self.m_turningEncoder.getDistance()))

  """
  * Sets the desired state for the module.
  *
  * @param desiredState Desired state with speed and angle.
  """
  def setDesiredState(self, desiredState):
    #Optimize the reference state to avoid spinning further than 90 degrees
    state = SwerveModuleState.optimize(desiredState, Rotation2d(self.m_turningEncoder.getDistance()))

    # Calculate the drive output from the drive PID controller.
    driveOutput = self.m_drivePIDController.calculate(self.m_driveEncoder.getRate(), state.speed)

    driveFeedforward = self.m_driveFeedforward.calculate(state.speed)

    # Calculate the turning motor output from the turning PID controller.
    turnOutput = self.m_turningPIDController.calculate(self.m_turningEncoder.getDistance(), state.angle.getRadians())

    turnFeedforward = self.m_turnFeedforward.calculate(self.m_turningPIDController.getSetpoint().velocity)

    self.m_driveMotor.setVoltage(driveOutput + driveFeedforward)
    self.m_turningMotor.setVoltage(turnOutput + turnFeedforward)
