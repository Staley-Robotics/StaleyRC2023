# import math
# import ctre
# import ctre.sensors
# from wpimath.geometry import Rotation2d
# from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
#
# kMaxSpeed = 2
#
# kWheelRadius: float = 0.05  # In Meters
# kDriveEncoderResolution = 2048
# kTurningEncoderResolution = 4096
#
#
# class SwerveModule:
#     m_driveMotor = None
#     m_turningMotor = None
#     m_turningEncoder = None
#
#     """
#   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
#   *
#   * @param driveMotorChannel PWM output for the drive motor.
#   * @param turningMotorChannel PWM output for the turning motor.
#   * @param driveEncoderChannelA DIO input for the drive encoder channel A
#   * @param driveEncoderChannelB DIO input for the drive encoder channel B
#   * @param turningEncoderChannelA DIO input for the turning encoder channel A
#   * @param turningEncoderChannelB DIO input for the turning encoder channel B
#   """
#
#     def __init__(self,
#                  dMotorId,
#                  tMotorId,
#                  tEncoderId,
#                  dMotorReversed=False,
#                  dEncoderReversed=False,
#                  tMotorReversed=False,
#                  tEncoderReversed=False,
#                  tEncoderOffset=0.0):
#         # Create Devices
#         self.m_driveMotor = ctre.WPI_TalonFX(dMotorId, "canivore1")
#         self.m_turningMotor = ctre.WPI_TalonFX(tMotorId, "canivore1")
#         self.m_turningEncoder = ctre.sensors.CANCoder(tEncoderId, "canivore1")
#
#         # Reset Devices To Factory
#         self.m_driveMotor.configFactoryDefault()
#         self.m_turningMotor.configFactoryDefault()
#         self.m_turningEncoder.configFactoryDefault()
#
#         # Turning Encoder Configuration
#         self.m_turningEncoder.configSensorInitializationStrategy(
#             ctre.sensors.SensorInitializationStrategy.BootToAbsolutePosition)
#         self.m_turningEncoder.configAbsoluteSensorRange(ctre.sensors.AbsoluteSensorRange.Signed_PlusMinus180)
#         self.m_turningEncoder.configSensorDirection(
#             tEncoderReversed)  # Do we need to inverse this for certain positions?
#         self.m_turningEncoder.configMagnetOffset(tEncoderOffset)
#         self.m_turningEncoder.setPositionToAbsolute()
#
#         # Turning Motor Sensor Configuration
#         self.m_turningMotor.configRemoteFeedbackFilter(self.m_turningEncoder, 0)
#         self.m_turningMotor.configSelectedFeedbackSensor(ctre.RemoteFeedbackDevice.RemoteSensor0, 0)
#         self.m_turningMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.None_, 1)
#
#         # Drive Motor Sensor Configuration
#         ### DO we need this for open loop?
#         # self.m_driveMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor)
#
#         # Motor Inverting
#         self.m_driveMotor.setInverted(dMotorReversed)
#         self.m_driveMotor.setSensorPhase(dEncoderReversed)
#         self.m_turningMotor.setInverted(tMotorReversed)
#
#         # Motor Breaking
#         self.m_driveMotor.setNeutralMode(ctre.NeutralMode.Coast)
#         self.m_turningMotor.setNeutralMode(ctre.NeutralMode.Brake)
#
#         # Drive PID
#         ### DO we need this for open loop?
#         # self.m_driveMotor.config_kF(0, 0) # 0.065
#         # self.m_driveMotor.config_kP(0, 0) # 0.15
#         # self.m_driveMotor.config_kI(0, 0)
#         # self.m_driveMotor.config_kD(0, 0)
#
#         # Rotate PID
#         self.m_turningMotor.config_kF(0, 0)  # 0.14
#         self.m_turningMotor.config_kP(0, 0.7)  # 1.2
#         self.m_turningMotor.config_kI(0, 0)
#         self.m_turningMotor.config_kD(0, 0)
#
#         # Motion Magic
#         #### NOT USING
#         # self.m_turningMotor.configMotionAcceleration(4096)
#         # self.m_turningMotor.configMotionCruiseVelocity(5108)
#
#     """
#   * Returns the current state of the module.
#   *
#   * @return The current state of the module.
#   """
#
#     def getState(self):
#         vTp100ms = self.m_driveMotor.getSelectedSensorVelocity()
#         vTps = vTp100ms * 10
#         vRps = vTps / kTurningEncoderResolution
#         vMps = vRps * kWheelRadius * 2 * math.pi
#         angle = self.m_turningEncoder.getPosition()
#         rotation = Rotation2d().fromDegrees(angle)
#         return SwerveModuleState(vMps, rotation)
#
#     """
#   * Returns the current position of the module.
#   *
#   * @return The current position of the module.
#   """
#
#     def getPosition(self):
#         ticks = self.m_driveMotor.getSelectedSensorPosition()
#         meters = ticks / kDriveEncoderResolution
#         angle = self.m_turningEncoder.getPosition()
#         rotation = Rotation2d().fromDegrees(angle)
#         return SwerveModulePosition(meters, rotation)
#
#     """
#   * Sets the desired state for the module.
#   *
#   * @param desiredState Desired state with speed and angle.
#   """
#
#     def setDesiredState(self, desiredState):
#         # Get Current Angle Once (because it is used multiple times)
#         currentAngle = self.m_turningEncoder.getPosition()
#         currentRotation = Rotation2d().fromDegrees(currentAngle)
#
#         # Optimize the reference state to avoid spinning further than 90 degrees
#         state = SwerveModuleState.optimize(desiredState, currentRotation)
#
#         # Drive
#         driveOutput = state.speed / kMaxSpeed
#         self.m_driveMotor.set(ctre.ControlMode.PercentOutput, driveOutput)
#
#         # Turn Motor
#         turnDegrees = state.angle.degrees()
#         turnPosition = turnDegrees / 360 * kTurningEncoderResolution
#         self.m_turningMotor.set(ctre.ControlMode.Position, turnPosition)  # Could this be TurnDegrees?
