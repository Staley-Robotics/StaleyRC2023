import math
import ctre
from wpilib import Encoder
from wpimath.controller import *
from wpimath.trajectory import *
from wpimath.geometry import *
from wpimath.kinematics import *

kMaxSpeed = 2
kMaxSpeedX = 2
kMaxSpeedY = 2
kMaxSpeedR = 2 * math.pi

kWheelRadius: float = 0.05  # In Meters
kEncoderResolution = 2048
kModuleMaxAngularVelocity = 2 * math.pi
kModuleMaxAngularAcceleration = 2 * math.pi


class SwerveModule:
    m_drive_motor = None
    m_turning_motor = None

    m_drive_encoder = None
    m_turning_encoder = None

    # Gains are for example purposes only - must be determined for your own robot!
    m_drive_PID_controller = PIDController(1, 0, 0)

    # Gains are for example purposes only - must be determined for your own robot!
    m_turning_PID_controller = ProfiledPIDController(0.06, 0, 0, TrapezoidProfile.Constraints(
        kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration))

    # Gains are for example purposes only - must be determined for your own robot!
    m_drive_feedforward = SimpleMotorFeedforwardMeters(1, 3)
    m_turn_feedforward = SimpleMotorFeedforwardMeters(0.5, 0.5)

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

    def __init__(self,
                 drive_motor_channel,
                 turning_motor_channel,
                 drive_encoder_channel_a,
                 drive_encoder_channel_b,
                 turning_encoder_channel_a,
                 turning_encoder_channel_b):
        self.m_turningEncoder = Encoder(turning_encoder_channel_a, turning_encoder_channel_b)
        self.m_drive_encoder = Encoder(drive_encoder_channel_a, drive_encoder_channel_b)

        self.m_drive_motor = ctre.WPI_TalonFX(drive_motor_channel, "canivore1")
        self.m_turning_motor = ctre.WPI_TalonFX(turning_motor_channel, "canivore1")

        self.m_drive_motor.configFactoryDefault()
        self.m_turning_motor.configFactoryDefault()

        self.m_turning_motor.setSensorPhase(False)

        self.m_turning_motor.setInverted(True)

        self.m_drive_motor.setSelectedSensorPosition(0)
        self.m_turning_motor.setSelectedSensorPosition(0)

        self.m_drive_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor)
        self.m_turning_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.

        try:
            self.m_drive_encoder.setDistancePerPulse(2 * math.pi * kWheelRadius / kEncoderResolution)

            # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
            # This is the angle through an entire rotation (2 * pi) divided by the
            # encoder resolution.
            self.m_turning_encoder.setDistancePerPulse(2 * math.pi / kEncoderResolution)
        except:
            pass
        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.m_turning_PID_controller.setTolerance(500)
        self.m_turning_PID_controller.enableContinuousInput(-math.pi, math.pi)

    """
  * Returns the current state of the module.
  *
  * @return The current state of the module.
  """

    def getState(self):
        SwerveModuleState(self.m_drive_encoder.getRate(), Rotation2d(self.m_turning_encoder.getDistance()))

    """
  * Returns the current position of the module.
  *
  * @return The current position of the module.
  """

    def getPosition(self):
        # return SwerveModulePosition(self.m_driveEncoder.getDistance(),
        # Rotation2d(self.m_turningEncoder.getDistance()))
        pass

    """
  * Sets the desired state for the module.
  *
  * @param desiredState Desired state with speed and angle.
  """

    def setDesiredState(self, desiredState):
        # Constants
        wheelRadius = 0.05  # 5 CM
        wheelMetersPerRevolution = 2 * wheelRadius * math.pi
        driveTicksPerRevolution = 2048
        rotationTicksPerRevolution = 2048
        rotationGearRatio = 10  # 10 motor revolutions per 1 gear revolution

        # Get Current Angle Once (because it is used multiple times)
        currentRotationPos = self.m_turning_motor.getSelectedSensorPosition()  # Current Sensor Position
        currentRotationRadians = currentRotationPos / rotationGearRatio / rotationTicksPerRevolution * 2 * math.pi  # Radians
        currentRotation = Rotation2d(currentRotationRadians)  # Create Rotation2d Object

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = SwerveModuleState.optimize(desiredState, currentRotation)

        # Drive Velocity
        driveVelocityTicksPer100ms = self.m_drive_motor.getSelectedSensorVelocity()
        driveVelocityRevolutionsPerSec = driveVelocityTicksPer100ms * 10 / driveTicksPerRevolution
        driveVelocityMetersPerSecond = driveVelocityRevolutionsPerSec * wheelMetersPerRevolution

        # Calculate the drive output from the drive PID controller.
        # driveOutput = self.m_drivePIDController.calculate(driveVelocityMetersPerSecond, state.speed)
        driveOutput = min(state.speed, 1.0) * 4
        driveFeedforward = self.m_drive_feedforward.calculate(state.speed)

        # print( currentRotation.degrees() )
        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.m_turning_PID_controller.calculate(currentRotation.radians(), state.angle.radians())
        turnFeedforward = self.m_turn_feedforward.calculate(self.m_turning_PID_controller.getSetpoint().velocity)

        self.m_drive_motor.setVoltage(driveOutput)  # + driveFeedforward)
        self.m_turning_motor.setVoltage(turnOutput + turnFeedforward)
        # self.m_turningMotor.set( ctre.ControlMode.Position, int( state.angle.degrees() / 360 * kEncoderResolution * 10 ) )
