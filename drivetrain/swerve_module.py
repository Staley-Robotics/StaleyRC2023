import math
import ctre
import ctre.sensors
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
                 turning_sensor_channel,
                 offset=0):
#        self.m_turningEncoder = Encoder(turning_encoder_channel_a, turning_encoder_channel_b)
#        self.m_drive_encoder = Encoder(drive_encoder_channel_a, drive_encoder_channel_b)
        self.m_turning_encoder = ctre.sensors.WPI_CANCoder(turning_sensor_channel, "canivore1")

        self.m_drive_motor = ctre.WPI_TalonFX(drive_motor_channel, "canivore1")
        self.m_turning_motor = ctre.WPI_TalonFX(turning_motor_channel, "canivore1")

        self.m_drive_motor.configFactoryDefault()
        self.m_turning_motor.configFactoryDefault()
        self.m_turning_encoder.configFactoryDefault()

        self.m_turning_motor.setSensorPhase(True)
        self.m_turning_motor.setInverted(False)

        self.m_drive_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor)
        self.m_turning_encoder.configAbsoluteSensorRange(ctre.sensors.AbsoluteSensorRange.Unsigned_0_to_360)

        self.m_turning_motor.configRemoteFeedbackFilter(self.m_turning_encoder, 0)
        self.m_turning_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.RemoteSensor0)

        self.m_drive_motor.setSelectedSensorPosition(0)
        self.m_turning_motor.setSelectedSensorPosition(0)
        self.m_turning_encoder.setPosition(self.m_turning_encoder.getAbsolutePosition()-offset)
        #abs_pos = self.m_turning_encoder.getAbsolutePosition() #getSensorCollection().getIntegratedSensorAbsolutePosition()
        #self.m_turning_motor.setSelectedSensorPosition(abs_pos * 56.8888889)

        self.m_turning_motor.configClosedLoopPeriod(0, 20)
        self.m_turning_motor.config_kP(0, 0.03)
        #self.m_turning_motor.config_kD(0, .120)
        #self.m_turning_motor.config_kF(0, 0.1)
        self.m_turning_motor.configAllowableClosedloopError(0, 0.1)
        #self.m_turning_motor.

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
        self.m_turning_PID_controller.setTolerance(100)
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
        # Get Current Angle Once (because it is used multiple times)
        currentRotation = Rotation2d().fromDegrees(self.m_turning_encoder.getPosition())
        # Optimize the reference state to avoid spinning further than 90 degrees
        state = SwerveModuleState.optimize(desiredState, currentRotation)

        # Drive Velocity
        driveOutput = min(state.speed, 1.0) * 4
        self.m_drive_motor.setVoltage(driveOutput)  # + driveFeedforward)

        # Rotation
        print(self.m_drive_motor.getDeviceID(), self.m_turning_encoder.getAbsolutePosition(), self.m_turning_encoder.getPosition(), state.angle.degrees())
        self.m_turning_motor.set(ctre.ControlMode.Position, (state.angle.degrees() * 5.68888889) )
