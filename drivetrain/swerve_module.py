import math
import ctre
from wpilib import Encoder
from wpimath.controller import *
from wpimath.trajectory import *
from wpimath.geometry import *
from wpimath.kinematics import *

max_speed = 2
max_speed_x = 2
max_speed_y = 2
max_speed_r = 2 * math.pi

wheel_radius: float = 0.05  # In Meters
encoder_resolution = 2048
max_velocity = 2 * math.pi
max_acceleration = 2 * math.pi


class SwerveModule:
    drive_motor = None
    turning_motor = None

    drive_encoder = None
    turning_encoder = None

    drive_PID_controller = PIDController(1, 0, 0)

    turning_PID_controller = ProfiledPIDController(0.06, 0, 0, TrapezoidProfile.Constraints(
        max_velocity, max_acceleration))

    drive_feedforward = SimpleMotorFeedforwardMeters(1, 3)
    turn_feedforward = SimpleMotorFeedforwardMeters(0.5, 0.5)

    location: Translation2d

    def __init__(self, drive_channel, turning_channel, drive_encoder_a, drive_encoder_b, turning_encoder_a,
                 turning_encoder_b, location):
        self.location = location
        self.turning_encoder = Encoder(turning_encoder_a, turning_encoder_b)
        self.drive_encoder = Encoder(drive_encoder_a, drive_encoder_b)

        self.drive_motor = ctre.WPI_TalonFX(drive_channel, "canivore1")
        self.turning_motor = ctre.WPI_TalonFX(turning_channel, "canivore1")

        self.drive_motor.configFactoryDefault()
        self.turning_motor.configFactoryDefault()

        self.turning_motor.setSensorPhase(False)

        self.turning_motor.setInverted(True)

        self.drive_motor.setSelectedSensorPosition(0)
        self.turning_motor.setSelectedSensorPosition(0)

        self.drive_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor)
        self.turning_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor)

        try:
            self.drive_encoder.setDistancePerPulse(2 * math.pi * wheel_radius / encoder_resolution)
            self.turning_encoder.setDistancePerPulse(2 * math.pi / encoder_resolution)
        except:
            pass
        self.turning_PID_controller.setTolerance(500)
        self.turning_PID_controller.enableContinuousInput(-math.pi, math.pi)

    def getState(self):
        return SwerveModuleState(self.drive_encoder.getRate(), Rotation2d(self.turning_encoder.getDistance()))

    def getPosition(self):
        return SwerveModulePosition(self.drive_encoder.getDistance(), Rotation2d(self.turning_encoder.getDistance()))

    def setDesiredState(self, desired_state):
        state = SwerveModuleState.optimize(desired_state, Rotation2d(
            self.turning_motor.getSelectedSensorPosition() * 2 * math.pi / 2048))
        drive_output = self.drive_PID_controller.calculate(
            self.drive_motor.getSelectedSensorVelocity() * 0.1 * 2 * math.pi / 2048, state.speed)

        drive_feedforward = self.drive_feedforward.calculate(state.speed)

        turn_output = self.turning_PID_controller.calculate(
            self.turning_motor.getSelectedSensorPosition() * 2 * math.pi / 2048, state.angle.radians() * 10)
        turn_feedforward = self.turn_feedforward.calculate(self.turning_PID_controller.getSetpoint().velocity)

        self.drive_motor.setVoltage(drive_output + drive_feedforward)
        self.turning_motor.setVoltage(turn_output + turn_feedforward)
