#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

from math import atan2
import wpilib
import wpilib.drive
import ctre
import wpimath.controller
import math

class SwerveWheelSettings:
    def __init__(self) -> None:
        self.turnID = 0
        self.driveID = 0
        self.encoderID = 0
        self.P = 0
        self.I = 0
        self.D = 0
        self.encoderOffset = 0
        self.location = ""

class SwerveWheel:
    def __init__(self, settings) -> None:
        self.turnMotor = ctre.WPI_TalonFX(settings.turnID)
        self.driveMotor = ctre.WPI_TalonFX(settings.driveID)
        self.encoder = ctre.TalonSRX(settings.encoderID)
        self.encoder.configSelectedFeedbackSensor(ctre.TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute)
        self.pidController = wpimath.controller.PIDController(settings.P, settings.I, settings.D)
        self.pidController.enableContinuousInput(0, 360)
        self.location = settings.location
        self.encoderOffset = settings.encoderOffset

    def getCorrectedEncoder(self):
        value = self.encoder.getSelectedSensorPosition() - self.encoderOffset
        value = (value + 4096) if value < 0 else value
        return (value * 360 / 4096)

    def set(self, heading, turn, power):
        turn *= -90 if self.location == "front" else 90
        encoderData = self.getCorrectedEncoder()
        normalizedAngle = (heading + turn) % 360
        flippedAngle = (normalizedAngle + 180) % 360

        flipped = abs(encoderData - flippedAngle) < abs(encoderData - normalizedAngle)
        desiredAngle = flippedAngle if flipped else normalizedAngle

        calculatedPID = -self.pidController.calculate(encoderData, desiredAngle) / 180
        power *= -1 if flipped else 1
        self.turnMotor.set(ctre.ControlMode.PercentOutput, calculatedPID)
        self.driveMotor.set(ctre.ControlMode.PercentOutput, -power)
    
class SwerveDriveSettings:
    def __init__(self) -> None:
        self.FLSwerve = SwerveWheelSettings()
        self.FRSwerve = SwerveWheelSettings()
        self.BLSwerve = SwerveWheelSettings()
        self.BRSwerve = SwerveWheelSettings()

class SwerveDrive:
    def __init__(self, settings) -> None:
        self.wheels = [SwerveWheel(settings.FLSwerve), SwerveWheel(settings.FRSwerve), SwerveWheel(settings.BLSwerve), SwerveWheel(settings.BRSwerve)]

    def update(self, heading ,turn, power):
        for wheel in self.wheels:
            wheel.set(heading, turn, power)


def deadband(value):
    return 0 if abs(value) < 0.2 else value

class MyRobot(wpilib.TimedRobot):


    def getLeftStickX(self):
        return deadband(self.stick.getRawAxis(0))

    def getLeftStickY(self):
        return deadband(self.stick.getRawAxis(1))

    def getRightStickX(self):
        return deadband(self.stick.getRawAxis(4))

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        settings = SwerveDriveSettings()
        settings.FLSwerve = SwerveWheelSettings()
        settings.FLSwerve.driveID = 1
        settings.FLSwerve.turnID = 2
        settings.FLSwerve.encoderID = 10
        settings.FLSwerve.encoderOffset = 3500
        settings.FLSwerve.location = "front"
        settings.FLSwerve.P = 1
        settings.FLSwerve.I = 0
        settings.FLSwerve.D = 0.05
        settings.FRSwerve = SwerveWheelSettings()
        settings.FRSwerve.driveID = 7
        settings.FRSwerve.turnID = 8
        settings.FRSwerve.encoderID = 12
        settings.FRSwerve.encoderOffset = 1520
        settings.FRSwerve.location = "front"
        settings.FRSwerve.P = 1.5
        settings.FRSwerve.I = 0
        settings.FRSwerve.D = 0.1
        settings.BLSwerve = SwerveWheelSettings()
        settings.BLSwerve.driveID = 3
        settings.BLSwerve.turnID = 4
        settings.BLSwerve.encoderID = 11
        settings.BLSwerve.encoderOffset = 1742
        settings.BLSwerve.location = "back"
        settings.BLSwerve.P = 0.9
        settings.BLSwerve.I = 0
        settings.BLSwerve.D = 0.05
        settings.BRSwerve = SwerveWheelSettings()
        settings.BRSwerve.driveID = 5
        settings.BRSwerve.turnID = 6
        settings.BRSwerve.encoderID = 9
        settings.BRSwerve.encoderOffset = 3864
        settings.BRSwerve.location = "back"
        settings.BRSwerve.P = 1.2
        settings.BRSwerve.I = 0
        settings.BRSwerve.D = 0.1

        self.stick = wpilib.Joystick(0)
        self.drive = SwerveDrive(settings)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        tx = -self.getLeftStickX()
        ty = -self.getLeftStickY()
        rz = self.getRightStickX()

        # print("tx: {} ty: {} rz: {}".format(tx, ty, rz))

        heading = 0 if tx == 0 and ty == 0 else math.degrees(atan2(tx, ty))
        power = math.hypot(tx, ty)

        # print("heading: {} power: {} turn: {}".format(heading, power, rz))

        self.drive.update(heading, rz, power)

if __name__ == "__main__":
    wpilib.run(MyRobot)
