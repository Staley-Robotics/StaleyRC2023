from enum import Enum

from wpilib import *


class Mode(Enum):
    DISABLED = 0
    TELEOP = 1
    AUTO = 2
    TEST = 3


class PipelineManager:
    controllers: tuple[XboxController, ...]
    drive: any
    rotation: any
    direction_x: any
    direction_y: any
    point_1: any
    point_2: any
    point_3: any
    point_4: any
    shaft_axis: any
    pivot_axis: any
    pivot_negative_axis: any
    grip: any
    release: any

    def __init__(self, *controller: XboxController):
        self.controllers = controller
        self.mode = Mode.DISABLED

    def set_mode(self, mode: Mode):
        self.mode = mode
        if self.mode == Mode.AUTO:
            self.drive = None
            self.rotation = None
            self.direction_x = None
            self.direction_y = None
            self.point_1 = None
            self.point_2 = None
            self.point_3 = None
            self.point_4 = None
            self.shaft_axis = None
            self.grip = None
        elif self.mode == Mode.TELEOP:
            self.drive = self.controllers[0].getLeftY
            self.rotation = self.controllers[0].getLeftX
            self.direction_x = self.controllers[0].getRightX
            self.direction_y = self.controllers[0].getRightY
            self.point_1 = self.controllers[1].getAButtonPressed
            self.point_2 = self.controllers[1].getBButtonPressed
            self.point_3 = self.controllers[1].getXButtonPressed
            self.point_4 = self.controllers[1].getYButtonPressed
            self.pivot_axis = self.controllers[1].getRightTriggerAxis
            self.pivot_negative_axis = self.controllers[1].getLeftTriggerAxis
            self.shaft_axis = self.controllers[1].getRightY
            self.grip = self.controllers[1].getRightBumperPressed
            self.release = self.controllers[1].getLeftBumperPressed

    def drive_constant(self, val):
        self.drive = val

    def rotation_constant(self, val):
        self.rotation = val

    def direction_x_constant(self, val):
        self.direction_x = val

    def direction_y_constant(self, val):
        self.direction_y = val

class Gains:
    def __init__(self, _kP, _kI, _kD, _kF, _kIzone, _kPeakOutput):
        self.kP = _kP
        self.kI = _kI
        self.kD = _kD
        self.kF = _kF
        self.kIzone = _kIzone
        self.kPeakOutput = _kPeakOutput


k_timeout: int = 20
PID_loop_idx: int = 0
k_slot_idx: int = 0
k_gains: Gains = Gains(0.05, 0.0, 1.0, 0.0, 0, 1.0)
k_sensor_phase = True
k_motor_invert = False
