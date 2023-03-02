from enum import Enum
from typing import Tuple

from wpilib import *


class Mode(Enum):
    DISABLED = 0
    TELEOP = 1
    AUTO = 2
    TEST = 3


class PipelineManager:
    controllers: Tuple[XboxController, ...]
    throttle: any
    rotation: any
    direction_x: any
    direction_y: any
    point_1: any
    point_2: any
    point_3: any
    point_4: any
    shaft_axis: any
    stepper_up: any
    stepper_down: any
    grip: any

    def __init__(self, *controller: XboxController):
        self.controllers = controller
        self.mode = Mode.DISABLED

    def set_mode(self, mode: Mode):
        # self.throttle = None
        # self.rotation = None
        # self.direction_x = None
        # self.direction_y = None
        # self.point_1 = None
        # self.point_2 = None
        # self.point_3 = None
        # self.point_4 = None
        # self.shaft_axis = None
        # self.grip = None

        self.throttle = self.controllers[0].getLeftY
        self.rotation = self.controllers[0].getLeftX
        self.direction_x = self.controllers[0].getRightX
        self.direction_y = self.controllers[0].getRightY
        self.point_1 = self.controllers[0].getAButtonPressed
        self.point_2 = self.controllers[0].getBButtonPressed
        self.point_3 = self.controllers[0].getXButtonPressed
        self.point_4 = self.controllers[0].getYButtonPressed
        self.shaft_axis = self.controllers[0].getLeftTriggerAxis
        self.grip = lambda: self.controllers[0].getRightTriggerAxis() > 0.7


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
k_gains: Gains = Gains(0.1, 0.0, 1.0, 0.0, 0, 1.0)
k_sensor_phase = True
k_motor_invert = False
