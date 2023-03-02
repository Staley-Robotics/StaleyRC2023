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