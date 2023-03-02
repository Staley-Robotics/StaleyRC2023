from ntcore import *


class Limelight:

    def __init__(self):
        self.state = NetworkTableInstance.getDefault()
        self.config = self.state.getTable("limelight")
        self.led_state = self.config.getEntry('ledMode')
        self.cam_mode = self.config.getEntry('camMode')
        self.pipeline_index = self.config.getEntry('pipeline')

    def val(self, key):
        return self.config.getEntry(key).getDouble(0.0)
