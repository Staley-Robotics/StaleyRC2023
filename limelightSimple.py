from ntcore import *

table = NetworkTableInstance.getDefault().getTable("limelight-two")


class Limelight:

    def __init__(self):

        table.putNumber('ledMode', 0)
        table.putNumber('camMode', 1)
        table.putNumber('stream', 2)
        table.putNumber('pipeline', 0)

    def look(self):

        tx = table.getNumber('tx', None)
        ty = table.getNumber('ty', None)
        # ta = table.getNumber('ta', None)
        # ts = table.getNumber('ts', None)

        # print(f"targets = {tx}", f"offset = {ty}")
