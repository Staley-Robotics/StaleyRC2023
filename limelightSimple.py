from ntcore import *

table = NetworkTableInstance.getDefault().getTable("limelight-two")


class Limelight:

    def __init__(self):

        table.setDefaultNumber('ledMode', 0)
        table.putNumber('camMode', 0)
        table.putNumber('stream', 0)
        table.putNumber('pipeline', 0)

    def look(self):

        tx = table.getNumber('tx', None)
        ty = table.getNumber('ty', None)
        # ta = table.getNumber('ta', None)
        # ts = table.getNumber('ts', None)

        # print(f"targets = {tx}", f"offset = {ty}")

        tid = table.getNumber('tid', None)

        if tid != -1:
            print(f"the id = {tid}")
