import ntcore
from subsystems.pigeon2Subsystem import Pigeon2Subsystem
from constants import Constants


class NetworkTables:
    def __init__(self):
        inst = ntcore.NetworkTableInstance.getDefault()
        self.prev = 0
        self.rawGyro = [0.0, 0.0, 0.0]
        self.accumZ = [0.0, 0.0, 0.0]
        self.pigeon = Pigeon2Subsystem(Constants.Pigeon2ID)
        self.yaw = 0

        self.dblTopicYaw = inst.getDoubleTopic("/mapping/gyro/ypr/yaw")
        self.dblTopicRoll = inst.getDoubleTopic("/mapping/gyro/ypr/roll")
        self.dblTopicPitch = inst.getDoubleTopic("/mapping/gyro/ypr/pitch")
        self.dblTopicCompass = inst.getDoubleTopic("/mapping/gyro/compassHeading")
        self.dblArrayTopicAccumZ = inst.getDoubleArrayTopic("/mapping/gyro/accumZ/accumZ")
        self.dblTopicAccumZx = inst.getDoubleTopic("/mapping/gyro/accumZ/x")
        self.dblTopicAccumZy = inst.getDoubleTopic("mapping/gyro/accumZ/y")
        self.dblTopicAccumZz = inst.getDoubleTopic("/mapping/gyro/accumZ/z")
        self.dblArrayTopicRawGyro = inst.getDoubleArrayTopic("/mapping/gyro/rawGyro/rawGyro")
        self.dblTopicRawX = inst.getDoubleTopic("/mapping/gyro/rawGyro/rawX")
        self.dblTopicRawY = inst.getDoubleTopic("/mapping/gyro/rawGyro/rawY")
        self.dblTopicRawZ = inst.getDoubleTopic("/mapping/gyro/rawGyro/rawZ")
        self.intTopicUpTime = inst.getIntegerTopic("/mapping/gyro/upTime")
        self.dblTopicTemp = inst.getDoubleTopic("/mapping/gyro/temp")

        self.yawPub = self.dblTopicYaw.publish()
        self.pitchPub = self.dblTopicPitch.publish()
        self.rollPub = self.dblTopicRoll.publish()
        self.compassPub = self.dblTopicCompass.publish()
        self.accumZPub = self.dblArrayTopicAccumZ.publish()
        self.accumZxPub = self.dblTopicAccumZx.publish()
        self.accumZyPub = self.dblTopicAccumZy.publish()
        self.accumZzPub = self.dblTopicAccumZz.publish()
        self.rawGyroPub = self.dblArrayTopicRawGyro.publish()
        self.rawXPub = self.dblTopicRawX.publish()
        self.rawYPub = self.dblTopicRawY.publish()
        self.rawZPub = self.dblTopicRawZ.publish()
        self.upTimePub = self.intTopicUpTime.publish()
        self.tempPub = self.dblTopicTemp.publish()

        # Begins subscribing to topics
        # parameter is default value if no value is available when get() is called
        self.dblSubYaw = self.dblTopicYaw.subscribe(0.0)
        self.dblSubPitch = self.dblTopicPitch.subscribe(0.0)
        self.dblSubRoll = self.dblTopicRoll.subscribe(0.0)
        self.dblSubCompass = self.dblTopicCompass.subscribe(0.0)
        self.dblSubAccumZ = self.dblArrayTopicAccumZ.subscribe(self.accumZ)
        self.dblSubAccumZx = self.dblTopicAccumZx.subscribe(0.0)
        self.dblSubAccumZy = self.dblTopicAccumZy.subscribe(0.0)
        self.dblSubAccumZz = self.dblTopicAccumZz.subscribe(0.0)
        self.dblSubRawGyro = self.dblArrayTopicRawGyro.subscribe(self.rawGyro)
        self.dblSubRawX = self.dblTopicRawX.subscribe(0.0)
        self.dblSubRawY = self.dblTopicRawY.subscribe(0.0)
        self.dblSubRawZ = self.dblTopicRawZ.subscribe(0.0)
        self.intSubUpTime = self.intTopicUpTime.subscribe(0)
        self.dblSubTemp = self.dblTopicTemp.subscribe(0.0)

    def update(self):
        self.yawPub.set(self.pigeon.getYaw())
        self.pitchPub.set(self.pigeon.getPitch())
        self.rollPub.set(self.pigeon.getRoll())
        self.compassPub.set(self.pigeon.getCompass())
        self.accumZPub.set(self.pigeon.getAccumZ()[1])
        self.accumZxPub.set(self.pigeon.getAccumZ()[1][0])
        self.accumZyPub.set(self.pigeon.getAccumZ()[1][1])
        self.accumZzPub.set(self.pigeon.getAccumZ()[1][2])
        self.rawGyroPub.set(self.pigeon.getRawGyros()[1])
        self.rawXPub.set(self.pigeon.getRawGyros()[1][0])
        self.rawYPub.set(self.pigeon.getRawGyros()[1][1])
        self.rawZPub.set(self.pigeon.getRawGyros()[1][2])
        self.upTimePub.set(self.pigeon.getUpTime())
        self.tempPub.set(self.pigeon.getTemp())
