from cscore import CameraServer

CameraServer.enableLogging()

camera = CameraServer.startAutomaticCapture()
camera.setResolution(320, 240)

sink = CameraServer.getVideo()

while True:
    time, input_img = sink.grabFrame(input_img)

    if time == 0:  # There is an error
        continue
