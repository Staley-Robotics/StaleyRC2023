from cscore import CameraServer
import numpy as np


def main():
    cs = CameraServer.getInstance()
    cs.enableLogging()

    # Capture from the first USB Camera on the system
    camera = cs.startAutomaticCapture()
    camera.setResolution(320, 240)

    # Get a CvSink. This will capture images from the camera
    cv_sink = cs.getVideo()

    # (optional) Set up a CvSource. This will send images back to the Dashboard
    output_stream = cs.putVideo("Name", 320, 240)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cv_sink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            output_stream.notifyError(cv_sink.getError())
            # skip the rest of the current iteration
            continue

        #
        # Insert your image processing logic here!
        #

        # (optional) send some image back to the dashboard
        output_stream.putFrame(img)
