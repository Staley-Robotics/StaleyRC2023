import cv2
import numpy as np


class TestCam:

    def __init__(self):
        ret: bool
        image: np.array
        params: cv2.SimpleBlobDetector_Params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 1
        params.maxThreshold = 1000
        params.minArea = 50
        detector: cv2.SimpleBlobDetector = cv2.SimpleBlobDetector_create(params)
        camera = cv2.VideoCapture(0)
        while True:
            ret, image = camera.read()
            keypoints = detector.detect(image)
            im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255),
                                                  cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.imshow("Camera Stream", im_with_keypoints)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        camera.release()
        cv2.destroyAllWindows()


webcam = TestCam()
