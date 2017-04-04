from threading import Thread
import cv2
import NerdyFunctions, NerdyConstants

"""USB camera stream class for optimized FPS modified by tedfoodlin"""
__author__ = "pyimagesearch @http://www.pyimagesearch.com/"


class CameraStream:
    def __init__(self, src=-1):
        # initialize the video camera stream settings and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_EXPOSURE, -8.0)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, NerdyConstants.FRAME_X)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, NerdyConstants.FRAME_Y)

        (self.grabbed, self.frame) = self.stream.read()
        self.img = self.frame.copy()

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        thread = Thread(target=self.update, args=())
        # make a daemon thread so it terminates when main program terminates
        thread.daemon = True
        # start thread
        thread.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()
            self.img = self.frame.copy()
            NerdyFunctions.draw_static(self.img)
            cv2.imwrite("/tmp/stream/img.jpg", self.img)
            cv2.imshow("stream", self.img)

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
