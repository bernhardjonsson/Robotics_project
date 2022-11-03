import numpy as np
import cv2 as cv
from MyRobot import *





if __name__ == "__main__":
    #robot = MyRobot(0.5, 'COM4', 1000000)
    cap = cv.VideoCapture(1) # Might need to adjust, selects the camera
    print("Getting camera feed")
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    #robot.move_j(150,110,110,135)
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        gray = frame
        #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Display the resulting frame
        cv.imshow('frame', gray)
        if cv.waitKey(1) == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()
