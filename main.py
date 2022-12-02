import numpy as np
import cv2 as cv
from src.MyRobot import *
from src.calibration import *
from src.camera import *






if __name__ == "__main__":
    Initial_pos = (0,90,-90,-90) # Might want to adjust angles to get better intial view
    x = 200# x height of smarties
    print("-----Initializing Robot-----")
    robot = MyRobot(1, 'COM4', 1000000)
    print("--------Calibrating---------")
    cali = Calibration("/Images/chessboard/*.jpeg", 6, 8)
    cali.load("src/Calibration_result.bin")
    print("----Getting camera feed-----")
    cap = cv.VideoCapture(1) # Might need to adjust, selects the camera

    if not cap.isOpened():
        Exception("Cannot open camera")
    robot.move_j(Initial_pos[0],Initial_pos[1],Initial_pos[2],Initial_pos[3])

    # Might need to delay here

    # Capture frame-by-frame
    real_points = []

    while True:
        ret, frame = cap.read()
        frame = cali.distort_img(frame, Crop = True)

        # if frame is read correctly ret is True
        if not ret:
            Exception("Can't receive frame (stream end?)")
        # Show the image
        cv.imshow('frame', frame)

        # Our operations on the frame come here
        # Circle detection for the skittles
        # Get the array with format [x y r] in pixel coordinates
        circles = get_circles(frame)
        im_red, red_points = get_red_center(frame,circles)
        cv.imshow('red',im_red)

        if red_points is not None or len(red_points) != 0:
            print(f"Points in the image: {red_points}")

            real_points = from_pixel_to_frame(red_points, cali.cameramtx, robot.CamPos[2])
            print(f"Points in the camera frame: {real_points}")



        if cv.waitKey(1) == ord('q'):
            break


    for point in real_points:
        point = np.asarray([x,point[0],point[1]])
        print("current cam point" + str(point))
        point = robot.CamToWorld(point)   # translate point in camera frame to world frame
        print("Goal coordinates" + str(point))
        (j1,j2,j3,j4) = robot.inverse(point,[0,0,-1])
        robot.move_j(j1,j2,j3,j4)

        time.sleep(1)
        robot.move_j(Initial_pos[0],Initial_pos[1],Initial_pos[2],Initial_pos[3])



    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()
