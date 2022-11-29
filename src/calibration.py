import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import pickle


class Calibration:
    def __init__(self, img_path, nb_vertical, nb_horizontal):
        # setup for the calibration images
        self.PATH = img_path
        self.pattern_size = (nb_vertical, nb_horizontal)

        self.img_shape = None
        # for distorsion
        self.objpoints = []

        self.mtx = None
        self.dist = None
        self.roi = None
        self.cameramtx = None

    def calibrate(self, Show = False):

        #calibrate left camera
        ind = self.PATH.find("*")
        path = self.PATH
        print(path)
        self.mtx, self.dist, self.roi, self.cameramtx, self.imgpoints = self._calibrate(path,Show)

    def _calibrate(self, Path, Show):

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.pattern_size[0]*self.pattern_size[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:self.pattern_size[0],0:self.pattern_size[1]].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # Load the images
        images = glob.glob(Path)
        assert images

        # perform calibration for left images only
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            #Implement findChessboardCorners here
            ret, corners = cv2.findChessboardCorners(img, self.pattern_size)

            # If found, add object points, image points (after refining them)
            if ret == True:
                self.objpoints.append(objp)
                imgpoints.append(corners)

            if Show == True:
                img = cv2.drawChessboardCorners(img, (self.pattern_size[0],self.pattern_size[1]), corners,ret)
                cv2.imshow('img',img)
                cv2.waitKey(0)


        h, w = img.shape[:2]
        self.img_shape = gray.shape[::-1]
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, imgpoints, self.img_shape, None, None)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h))

        return mtx, dist, roi, newcameramtx, imgpoints

    def distort_img(self, img, Crop = False):

        #get undistortet image
        dst = cv2.undistort(img, self.mtx, self.dist, None, self.cameramtx)

        #crop if needed
        if Crop == True:
            x,y,w,h = self.roi
            dst = dst[y:y+h, x:x+w]
        return dst


    def save(self):
        dict_out = {'mtx': self.mtx, 'dist': self.dist, 'cammtx': self.cameramtx, 'roi': self.roi}

        f = open("Calibration_result.bin","wb")
        pickle.dump(dict_out, f)
        f.close


    def load(self,bin_file):
        with open(bin_file,'rb') as f:
            dict_in = pickle.load(f)
        self.mtx = dict_in['mtx']
        self.dist = dict_in['dist']
        self.cameramtx = dict_in['cammtx']
        self.roi = dict_in['roi']





if __name__ == "__main__":
    # test code

    cali = Calibration("../Images/chessboard/*.jpeg", 6, 8)
    img_left = cv2.imread("../Images/curr_frame.jpg")
    img_right = cv2.imread("../Images/prev_frame.jpg")




    print("-------------- Calibrating --------------")
    cali.calibrate(True)
    dst = cali.distort_img(img_left, Crop = True)
    cv2.imshow('1',dst)


    print("--------------- Saving -------------------")
    cali.save()

    print("--------------- Loading ------------------")
    cali.load("Calibration_result.bin")
    print(cali.cameramtx)



    cv2.waitKey(0)
