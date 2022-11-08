import numpy as np
import cv2 as cv
import imutils

# color values and thresholds for different Smarties (color value,threshold)
#   Green: (55,25)
#   Blues: (100,20)
#   red:   (179,5)
#   purple:(150,15)

def mask_smarties(frame,color_val,thres):
    """
        A masking function that uses color mask on object from image using
        hsv color space.
        Inputs:
            frame: image that gets masked
            color_val: hue value of color (hsv color space)
            thres: threshold of the color value the mask uses

    """
    # using hsv colorspace for maksing
    hsv_image = cv.cvtColor(frame, cv.COLOR_BGR2HSV)   # convert image to hsv color
    if (color_val-thres) > 0 or (color_val+thres) < 180:
        minhsv = np.array([color_val-thres, 100, 20])   # calc min and max values for mask
        maxhsv = np.array([color_val+thres, 255, 255])
        color_mask = cv.inRange(hsv_image,minhsv,maxhsv)
    else:
        minhsv = np.array([0, 100, 20])   # calc min and max values for mask
        maxhsv = np.array([(color_val+thres)%180, 255, 255])
        minhsv2 = np.array([(color_val-thres)%180, 100, 20])   # calc min and max values for mask
        maxhsv2 = np.array([179, 255, 255])
        cv.imshow('lower',cv.inRange(hsv_image,minhsv,maxhsv))
        cv.imshow('upper',cv.inRange(hsv_image,minhsv2,maxhsv2))
        color_mask = cv.inRange(hsv_image,minhsv,maxhsv) + cv.inRange(hsv_image,minhsv2,maxhsv2)

    color_mask = cv.erode(color_mask, None, iterations = 2)
    color_mask = cv.dilate(color_mask, None, iterations = 5)
    output_hsv = cv.bitwise_and(hsv_image, hsv_image, mask=color_mask)
    return cv.cvtColor(output_hsv, cv.COLOR_HSV2BGR), color_mask #convert output to RGB

def find_smarties(frame):
    """
        function that finds the smarties in image
        Inputs:
            frame: image that gets analysed
        Outputs:
            out_img: image that shows points of intrest
            points: list of tuples that hold the coordinate of the points
    """
    out_img = frame.copy()
    points = []
    cnts = cv.findContours(frame.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    for c in cnts:
        M = cv.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        points.append((cX,cY))

    return out_img, points

def find_keypoints(frame, nr_corners, quality=0.01, min_dist = 1,winSize = (5,5)):
    """
        Function that finds the key features of image
    """
    out_img = frame.copy()
    if(len(frame.shape)==3):
        gray1 = cv.cvtColor(prev,cv.COLOR_BGR2GRAY)

    feat1 = cv.goodFeaturesToTrack(gray1,nr_corners,quality,min_dist)
    for i in range(feat1.shape[0]):
        cv.circle(out_img, (int(feat1[i,0,0]), int(feat1[i,0,1])), 4,(255, 0, 0), cv.FILLED)

    # Set the needed parameters to find the refined corners, more accurate
    # position
    zeroZone = (-1, -1)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TermCriteria_COUNT, 40, 0.001)
    # Calculate the refined corner locations
    corners = cv.cornerSubPix(gray1, feat1, winSize, zeroZone, criteria)

    return out_img, corners

def rectify(prev,curr,feat1):
    """
        Function that
    """
    if(len(prev.shape)==3):
        prev = cv.cvtColor(prev,cv.COLOR_BGR2GRAY)
    if(len(curr.shape)==3):
        curr = cv.cvtColor(curr,cv.COLOR_BGR2GRAY)

    feat2, status, error = cv.calcOpticalFlowPyrLK(prev, curr, feat1, None)
    F, mask = cv.findFundamentalMat(feat1,feat2,cv.FM_LMEDS)
    h, w = curr.shape
    ret, H1, H2 = cv.stereoRectifyUncalibrated(feat1,feat2,F,(w,h))
    print(F)
    print(H1)
    print(H2)
    new_curr = []
    new_prev = []
    return new_prev, new_curr, feat2




if __name__ == "__main__":
    prev = cv.imread("Images/prev_frame.jpg")
    curr = cv.imread("Images/curr_frame.jpg")

    prev_img, feat1 = find_keypoints(prev,25)
    new_prev, new_curr, feat2 = rectify(prev, curr, feat1)



    for i in range(len(feat1)):
        f10=int(feat1[i][0][0])
        f11=int(feat1[i][0][1])
        f20=int(feat2[i][0][0])
        f21=int(feat2[i][0][1])
        cv.line(curr, (f10,f11), (f20, f21), (0, 255, 0), 2)
        cv.circle(curr, (f10, f11), 5, (0, 255, 0), -1)

    cv.imshow('curr',curr)
    cv.imshow('prev',prev_img)
    print(len(feat2))

    cv.waitKey(0)
    cv.destroyAllWindows()

""" Smarties detection

    smarties = cv.imread("Images/Smarties.jpg")
    masked_img, mask = mask_smarties(smarties,179,5)
    cnt,points = find_smarties(mask)

    cv.imshow('prev',smarties)
    cv.imshow('mask',masked_img)

    for p in points:
        cv.circle(smarties, p, 7, (0, 0, 0), 2)
    cv.imshow('points',smarties)
    print(points)

    cv.waitKey(0)
    cv.destroyAllWindows()
"""
