import numpy as np
import cv2 as cv
import imutils
import sys, os

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
        Function that finds the key features of image for further manipulation
        Inputs:
            frame: image that is analysed
            nr_corners: number of features to find
            quality: quality of feature that is found, value from [0,1]
            mind_dist: minumum distance between each feature
            winSize: window size for more accurate position of feature
        Outputs:
            out_img: image containing drawn features
            corners: list of positions for each feature
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
        Function that uses optical flow to rectify the two images
        Inputs:
            prev: previous image to consider
            curr: current image, showing current field of view
            feat1: last feature to be found from prev image
        Outputs:
            prev_rectified: previous image rectified
            curr_rectified: current image rectified
            feat2: features from current image
    """
    if(len(prev.shape)==3):
        prev_gray = cv.cvtColor(prev,cv.COLOR_BGR2GRAY)
    if(len(curr.shape)==3):
        curr_gray = cv.cvtColor(curr,cv.COLOR_BGR2GRAY)
    else:
        prev_gray = prev
        curr_gray = curr
    fx = 1430
    fy = 1430
    cx = 240
    cy = 320
    CameraM = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]])

    feat2, status, error = cv.calcOpticalFlowPyrLK(prev_gray, curr_gray, feat1, None)
    F, mask = cv.findFundamentalMat(feat1,feat2,cv.FM_LMEDS)
    feat1 = feat1[mask.ravel() == 1]
    feat2 = feat2[mask.ravel() == 1]
    h, w = curr_gray.shape
    ret, H1, H2 = cv.stereoRectifyUncalibrated(feat1,feat2,F,(w,h))

    #R1 = np.linalg.inv(CameraM)*H1*CameraM
    #R2 = np.linalg.inv(CameraM)*H2*CameraM
    #print(R1)
    #print(R2)
    #prevMapX, prevMapY = cv.initUndistortRectifyMap(CameraM,None,R1,CameraM,(w,h),cv.CV_32FC1)
    #currMapX, currMapY = cv.ixnitUndistortRectifyMap(CameraM,None,R2,CameraM,(w,h),cv.CV_32FC1)

    #prev_rectified = cv.remap(prev,prevMapX,prevMapY,cv.INTER_LINEAR, cv.BORDER_CONSTANT)
    #curr_rectified = cv.remap(curr,currMapX,currMapY,cv.INTER_LINEAR, cv.BORDER_CONSTANT)

    prev_rectified = cv.warpPerspective(prev,H1,(w,h),cv.INTER_LINEAR,cv.BORDER_REPLICATE)
    curr_rectified = cv.warpPerspective(curr,H2,(w,h),cv.INTER_LINEAR,cv.BORDER_REPLICATE)

    return prev_rectified, curr_rectified, feat2

def drawlines(img1,img2,lines,pts1,pts2):
    '''
        img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines
    '''
    out_img1 = img1.copy()
    out_img2 = img2.copy()
    r,c,tmp = img1.shape
    #img1 = cv.cvtColor(img1,cv.COLOR_GRAY2BGR)
    #img2 = cv.cvtColor(img2,cv.COLOR_GRAY2BGR)
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv.line(out_img1, (x0,y0), (x1,y1), color,2)
        img1 = cv.circle(out_img1,tuple(pt1),5,color,-1)
        img2 = cv.circle(out_img2,tuple(pt2),5,color,-1)
    return img1,img2

def check_epilines(prev,curr):
    """
        function that finds and draws the epilines on given images
        Inputs:
            prev: previous image
            curr: current image
        Outputs:
            img5: previous image with drawn epilines
            img3: current image with drawn epilines
    """
    # Create a sift detector
    sift = cv.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(prev, None)
    kp2, des2 = sift.detectAndCompute(curr, None)
    #cv.imshow('2.5',prev)
    #kp_img_prev = cv.drawKeypoints(prev, kp1, prev, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #kp_img_curr = cv.drawKeypoints(curr, kp2, curr, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    bf = cv.BFMatcher()
    matches = bf.match(des1, des2)
    matches = sorted(matches, key = lambda x:x.distance)

    nb_matches = 200
    good = []
    pts1 = []
    pts2 = []
    # Using 200 best matches
    for m in matches[:nb_matches]:
        good.append(m)
        # Extract points corresponding to matches.
        pts1.append(kp1[m.queryIdx].pt)
        pts2.append(kp2[m.trainIdx].pt)

    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)

    F, mask =cv.findFundamentalMat(pts1, pts2, method=cv.FM_RANSAC)

    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]


    # Find epilines corresponding to points in right image (second image) and
    # drawing its lines on left image
    lines1 = cv.computeCorrespondEpilines(pts2.reshape(-1, 1, 2), 2 ,F)
    lines1 = lines1.reshape(-1, 3)
    img5, img6 = drawlines(prev, curr, lines1, pts1, pts2)

    # Find epilines corresponding to points in left image (first image) and
    # drawing its lines on right image
    lines2 = cv.computeCorrespondEpilines(pts1.reshape(-1, 1, 2), 1, F)
    lines2 = lines2.reshape(-1, 3)
    img3, img4 = drawlines(curr, prev, lines2, pts2, pts1)
    return img5, img3



if __name__ == "__main__":
    # prev = cv.imread("Images/prev_frame.jpg")
    # curr = cv.imread("Images/curr_frame.jpg")
    # print(prev.shape)

    # prev_img, feat1 = find_keypoints(prev,200)

    # #cv.imshow('1',prev)
    # new_prev, new_curr = check_epilines(prev,curr)
    # #cv.imshow('2',prev)
    # prev_r, curr_r, feat2 = rectify(prev,curr,feat1)

    # prev_r, curr_r = check_epilines(prev_r,curr_r)


    # cv.imshow('epi1',new_prev)
    # cv.imshow('epi2',new_curr)
    # cv.imshow('r1',prev_r)
    # cv.imshow('r2',curr_r)
    # cv.waitKey(0)
    # cv.destroyAllWindows()


    # Circle detection for the skittles
    im = cv.imread("../Images/Smarties.jpg", cv.IMREAD_COLOR)
    im_gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)

    # Blur using 3x3 kernel
    gray_blurred = cv.blur(im_gray, (3,3))

    det_circles = cv.HoughCircles(gray_blurred, cv.HOUGH_GRADIENT, dp = 1, minDist = 20, param1 = 50,
                                    param2 = 30, minRadius = 1, maxRadius = 40)
    
    print(det_circles)
    # Draw detected circles
    if det_circles is not None:
        det_circles = np.uint16(np.around(det_circles))
        det_circles = det_circles[0]
        for pt in det_circles:
            a, b, r = pt[0], pt[1], pt[2]
            # Draw the circle
            cv.circle(im, (a, b), r, (0,255,0), 2)
            # Draw the center
            cv.circle(im, (a, b), 1, (0,0,255), 3)
        cv.imshow("Detected circles", im)
        cv.waitKey(0)
    else:
        print("No circles were detected.")

"""
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
"""


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
