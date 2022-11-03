import numpy as np
import cv2 as cv
import imutils

# color values and thresholds for different Smarties (color value,threshold)
#   Green: (55,25)
#   Blues: (100,20)
#   red:   (179,5)
#   purple:(150,15)

def mask_red(frame,color_val,thres):
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

def rectify():
    pass




if __name__ == "__main__":

    prev = cv.imread("Images/prev_frame.jpg")
    curr = cv.imread("Images/curr_frame.jpg")
    smarties = cv.imread("Images/Smarties.jpg")
    masked_img, mask = mask_red(smarties,179,5)
    cnt,points = find_smarties(mask)

    cv.imshow('prev',smarties)
    cv.imshow('mask',masked_img)

    for p in points:
        cv.circle(smarties, p, 7, (0, 0, 0), 2)
    cv.imshow('points',smarties)
    print(points)

    cv.waitKey(0)
    cv.destroyAllWindows()
