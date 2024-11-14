import cv2 as cv
import numpy as np
import math




update = True

def foo(bar): 
    global update
    update = True

def rescaleFrame(frame, scale = 0.1):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

cv.namedWindow("input",cv.WINDOW_NORMAL)
cv.createTrackbar("HL", "input", 10, 180, foo)
cv.createTrackbar("HH", "input", 30, 180, foo)
cv.createTrackbar("SL", "input", 154, 255, foo)
cv.createTrackbar("SH", "input", 255, 255, foo)
cv.createTrackbar("VL", "input", 145, 255, foo)
cv.createTrackbar("VH", "input", 255, 255, foo)
cv.createTrackbar("CannyHigh","input", 66,300,foo)
cv.createTrackbar("CannyLow","input",42,300,foo)
cv.createTrackbar("rho", "input", 1,5,foo)
cv.createTrackbar("theta", "input", 50,200,foo)
cv.createTrackbar("threshhold", "input", 25,200,foo)
cv.createTrackbar("minLength", "input", 25, 100,foo)
cv.createTrackbar("lineGap", "input", 10,100,foo)
while True:

    lower = (cv.getTrackbarPos("HL", 'input'), cv.getTrackbarPos("SL", 'input'), cv.getTrackbarPos("VL", 'input'))
    upper = (cv.getTrackbarPos("HH", 'input'), cv.getTrackbarPos("SH", 'input'), cv.getTrackbarPos("VH", 'input'))
    cannyLower = cv.getTrackbarPos("CannyLow", "input")
    cannyHigher = cv.getTrackbarPos("CannyHigh", "input")

    img = cv.imread('Photos/singleSample.jpg')

    # img = cv.imread('Photos/doubleSample.jpg')

    # img = cv.imread('Photos/tSample.jpg')

    # blank = cv.

    # img = cv.imread('Photos/stackedSamples.jpg')

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    redLowerBound = (1,1,1)
    redHighBound = (255,255,255)

    blueLowerBound = (1,1,1)
    blueHigherBound = (255,255,255)

    yellowLowerBound = (3,116,112)
    yellowHigherBound = (30,255,255)


# red

#blue

    yellow = cv.inRange(hsv, lower, upper)

    mask = yellow

    kernel = np.ones((55,55),np.uint8)
    eroded = cv.erode(mask, kernel)

    yellow = cv.bitwise_and(img, img,mask=yellow)

    canny = cv.Canny(yellow, cannyLower, cannyHigher)


    # lines = cv.HoughLines(canny, 1, np.pi / 180, 150, None, 0, 0)

    # if lines is not None:
    #     for i in range(0, len(lines)):
    #         rho = lines[i][0][0]
    #         theta = lines[i][0][1]
    #         a = math.cos(theta)
    #         b = math.sin(theta)
    #         x0 = a * rho
    #         y0 = b * rho
    #         pt1 = (int(x0 + img.shape[1]*(-b)), int(y0 + img.shape[0]*(a)))
    #         pt2 = (int(x0 - img.shape[1]*(-b)), int(y0 - img.shape[0]*(a)))
    #         cv.line(canny, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
    
    rho = cv.getTrackbarPos("rho", "input")
    theta = cv.getTrackbarPos("theta", "input")
    threshold = cv.getTrackbarPos("threshhold","input")
    minLength = cv.getTrackbarPos("minLength", "input")
    lineGap = cv.getTrackbarPos("lineGap", "input")

    linesP = cv.HoughLinesP(canny, rho, np.pi / theta, threshold, None, minLength, lineGap)
    
    canny = cv.cvtColor(canny, cv.COLOR_GRAY2BGR)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)



    #cv.findContours(yellow, cv.NETR_TREE, cv.CHAIN_APPROX_SIMPLE)


    mask = rescaleFrame(mask, 0.1)
    yellow = rescaleFrame(yellow, 0.1)
    img = rescaleFrame(img, 0.1)
    canny = rescaleFrame(canny, 0.1)
    


   
    cv.imshow('raw image', img)
    cv.imshow('yellow', yellow)
    cv.imshow('mask', mask)
    cv.imshow('canny', canny)

    

    key = cv.waitKey(1)
    if key == 27:
        break