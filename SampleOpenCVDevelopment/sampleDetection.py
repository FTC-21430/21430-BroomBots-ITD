import cv2 as cv
import numpy as np
import math
import pdb 



update = True

def foo(bar): 
    global update
    update = True

def rescaleFrame(frame, scale = 0.1):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

def rescan(x,y,z):
    z = 12
    y *= -1
    print(x)
    print(y)
    print(z)

cv.namedWindow("input",cv.WINDOW_NORMAL)
cv.createTrackbar("HL", "input", 12, 180, foo)
cv.createTrackbar("HH", "input", 61, 180, foo)
cv.createTrackbar("SL", "input", 119, 255, foo)
cv.createTrackbar("SH", "input", 255, 255, foo)
cv.createTrackbar("VL", "input", 160, 255, foo)
cv.createTrackbar("VH", "input", 255, 255, foo)
cv.createTrackbar("CannyHigh","input", 180,2000,foo)
cv.createTrackbar("CannyLow","input",288,1000,foo)
cv.createTrackbar("rho", "input", 1,5,foo)
cv.createTrackbar("theta", "input", 50,200,foo)
cv.createTrackbar("threshhold", "input", 25,200,foo)
cv.createTrackbar("minLength", "input", 25, 100,foo)
cv.createTrackbar("lineGap", "input", 10,100,foo)


while True:
    if update:

        foundSamples = []

        update = False
        lower = (cv.getTrackbarPos("HL", 'input'), cv.getTrackbarPos("SL", 'input'), cv.getTrackbarPos("VL", 'input'))
        upper = (cv.getTrackbarPos("HH", 'input'), cv.getTrackbarPos("SH", 'input'), cv.getTrackbarPos("VH", 'input'))
        cannyLower = cv.getTrackbarPos("CannyLow", "input")
        cannyHigher = cv.getTrackbarPos("CannyHigh", "input")

        img = cv.imread('ROBOT Photos/testImageOne.jpg')

        # img =cv.imread('Photos/singleSample.jpg')
        # img = rescaleFrame(img, 0.4)

        # img = cv.imread('ROBOT Photos/5.jpg')

        # blank = cv.

        # img = cv.imread('Photos/stackedSamples.jpg')


        img = rescaleFrame(img, 0.2)
        
        img = cv.GaussianBlur(img, (3,3), 5)

       

        # cropping time

        # pdb.set_trace()

        scaleX = img.shape[0]
        scaleY = img.shape[1]

        croppingFactor = 3

        cropScaleX = scaleX // croppingFactor // 2
        cropScaleY = scaleY // croppingFactor // 2 

        cropped = img[cropScaleX:scaleX-cropScaleX, cropScaleY:scaleY-cropScaleY]

        cv.imshow("crop", cropped)

        hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)

        redLowerBound = (1,1,1)
        redHighBound = (255,255,255)

        blueLowerBound = (1,1,1)
        blueHigherBound = (255,255,255)

        yellowLowerBound = lower
        yellowHigherBound = upper


    # red

    #blue

        yellow = cv.inRange(hsv, lower, upper)

        mask = yellow

        kernel = np.ones((55,55),np.uint8)
        eroded = cv.erode(mask, kernel)

        yellow = cv.bitwise_and(cropped, cropped,mask=yellow)

        cv.imshow("yellow before", yellow)

        # new_image = np.zeros(yellow.shape, yellow.dtype)

        
        # alpha = 0.8 # Simple contrast control
        # beta = 1.6    # Simple brightness control 

        # for y in range(yellow.shape[0]):
        #     for x in range(yellow.shape[1]):
        #         for c in range(yellow.shape[2]):
        #             yellow[y,x,c] = np.clip(alpha*yellow[y,x,c] + beta, 0, 255)




        seperationCanny = cv.Canny(yellow, cannyLower, cannyHigher)

        seperationCanny = cv.dilate(seperationCanny, np.ones((5,5)), 2)
    
        seperatedYellow = cv.subtract(mask, seperationCanny)

        Canny = cv.Canny(seperatedYellow, 5,10)
        

        Canny = cv.GaussianBlur(Canny,(3,3), 3)

        cv.imshow("after Canny",Canny)


        contours, hierarchy = cv.findContours(Canny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # largestContour = contours[0]

        # for i in range(len(contours)):
        #      if cv.arcLength(largestContour, True) < cv.arcLength(contours[i], True):
        #          largestContour = contours[i]
            
            # rediculusly bad sample position that any sample is better than this, I need to give it a value soo yeah
        bestSample = (10000,10000)

        for c in contours:
            if cv.arcLength(c,True) < 95:
                continue
            if cv.arcLength(c, True) > 200:
                continue

            cv.drawContours(yellow, [c], -1, (0,0,255), 5)

            approxContour = cv.approxPolyDP(c, cv.arcLength(c, True) / 22, True)
            cv.drawContours(yellow, [approxContour], -1, (0,255,0),3)
            
            # pdb.set_trace()
            # print(f"before {approxContour.size}")

            if approxContour.size > 8:
                finalApproxContour = cv.approxPolyDP(approxContour, cv.arcLength(c, True) / 10, True)
            else:
                finalApproxContour = approxContour

    
            # print(f" after {finalApproxContour.size}")

            if finalApproxContour.size == 8:
                 #found this code at https://learnopencv.com/head-pose-estimation-using-opencv-and-dlib/


                sample_points = np.array([
                                        (-0.75, 1.75, 0.75),     # Left Top point
                                        (-0.75, -1.75, 0.75),    # Left Bottom point
                                        (0.75, -1.75, 0.75),     # Right Bottom point
                                        (0.75, 1.75, 0.75),      # Right Top point
                                        
                                    ])

                # Camera internals
                size = yellow.shape
                focal_length = size[1]
                center = (size[1]/2, size[0]/2)
                camera_matrix = np.array(
                                    [[600.01851744*.2, 0, 906.817157357*.2],
                                    [0, 600.01851744*.2, 516.73047402*.2],
                                    [0, 0, 1]], dtype = "double"
                                    )
                # camera_matrix = np.array(
                #                     [[focal_length, 0, center[0]],
                #                     [0, focal_length, center[1]],
                #                     [0, 0, 1]], dtype = "double"
                #                     )
                finalApproxContour=np.array(finalApproxContour, dtype=np.float32)
                        
                dist_coeffs = np.zeros((5,1)) # Assuming no lens distortion
                dist_coeffs[0] = 0.0115588983608
                dist_coeffs[1] = -0.0313357203804
                dist_coeffs[2] = 0.00013459478315
                dist_coeffs[3] = 0.000897741867319
                dist_coeffs[4] = 0.00542752872672

                (success, rotation_vector, translation_vector) = cv.solvePnP(sample_points, finalApproxContour, camera_matrix, dist_coeffs, flags = cv.SOLVEPNP_ITERATIVE)
                rotMat,_ = cv.Rodrigues(rotation_vector)

                if success:
                    points, _ = cv.projectPoints(sample_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
                    print(f"transform{translation_vector}")
                    print(f"rotaiton{rotation_vector}")
                    print("rot matrix\n", rotMat)
                averagePoint = (0,0)

                for p in points:

                    averagePoint += p

                    cv.circle(yellow,tuple(p.reshape(2).astype(int).tolist()),4,(255,0,0),2)

                averagePoint /= 4

                # if (0,0) - averagePoint < contours[bestSample].



     



    
    cv.imshow('raw image', img)
    cv.imshow('yellow', yellow)
    cv.imshow('mask', seperatedYellow)
    cv.imshow('canny', seperationCanny)

    

    key = cv.waitKey(1)
    if key == 27:
        break




    
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
        
        # rho = cv.getTrackbarPos("rho", "input")
        # theta = cv.getTrackbarPos("theta", "input")
        # threshold = cv.getTrackbarPos("threshhold","input")
        # minLength = cv.getTrackbarPos("minLength", "input")
        # lineGap = cv.getTrackbarPos("lineGap", "input")

        # linesP = cv.HoughLinesP(canny, rho, np.pi / theta, threshold, None, minLength, lineGap)
        
        # # canny = cv.cvtColor(canny, cv.COLOR_GRAY2BGR)
        
        # if linesP is not None:
        #     for i in range(0, len(linesP)):
        #         l = linesP[i][0]
        #         # cv.line(canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)