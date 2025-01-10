import cv2 as cv
import numpy as np
import math
import pdb 

update = True

PIXELS2INCHES = 3/40

def foo(bar): 
    global update
    update = True

def rescaleFrame(frame, scale = 0.1):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

def avgInt(numbers) -> int:
    count = 0
    i = 0
    for n in numbers:
        count += n
        i += 1
    if i != 0:
        count /= i
        avg = int(count)
        return avg
    else:
        return -1


cv.namedWindow("input",cv.WINDOW_NORMAL)
cv.createTrackbar("HL", "input", 12, 180, foo)
cv.createTrackbar("HH", "input", 61, 180, foo)
cv.createTrackbar("SL", "input", 119, 255, foo)
cv.createTrackbar("SH", "input", 255, 255, foo)
cv.createTrackbar("VL", "input", 186, 255, foo)
cv.createTrackbar("VH", "input", 255, 255, foo)
cv.createTrackbar("CannyHigh","input", 27,2000,foo)
cv.createTrackbar("CannyLow","input",272,1000,foo)
cv.createTrackbar("LengthMin","input", 290, 500, foo)
cv.createTrackbar("LengthMax","input", 675, 1500, foo)
cv.createTrackbar("minDistance", "input", 65,100,foo)

while True:
    if update:

        foundSamplePosiitons = []

        update = False
        lower = (cv.getTrackbarPos("HL", 'input'), cv.getTrackbarPos("SL", 'input'), cv.getTrackbarPos("VL", 'input'))
        upper = (cv.getTrackbarPos("HH", 'input'), cv.getTrackbarPos("SH", 'input'), cv.getTrackbarPos("VH", 'input'))
        cannyLower = cv.getTrackbarPos("CannyLow", "input")
        cannyHigher = cv.getTrackbarPos("CannyHigh", "input")
        lengthMin = cv.getTrackbarPos("LengthMin", 'input')
        lengthMax = cv.getTrackbarPos("LengthMax", 'input')
        minDistance = cv.getTrackbarPos("minDistance",'input')

        # img = cv.imread('ROBOT Photos/testImageOne.jpg')
# 
        # img =cv.imread('Photos/singleSample.jpg')
        
        # img = cv.imread('Photos/cornerTouch.jpg')
        
        # img = rescaleFrame(img, 0.4)

        img = cv.imread('ROBOT Photos/8.jpg')

        # blank = cv.

        # img = cv.imread('Photos/stackedSamples.jpg')

                
        # Refining the camera matrix using parameters obtained by calibration
        
        camera_matrix = np.array(
                                    [[600.01851744, 0, 906.817157357],
                                    [0, 600.01851744, 516.73047402],
                                    [0, 0, 1]], dtype = "double"
                                    )

        dist_coeffs = np.zeros((5,1)) # Assuming no lens distortion
        dist_coeffs[0] = 0.0115588983608
        dist_coeffs[1] = -0.0313357203804
        dist_coeffs[2] = 0.00013459478315
        dist_coeffs[3] = 0.000897741867319
        dist_coeffs[4] = 0.00542752872672
        
        
        dist_coeffs = np.array([0.0115588983608,-0.0313357203804,0.00013459478315,0.000897741867319,0.00542752872672])

        # Method 1 to undistort the image
        dst = cv.undistort(img, camera_matrix, dist_coeffs)

        cv.imshow("raw", img)
        cv.imshow("undisorted", dst)
        
        img = dst

        # img = rescaleFrame(img, 0.2)
        
        img = cv.GaussianBlur(img, (3,3), 5)

       

        # cropping time

        # pdb.set_trace()

        scaleX = img.shape[0]
        scaleY = img.shape[1]

        croppingFactor = 2

        cropScaleX = scaleX // croppingFactor //2
        cropScaleY = scaleY // croppingFactor //2 
        cropScaleX += 5
        cropScaleY += 40

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

        seperationCanny = cv.dilate(seperationCanny, np.ones((4,4)), 2)
        
        cv.imshow("seperationCanny", seperationCanny)
    
        seperatedYellow = cv.subtract(mask, seperationCanny)

        Canny = cv.Canny(seperatedYellow, 5,10)
        

        Canny = cv.GaussianBlur(Canny,(3,3), 3)

        cv.imshow("after Canny",Canny)


        contours, hierarchy = cv.findContours(Canny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        for c in contours:
            if cv.arcLength(c,True) < lengthMin:
                continue
            if cv.arcLength(c, True) > lengthMax:
                continue

            # cv.drawContours(yellow, [c], -1, (0,0,255), 5)

            approxContour = cv.approxPolyDP(c, cv.arcLength(c, True) / 22, True)
            
            
            # pdb.set_trace()
          

            if approxContour.size > 8:
                finalApproxContour = cv.approxPolyDP(approxContour, cv.arcLength(c, True) / 10, True)
            else:
                finalApproxContour = approxContour
            
            positionsX = []
            positionsY = []
            for pos in approxContour:
                pos = pos[0]
                positionsX.append(pos[0])
                positionsY.append(pos[1])
            avgX = avgInt(positionsX)
            avgY = avgInt(positionsY)
            
            
            
            centorPos = (avgX,avgY)
            
            
            
            originalPos = True
            
            for s in foundSamplePosiitons:
                if math.dist(centorPos, s) < minDistance:
                    originalPos = False
            
            if originalPos:
                cv.drawContours(yellow, [approxContour], -1, (0,255,0),3)
                foundSamplePosiitons.append(centorPos)
                cv.circle(yellow, centorPos ,10,(255,0,0), 3)
                
                bestDistance = 0
                bestI = 0
                i = 0
                lastPos = ()
                
                pos1 = ()
                pos2 = ()
                
                for pos in approxContour:
                    
                    pos = pos[0]
                    
                    if i == 0: 
                        lastPos = pos
                        i += 1
                        continue
                    
                    dist = math.dist(pos, lastPos)
                    
                    if dist > bestDistance:
                        bestDistance = dist
                        bestI = i
                        pos1 = pos
                        pos2 = lastPos
                        
                    
                    lastPos = pos
                    i += 1
                
                
                
                anglepos1 = ()
                anglepos2 = ()
                
            
                
                if pos1[0] < pos2[0]:
                    anglepos1 = pos1
                    anglepos2 = pos2
                else:
                    anglepos1 = pos2
                    anglepos2 = pos1
                
                
                
                
                difX = anglepos2[0] - anglepos1[0]
                difY = anglepos2[1] - anglepos1[1]
                
                sampleAngle = math.atan2(difX,difY) * (180/math.pi)
                sampleAngle -= 180
                
                distance = math.sqrt(math.pow(difX,2)+math.pow(difY,2))
                
                # print(anglepos1)
                # print(anglepos2)
                print(distance)
                
                
                image_center_x = yellow.shape[0] // 2
                image_center_y = yellow.shape[1] // 2
                
                
                
                inch_position_x = (centorPos[0] - image_center_x) * PIXELS2INCHES
                inch_position_y = (centorPos[1] - image_center_y) * PIXELS2INCHES
                
                print(inch_position_x, inch_position_y)
                
                print("------------")
              
                    
                
            
            
        yellow = rescaleFrame(yellow, 3)
        cv.imshow("final yellow", yellow)
        
        
    key = cv.waitKey(1)
    if key == 27:
        break
