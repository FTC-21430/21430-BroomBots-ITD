import cv2 as cv

def rescaleFrame(frame, scale = 0.6):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

img = cv.imread('Photos/electro_fancy.jpg')

img = rescaleFrame(img)



gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

blur = cv.GaussianBlur(img, (1,1), cv.BORDER_DEFAULT)

canny = cv.Canny(blur, 300, 300)

dilated = cv.dilate(canny, (5,5), iterations=1)

eroded = cv.erode(dilated, (5,5), iterations=1)

resized = cv.resize(img, (500,500), interpolation=cv.INTER_CUBIC)
#cv.imshow('canny', canny)
cv.imshow('dilated', eroded)
cv.imshow('Electro', img)


cropped = img[50:200, 200:400]
#cv.imshow('crop', cropped)

cv.waitKey(0)