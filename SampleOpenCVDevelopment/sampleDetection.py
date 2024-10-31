import cv2 as cv

def rescaleFrame(frame, scale = 0.1):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

img = cv.imread('Photos/submersible.jpg')





gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

blur = cv.GaussianBlur(img, (1,1), cv.BORDER_REFLECT)

canny = cv.Canny(blur, 130, 130)

dilated = cv.dilate(canny, (5,5), iterations=1)

resized = cv.resize(img, (500,500), interpolation=cv.INTER_CUBIC)
#cv.imshow('canny', canny)
canny = rescaleFrame(canny, 0.1)
img = rescaleFrame(img, 0.1)
cv.imshow('final', canny)
cv.imshow('raw image', img)



cv.waitKey(0)