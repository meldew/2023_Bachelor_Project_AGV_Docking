import cv2 as cv
import numpy as np

width = 640
height = 480

cap = cv.VideoCapture(0)
cap.set(3, width)
cap.set(4, height)

def empty():
    pass

cv.namedWindow("Parameters")
cv.resizeWindow("Parameters",640,240)
cv.createTrackbar("Treshhold1","Parameters",23,255,empty)
cv.createTrackbar("Treshhold2","Parameters",204,255,empty)
cv.createTrackbar("Area","Parameters",5000,30000,empty)

def getContours(img,imgCountor): 
    countors, hierarchy = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for cnt in countors: 
        area = cv.contourArea(cnt)
        minimum_area = cv.getTrackbarPos("Area","Parameters")
        if area > minimum_area: 
            cv.drawContours(imgCountor, cnt , -1, (255, 0, 255), 7)
            peri = cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, 0.02 * peri, True)            
            x, y, w, h = cv.boundingRect(approx)
            center_x = x + w/2
            center_y = y + h/2
            
            cv.rectangle(imgCountor, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv.circle(imgCountor, (int(center_x), int(center_y)), 15, (0, 0, 255), 3) 
            cv.putText(imgCountor, "Points: " + str(len(approx)),(x + w +20, y + h + 20),
                                            cv.FONT_HERSHEY_COMPLEX,0.7,(0, 255, 0), 2)
            cv.putText(imgCountor, "Area: " + str(int(area)),(x + w +20, y + h + 45),
                                            cv.FONT_HERSHEY_COMPLEX,0.7,(0, 255, 0), 2)
while True: 
    success, img = cap.read()
    imgCountor = img.copy()
    imgBlur = cv.GaussianBlur(img, (7, 7),1)
    imgGray = cv.cvtColor(imgBlur, cv.COLOR_BGR2GRAY)

    threshold1 = cv.getTrackbarPos("Treshhold1","Parameters")
    threshold2 = cv.getTrackbarPos("Treshhold2","Parameters")
    kernel = np.ones((5,5))
    imgCanny = cv.Canny(imgGray,threshold1,threshold2)
    imgDil = cv.dilate(imgCanny, kernel, iterations=1)

    getContours(imgDil,imgCountor)

    cv.imshow("Frame",imgCountor)
    if cv  .waitKey(1) == ord('q'):
        break    