#!/usr/bin/env python3
import cv2
import numpy as np
#from matplotlib import pyplot as plt


def getContours(image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        print(area)
        if area > 50:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.3 * perimeter, True)
            print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)
            if objCor == 4:
                aspRatio = w / float(h)
                objectType = 'rectangle'
            else:
                objectType = "None"

            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(imgContour, objectType, (x + (w // 2) - 10, y + (h // 2) - 10),
                        cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 0), 2)


img = cv2.imread('chess1.png')
imgContour = img.copy()
imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lowerHSV = np.array([0, 0, 47])
upperHSV = np.array([29, 255, 255])
maskHSV = cv2.inRange(imgHSV, lowerHSV, upperHSV)
imgResultHSV = cv2.bitwise_and(img, img, mask=maskHSV)

mask_blur = cv2.GaussianBlur(maskHSV, (31, 31), 0)
mask_canny = cv2.Canny(mask_blur, 100, 175)
getContours(mask_canny)
cv2.imshow("Stacked Images", mask_canny)
cv2.waitKey(0)
