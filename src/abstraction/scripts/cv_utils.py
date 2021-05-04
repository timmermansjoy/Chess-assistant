import numpy as np
import cv2

def magic(original):
    morph_kernel = np.ones((10,10),np.uint)
    gray = cv2.cvtColor(original,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3,3),0)
    #clean = cv2.morphologyEx(blur, cv2.MORPH_OPEN, morph_kernel,iterations = 1)    
    edges = cv2.Canny(blur,150,200,apertureSize = 3)
    lines = cv2.HoughLinesP(edges,2,np.pi/180,125,minLineLength=20,maxLineGap=250)
    line_img = original.copy()
    for line in lines:
        x1,y1,x2,y2 = line[0]
        cv2.line(line_img,(x1,y1),(x2,y2),(0,255,0),2)

    img = crop(line_img)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    upper_val = np.array([255, 255, 255])
    lower_val = np.array([60, 150, 150])    
    mask = cv2.inRange(hsv, lower_val, upper_val)

    canny = canny_edge_detection(mask)
    dilate = cv2.dilate(canny, (3,3), 9)
    orig = crop(original)
    cont = getContours(dilate, orig)
    return cont


def crop(img):
    width = 1100
    height = 1100

    points1 = np.float32([[230, 75], [685, 75], [230, 520], [685, 530]])
    points2 = np.float32([[0,0], [width, 0], [0, height], [width, height]])
    matrix = cv2.getPerspectiveTransform(points1, points2)
    imgOutput = cv2.warpPerspective(img, matrix, (width, height))
    res = cv2.resize(imgOutput, (900, 900))
    return res

def getContours(image, imgContour):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    count = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        #print(area)
        if area > 1000:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.15 * perimeter, True)
            #print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)
            print('x: {} y: {} w: {} h: {}'.format(x, y, w, h))

            if objCor == 4:
                aspRatio = w / float(h)
                if 0.7 < aspRatio < 1.3:
                    objectType = "Square"
            else:
                objectType = "None"
            if objectType != "None":
                cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(imgContour, objectType, (x + (w // 2), y + (h // 2)),
                            cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 0), 2)
                count +=1
    return imgContour

def canny_edge_detection(frame):
    blur = cv2.GaussianBlur(frame, (13,13), 0)
    canny = cv2.Canny(blur, 50, 100)
    return canny

def otsus_thresholding(img):
    blur = cv2.GaussianBlur(img,(7,7),0)
    ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    return th3