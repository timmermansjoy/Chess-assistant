import numpy as np
import cv2

def calculate_corners(original):
    chessboardSize = (7,7)
    frameSize = (1069, 599)

    # Termination Criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1,2)

    img = original.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

    # If found, refine corners
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        corners2 = corners2.ravel()
        corners2 = np.reshape(corners2,(7,7,2))
        return corners2
    return None
    #     return corners2
    # else:
    #     return None


def get_distance(point, squares = 1):
    point - (corners[0][1][0] - point) * squares


def get_squares(corners, img):
    """
    We draw the squares of the board onto the image by taking the inside corners
    and calculating the distance of those. after that we connect the rest of our corners


    | xtl |   |   |   | xtr |
    |     |   |   |   |     |
    |     |   |   |   |     |
    | xbr |   |   |   | xbr |
    """

    xtl = corners[0][0][0] - (corners[0][1][0] - corners[0][0][0])
    ytl = corners[0][0][1] - (corners[1][0][1] - corners[0][0][1])

    xtr = corners[0][6][0] + (corners[0][6][0] - corners[0][5][0])
    ytr = corners[0][6][1] - (corners[1][6][1] - corners[0][6][1])

    xbl = corners[6][0][0] - (corners[6][1][0] - corners[6][0][0])
    ybl = corners[6][0][1] + (corners[6][0][1] - corners[5][0][1])

    xbr = corners[6][6][0] + (corners[6][6][0] - corners[6][5][0])
    ybr = corners[6][6][1] + (corners[6][6][1] - corners[5][6][1])
    cv2.line(img, (xtl, ytl), (xtr,ytr), (0,0,255), 2)
    cv2.line(img, (xtl, ytl), (xbl,ybl), (0,0,255), 2)
    cv2.line(img, (xbl, ybl), (xbr,ybr), (0,0,255), 2)
    cv2.line(img, (xbr, ybr), (xtr,ytr), (0,0,255), 2)
    for i in range(len(corners)):
        begin_vertical = None
        begin_horizontal = None
        # Left outer line
        xl = corners[i][0][0] - (corners[i][1][0] - corners[i][0][0])
        yl = (corners[i][0][1])
        cv2.line(img, (xl, yl), (corners[i][0][0], corners[i][0][1]), (0,0,255), 2)

        # Right outer line
        xr = corners[i][6][0] + (corners[i][6][0] - corners[i][5][0])
        yr =(corners[i][6][1])
        cv2.line(img, (corners[i][6][0], corners[i][6][1]), (xr,yr), (0,0,255), 2)

        # top outer line
        xt = corners[0][i][0]
        yt = corners[0][i][1] - (corners[1][i][1] - corners[0][i][1])
        cv2.line(img, (corners[0][i][0], corners[0][i][1]), (xt,yt), (0,0,255), 2)

        # bottom outer line
        xb = corners[6][i][0]
        yb = corners[6][i][1] + (corners[6][i][1] - corners[5][i][1])
        cv2.line(img, (corners[6][i][0], corners[6][i][1]), (xb,yb), (0,0,255), 2)

        # B ttomouter line

        for j in range(len(corners[i])):
            if begin_vertical is not None:
                cv2.line(img, (begin_vertical[0],begin_vertical[1]), (corners[i][j][0],corners[i][j][1]), (0,0,255), 2)
                cv2.line(img, (begin_horizontal[0], begin_horizontal[1]) ,(corners[j][i][0],corners[j][i][1]), (0,0,255), 2)
            begin_vertical = corners[i][j]
            begin_horizontal = corners[j][i]
    return img


# def magic(original):
    # morph_kernel = np.ones((10,10),np.uint)
    # gray = cv2.cvtColor(original,cv2.COLOR_BGR2GRAY)
    # blur = cv2.GaussianBlur(gray, (3,3),0)
    # #clean = cv2.morphologyEx(blur, cv2.MORPH_OPEN, morph_kernel,iterations = 1)
    # edges = cv2.Canny(blur,150,200,apertureSize = 3)
    # lines = cv2.HoughLinesP(edges,2,np.pi/180,125,minLineLength=20,maxLineGap=250)
    # line_img = original.copy()
    # for line in lines:
    #     x1,y1,x2,y2 = line[0]
    #     cv2.line(line_img,(x1,y1),(x2,y2),(0,255,0),2)

    # img = crop(line_img)

    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # upper_val = np.array([255, 255, 255])
    # lower_val = np.array([60, 150, 150])
    # mask = cv2.inRange(hsv, lower_val, upper_val)

    # orig = crop(original)

    # blurred_mask = cv2.GaussianBlur(mask, (19,19), 0)
    # #corners = cv2.goodFeaturesToTrack(blurred_mask, 100, 0.3, 60)
    # x_vals = [6.0, 116.0, 227.0, 338.0, 449.0, 559.0, 669.0, 778.0, 888.0]
    # y_vals = [6.0, 116.0, 228.0, 339.0, 450.0, 561.0, 672.0, 783.0, 894.0]
    # corners = []
    # for xval in x_vals:
    #     for yval in y_vals:
    #         cv2.circle(orig, (int(xval),int(yval)), 10, (0,0,255), -1)

    # # for corner in corners:
    # #     x,y = corner.ravel()
    # #     cv2.circle(orig, (int(x),int(y)), 10, (0,0,255), -1)
    #     #cv2.putText(orig, '{},{}'.format(x,y), (int(x) + 5, int(y) + 5), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)

    # return orig
    # # canny = canny_edge_detection(mask)
    # # dilate = cv2.dilate(canny, (3,3), 9)
    # # orig = crop(original)
    # # cont = getContours(dilate, orig)
    # # return cont


def detect_and_draw_circles(img):
    current_img = cv2.medianBlur(img,5)
    current_img = cv2.cvtColor(current_img, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(current_img,cv2.HOUGH_GRADIENT,1,20, param1= 75,param2=15,minRadius=8,maxRadius=15)
    if circles is not None:
        circles = np.round(circles [0,:]).astype("int")
        has_found_pieces = True
        for (x,y,radius) in circles:
            cv2.circle(img ,(x,y),radius,(0,255,0),2)
    return img


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
