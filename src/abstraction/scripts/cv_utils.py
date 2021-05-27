import numpy as np
import cv2
import random
import time

def calculate_corners(original):
    """
    get back the inside corners of the chessboard with the findChessboardCorners cv2 functtion
    and shape tthe corners into a 7x7 array
    """
    chessboardSize = (7, 7)

    # Termination Criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

    img = original.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #gray = cv2.fastNlMeansDenoisingMulti(gray, 2, 5, None, 4, 7, 35)

    # thr_img = remove_shadow(img)
    cv2.imshow('tresh', gray)
    cv2.waitKey(2)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

    # If found, refine corners
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        corners2 = corners2.ravel()
        corners2 = np.reshape(corners2, (7, 7, 2))
        return corners2
    return None


def get_squares(corners, img):
    """
    We draw the squares of the board onto the image by taking the inside corners
    and calculating the distance of those. after that we connect the rest of our corners
    """
    xtl = corners[0][0][0] - (corners[0][1][0] - corners[0][0][0])
    ytl = corners[0][0][1] - (corners[1][0][1] - corners[0][0][1])

    xtr = corners[0][6][0] + (corners[0][6][0] - corners[0][5][0])
    ytr = corners[0][6][1] - (corners[1][6][1] - corners[0][6][1])

    xbl = corners[6][0][0] - (corners[6][1][0] - corners[6][0][0])
    ybl = corners[6][0][1] + (corners[6][0][1] - corners[5][0][1])

    xbr = corners[6][6][0] + (corners[6][6][0] - corners[6][5][0])
    ybr = corners[6][6][1] + (corners[6][6][1] - corners[5][6][1])

    top_points = np.array([[xtl, ytl]])
    bottom_points = np.array([[xbl, ybl]])

    new_corners = np.array([])
    for i in range(len(corners)):

        # Left outer line
        xl = corners[i][0][0] - (corners[i][1][0] - corners[i][0][0])
        yl = (corners[i][0][1])
        left = np.array([[xl, yl]])

        # Right outer line
        xr = corners[i][6][0] + (corners[i][6][0] - corners[i][5][0])
        yr = (corners[i][6][1])
        right = np.array([[xr, yr]])

        # top outer line
        xt = corners[0][i][0]
        yt = corners[0][i][1] - (corners[1][i][1] - corners[0][i][1])
        top_points = np.vstack((top_points, np.array([[xt, yt]])))

        # bottom outer line
        xb = corners[6][i][0]
        yb = corners[6][i][1] + (corners[6][i][1] - corners[5][i][1])
        bottom_points = np.vstack((bottom_points, np.array([[xb, yb]])))
        row = np.vstack((left, corners[i], right))

        if len(new_corners) > 0:
            new_corners = np.vstack((new_corners, np.array([row])))
        else:
            new_corners = np.array([row])

    top_points = np.vstack((top_points, [[xtr, ytr]]))
    bottom_points = np.vstack((bottom_points, [[xbr, ybr]]))
    corners = np.vstack((np.array([top_points]), new_corners, np.array([bottom_points])))
    img = crop_on_board(img, xtl, ytl, xtr, ytr, xbl, ybl, xbr, ybr)
    return corners, img

def detect_piece(roi, piece_cascade, white_cascade, black_cascade):
    scaleVal = 1.1
    pieces = piece_cascade.detectMultiScale(roi, scaleVal, 1)
    piece = None
    if len(pieces) > 0:
        white_count = 0
        black_count = 0
        white_pieces = white_cascade.detectMultiScale(roi, scaleVal, 1)
        black_pieces = black_cascade.detectMultiScale(roi, scaleVal, 1)
        while white_count < 10 and black_count < 10 and scaleVal < 1.5:
            scaleVal += 0.01
            if len(black_pieces) > 0:
                black_count += 1
            if len(white_pieces) > 0:
                white_count += 1
            black_pieces = black_cascade.detectMultiScale(roi, scaleVal, 1)
            white_pieces = white_cascade.detectMultiScale(roi, scaleVal, 1)
        if len(white_pieces) == 10 or len(white_pieces) > len(black_pieces):
            return 0
        elif len(black_pieces) == 10 or len(black_pieces) > len(white_pieces):
            return 1
        elif len(black_pieces) == 0 and len(white_pieces) == 0:
            return -1
        return -1
    return None

def find_move(white_cascade, black_cascade, chess_cascade, previous_placement, corners, img):
    placement = previous_placement
    x1 = x2 = y1 = y2 = None
    possible_begin_moves = []
    possible_end_moves = []
    names = [['WHITE', (0,255,0)], ['BLACK', (255,0,255)]]
    #img = cv2.Canny(img, 100, 200)
    for i in range(len(corners) - 1):
        for j in range(len(corners[i]) - 1):
            x_start = corners[i][j][0]
            x_end = corners[i][j + 1][0]
            y_start = corners[i][j][1]
            y_end = corners[i + 1][j][1]
            wT, hT, _ = img.shape
            points = np.float32([(x_start, y_start), (x_end, y_start),
                                 (x_start, y_end), (x_end, y_end)])
            roi = warpImg(img, points, hT, wT)
            roi = cv2.resize(roi, (100,101))
            piece = detect_piece(roi, chess_cascade, white_cascade, black_cascade)
            if piece is None and previous_placement[i][j] is not None:
                possible_begin_moves.append((i,j))
            elif piece is not None and previous_placement[i][j] is None:
                possible_end_moves.append((i,j))
            # elif piece == 1:
            #     if previous_placement[i][j] == 0 or previous_placement is None:
            #         possible_end_moves.append((j, i))
            # elif piece == 0:
            #     if previous_placement[i][j] == 1 or previous_placement is None:
            #         possible_end_moves.append((j, i))
            # else:
            #     piece = previous_placement[i][j]
            placement[i][j] = piece
            if piece is not None:
                cv2.putText(img, 'PIECE', (int(x_start),int(y_start)+15), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,0,255), 2)
    #print(placement)
    cv2.imshow('img', img)
    cv2.waitKey(2)
    return placement, possible_begin_moves, possible_end_moves


def get_player(roi):
    pass


def warpImg(img, points, w, h, inv=False):
    """
    Warp the image to the given points
    """

    pts1 = np.float32([points])
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)

    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp


def crop_on_board(img, xtl, ytl, xtr, ytr, xbl, ybl, xbr, ybr):
    """
    crop the image so that only the playable board is visable
    """
    wT, hT, _ = img.shape
    points = np.float32([(xtl, ytl), (xtr, ytr),
                         (xbl, ybl), (xbr, ybr)])
    img = warpImg(img, points, hT, wT)
    return img


def rand_color():
    """
    get random
    """
    r = random.randrange(256)
    g = random.randrange(256)
    b = random.randrange(256)

    return (r, g, b)


def get_move(corners, img, previous_images):
    total_difference = np.zeros_like(img)
    for image in previous_images:
        diff1 = cv2.subtract(image, img)
        diff2 = cv2.subtract(img, image)
        #diff = difference(img, image)
        total_difference = cv2.addWeighted(total_difference, 1, diff1, 1, 0.0)
        total_difference = cv2.addWeighted(total_difference, 1, diff2, 1, 0.0)

    square_sums = []
    for i in range(len(corners) - 1):
        for j in range(len(corners[i]) - 1):
            x_start = corners[i][j][0]
            x_end = corners[i][j + 1][0]
            y_start = corners[i][j][1]
            y_end = corners[i + 1][j][1]
            wT, hT = total_difference.shape
            points = np.float32([(x_start, y_start), (x_end, y_start),
                                 (x_start, y_end), (x_end, y_end)])

            roi = warpImg(total_difference, points, hT, wT)
            s = np.sum(roi)
            if len(square_sums) == 2:
                if s > square_sums[0][2]:
                    square_sums[0] = [i, j, s]
                elif s > square_sums[1][2]:
                    square_sums[1] = [i, j, s]
            elif s > 12000000:
                square_sums.append([i, j, s])
    x1 = None
    y1 = None
    x2 = None
    y2 = None
    if len(square_sums) == 1:
        x1 = square_sums[0][0]
        y1 = square_sums[0][1]
    elif len(square_sums) == 2:
        x1 = square_sums[0][0]
        y1 = square_sums[0][1]
        x2 = square_sums[1][0]
        y2 = square_sums[1][1]
    return x1, y1, x2, y2
    return total_difference


def remove_shadow(img):
    dilated_img = cv2.dilate(img, np.ones((3, 3), np.uint8))
    bg_img = cv2.medianBlur(dilated_img, 21)
    diff_img = 255 - cv2.absdiff(img, bg_img)
    norm_img = diff_img.copy()  # Needed for 3.x compatibility
    cv2.normalize(diff_img, norm_img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    _, thr_img = cv2.threshold(norm_img, 230, 0, cv2.THRESH_TRUNC)
    cv2.normalize(thr_img, thr_img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    return thr_img


###########
# NOT USED
###########

def draw_circles2(img, corners):
    blue = 0
    red = 255
    count = 0
    for i in range(len(corners)):
        for j in range(len(corners[i])):
            x = corners[i][j][0]
            y = corners[i][j][1]
            cv2.circle(img, (x, y), 5, (blue, 0, red), -1)
            if count % 2 == 0:
                red -= 5
            else:
                blue += 5
            count += 1
    return img


def draw_circles(img):
    current_img = cv2.medianBlur(img, 5)
    current_img = cv2.cvtColor(current_img, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(current_img, cv2.HOUGH_GRADIENT, 1, 20, param1=75, param2=15, minRadius=15, maxRadius=30)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        has_found_pieces = True
        for (x, y, radius) in circles:
            cv2.circle(img, (x, y), radius, (0, 255, 0), 2)
    return img
