from collections import deque

import cv2 as cv
import numpy as np
import imutils
import matplotlib.pyplot as plt

centers = {"side": [], "front": []}
rads = {"side": [], "front": []}
frames = {"side": [], "front": []}

def brightness_contrast(img, brightness=200, contrast=100):
    brightness = int((brightness - 0) * (255 - (-255)) / (510 - 0) + (-255))
    contrast = int((contrast - 0) * (127 - (-127)) / (254 - 0) + (-127))

    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            max = 255
        else:
            shadow = 0
            max = 255 + brightness
        al_pha = (max - shadow) / 255
        ga_mma = shadow
        # The function addWeighted calculates
        # the weighted sum of two arrays
        cal = cv.addWeighted(img, al_pha,
                             img, 0, ga_mma)
    else:
        cal = img
    if contrast != 0:
        Alpha = float(131 * (contrast + 127)) / (127 * (131 - contrast))
        Gamma = 127 * (1 - Alpha)
        # The function addWeighted calculates
        # the weighted sum of two arrays
        cal = cv.addWeighted(cal, Alpha,
                             cal, 0, Gamma)
    # putText renders the specified text string in the image.
    cv.putText(cal, 'B:{},C:{}'.format(brightness,
                                       contrast), (10, 30),
               cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    return cal

def apply_orange(img):
    mask = brightness_contrast(img)
    orange_space = [(0, 120, 98), (54, 255, 255)]
    # mask = cv.erode(mask, None, iterations=4)
    # mask = cv.dilate(mask, None, iterations=4)

    return cv.inRange(mask, orange_space[0], orange_space[1])

def find_circle(frame, mask, label):
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,
                            cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(c)
        M = cv.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        # if radius > 10:
        #     # draw the circle and centroid on the frame,
        #     # then update the list of tracked points
        #     cv.circle(frame, (int(x), int(y)), int(radius),
        #                (0, 255, 255), 2)
        #     cv.circle(frame, center, 5, (0, 0, 255), -1)
    # update the points queue
    centers[label].append(center)
    rads[label].append(radius)

def trim_data(label):
    average = np.average(rads[label])
    for r in rads[label]:
        if abs((r - average)/average) > .2:
            i = rads[label].index(r)
            rads[label].pop(i)
            centers[label].pop(i)



def fit_curve(label, width, height):
    cs = centers[label]
    xs = []
    ys = []

    for c in cs:
        xs.append(c[0])
        ys.append(c[1])
    model = np.polyfit(xs, ys, 2)

    def apply_poly(x):
        return int(model[0]*pow(x,2)+ model[1]*x + model[2])

    out = []

    for i in range(0, int(width), 10):
        j = apply_poly(i)
        if j < height:
            out.append([i,j])

    return np.asarray(out)

def draw_parabola(frame, model, color):
    for point in model:
        x = point[0]
        y = point[1]
        cv.circle(frame, (x,y), 10, color, -1)
    return frame

def main():
    side = cv.VideoCapture('angle.mov')
    front = cv.VideoCapture('front.mov')
    while side.isOpened() and front.isOpened():
        s_ret, s_frame = side.read()
        f_ret, f_frame = front.read()
        if not (s_ret or f_ret):
            print("Can't receive frame (stream end?)")
            break
        if s_frame is None or f_frame is None:
            break

        s_frame = cv.GaussianBlur(s_frame, (11, 11), 0)
        f_frame = cv.GaussianBlur(f_frame, (11, 11), 0)


        s_hsv = cv.cvtColor(s_frame, cv.COLOR_BGR2HSV)
        f_hsv= cv.cvtColor(f_frame, cv.COLOR_BGR2HSV)

        s_orange = apply_orange(s_hsv)
        f_orange = apply_orange(f_hsv)

        find_circle(f_frame, f_orange, "front")
        find_circle(s_frame, s_orange, "side")

    side.release()
    front.release()
    cv.destroyAllWindows()

    side = cv.VideoCapture('angle.mov')
    front = cv.VideoCapture('front.mov')

    width = int(side.get(cv.CAP_PROP_FRAME_WIDTH))  # float `width`
    height = int(side.get(cv.CAP_PROP_FRAME_HEIGHT))

    trim_data("front")
    trim_data("side")

    f_model = fit_curve("front", width, height)
    s_model = fit_curve("side", width, height)

    # poly_linspace = np.linspace(100, width, height)

    while side.isOpened() and front.isOpened():
        s_ret, s_frame = side.read()
        f_ret, f_frame = front.read()

        if not (s_ret or f_ret):
            print("Can't receive frame (stream end?)")
            break
        if s_frame is None or f_frame is None:
            break

        red = [255,0,0]
        f_result = draw_parabola(f_frame, f_model, red)

        cv.imshow("front frame", f_result)
        cv.waitKey(3000)


if __name__ == "__main__":
    main()



