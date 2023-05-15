
import numpy as np
import cv2 as cv
import cv2
import matplotlib.pyplot as plt


def get_2D_pos_prec(image1=None, debug = False, fig_name = "1", normalize = True, w_pixels = 900, h_pixels = 900):
    img_gray = cv2.cvtColor(image1, cv2.COLOR_RGB2GRAY)
    #ret, img_binary = cv2.threshold(img_gray, 130, 255, cv2.THRESH_BINARY_INV)  # 130, 255 #BU Important
    ret, img_binary = cv2.threshold(img_gray, 120, 255, cv2.THRESH_BINARY_INV) #130, 255 #90 phantom
    #kernel = np.ones((7, 7), np.uint8) #3 phantom
    #img_binary = cv2.erode(img_binary, kernel, iterations=1)
    #kernel = np.ones((11, 11), np.uint8)
    #img_binary = cv2.dilate(img_binary, kernel, iterations=1)

    if debug == True:
        """plt.figure(fig_name)
        plt.imshow(img_binary, cmap=plt.cm.gray)
        plt.pause(0.00001)
        plt.clf()"""
        cv2.namedWindow(fig_name, cv2.WINDOW_NORMAL)
        cv2.imshow(fig_name, img_binary)
        k = cv2.waitKey(1)

    contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    xC, yC= 0,0
    checkMore = False
    for cnt in contours:
        M = cv.moments(cnt)
        if M['m00']!=0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            area = cv.contourArea(cnt)
            # print(area)
            if area > 10:
                if (checkMore == True):
                    print(None)
                checkMore = True
                xC, yC = cx, cy
                #print("X: " + str(cx) + " Y: " + str(cy))
    if normalize == True:
        xC = (w_pixels / 2 - xC) / (w_pixels / 2) * 1
        yC = (h_pixels / 2 - yC) / (h_pixels / 2) * 1

    return round(-xC,3), round(yC,3)

