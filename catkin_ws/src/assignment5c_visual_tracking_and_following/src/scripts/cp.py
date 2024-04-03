#!/usr/bin/env python3

import cv2
import numpy as np

def nothing(x):
    pass

winname = "Color Picker"

cv2.namedWindow("Color Picker")
#cv2.resizeWindow("Color Picker", 320, 240)

cv2.createTrackbar('H min',winname,0,255,nothing)
cv2.createTrackbar('S min',winname,0,255,nothing)
cv2.createTrackbar('V min',winname,0,255,nothing)
cv2.createTrackbar('H max',winname,0,255,nothing)
cv2.createTrackbar('S max',winname,0,255,nothing)
cv2.createTrackbar('V max',winname,0,255,nothing)

cv_image = cv2.imread("thin_blue_line.png")
height, width, channels = cv_image.shape
        
        # crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
crop_image = cv_image[int((height/2))+0:-1,:]
#cv_image = cv2.resize(cv_image, (320, 240))

while(True):
    cv_image = cv2.imread("thin_blue_line.png")
    H = cv2.getTrackbarPos('H min', 'Color Picker')
    S = cv2.getTrackbarPos('S min', 'Color Picker')
    V = cv2.getTrackbarPos('V min', 'Color Picker')
    H2 = cv2.getTrackbarPos('H max', 'Color Picker')
    S2 = cv2.getTrackbarPos('S max', 'Color Picker')
    V2 = cv2.getTrackbarPos('V max', 'Color Picker')

    print(H, S, V)
    print(H2, S2, V2)
    print("\n\n")

    hsv = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([H,S,V])
    upper_bound = np.array([H2,S2,V2])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    final = cv2.bitwise_and(crop_image, crop_image, mask)
    cv2.imshow("Color Picker", final)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()    




