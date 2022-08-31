import cv2
import numpy as np


root = "/home/hgy/.robot/data/maps/lvisam/lvisam-ori.png"
img = cv2.imread(root, 0)
kernel = np.ones((10, 10), np.uint8)
img_dilate = cv2.erode(img, kernel, 2)

cv2.imshow("dilate", img_dilate)
cv2.waitKey(0)
cv2.destroyAllWindows() 

cv2.imwrite("/home/hgy/.robot/data/maps/lvisam/lvisam.png", img_dilate)