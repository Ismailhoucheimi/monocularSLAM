import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('2.jpg',1)
img = cv2.resize(img, (0,0), fx=0.1, fy=0.1) 
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

gray = np.float32(gray)
dst = cv2.cornerHarris(gray,2,3,0,0.04)
img[dst>0.01*dst.max()] = [0,0,255]


cv2.imshow('Image 1', img)
cv2.waitKey(0)
cv2.destroyAllWindows()