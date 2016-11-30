#template.py
#Created 03/22/2016, Username
#Purpose: This is a template for python scripts with standard libraries imported
#Some of the commonly used global variables/objects are listed as well

import cv2
import numpy as np
import sharedTypes as ST #this includes the global variables/objects
import exitScript

#define script local variables here
A = 1

#write functions here
def testMethod():
    return cv2.resize(ST.cam.frame, (0,0), fx=0.5, fy=0.5)



#if script is called directly from terminal, useful for debugging methods
if __name__ == "__main__":
    print "This was called from terminal."
    ST.init() #initializes global variables and classes
    videoSource = cv2.VideoCapture('./media/testVid.mp4')
    cv2.namedWindow('frame')
        
    #The Camera class from SharedTypes is an object that stores and preprocesses
    #the video feed. Preprocessing includes undistortion and resizing. The Camera
    #object is named 'cam' (ie. cam = Camera() )    
    ST.cam.frame #this is a numpy array that stores the video frame
    ST.cam.exampleMethod() #example method
    print ST.testVar
    print ST.testArr

    blankFrame = np.zeros_like(ST.cam.frame) #creating empty matrix, size of video frame

    while(1):
        ret, ST.cam.frame = videoSource.read() #read
        processedFrame = testMethod() #process
        cv2.imshow('frame', ST.cam.frame) #show

        #waits 30ms for keyboard stroke, pauses video stream
	ST.key = cv2.waitKey(30) & 0xff
        if ST.key == ord('q'):
            exitScript.exitScript(vidSource) #easy to use quit script
