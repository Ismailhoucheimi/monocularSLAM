#template.py
#Created 03/22/2016, Username
#Purpose: This is a template for python scripts with standard libraries imported
#Some of the commonly used global variables/objects are listed as well

import cv2
import numpy as np
import sharedTypes as ST #this includes the global variables/objects
import exitScript
import pdb
#define script local variables here
A = 1

#write functions here
def templateMatch(templateImg, queryImg,initMask=None):
    templateImg = cv2.cvtColor(templateImg, cv2.COLOR_BGR2GRAY) 
    queryImg = cv2.cvtColor(queryImg, cv2.COLOR_BGR2GRAY) 
    if initMask != None: 
        queryImg = cv2.multiply(queryImg,initMask)
    w, h = templateImg.shape[::-1]

    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
               'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
    
    meth = eval(methods[3])
    res = cv2.matchTemplate(image=queryImg,templ=templateImg, method=meth)
    min_val, max_val, min_loc,max_loc = cv2.minMaxLoc(res)

    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    if meth in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        top_left = min_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
    else:
        top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
    
    #ST.cam.frame =  cv2.rectangle(ST.cam.frame,top_left, bottom_right, 255, 2)
    
    x = int((top_left[1]+bottom_right[1])/2.0)
    y = int((top_left[0]+bottom_right[0])/2.0)
    keyPts = [[x,y]]
    
    return keyPts


#if script is called directly from terminal, useful for debugging methods
if __name__ == "__main__":
    print "This was called from terminal."
    ST.init() #initializes global variables and classes
    ST.cam.vidSource = cv2.VideoCapture('./media/ForwardMovementID.mp4')
    cv2.namedWindow('frame')
    cv2.namedWindow('Masks') 

    initMask = np.zeros((600,600),dtype =np.uint8) #creating empty matrix, size of video frame
    initMask[300:500,300:500] = 1
    
    template = cv2.imread('./media/Tag16h6.png')

    while(1):
        ret, ST.cam.frame = ST.cam.vidSource.read() #read
        ST.cam.frame = cv2.imread('./media/aprilTagTest.png') 
        
        keyPts = templateMatch(template, ST.cam.frame ) #process
        print keyPts
        cv2.imshow('frame', ST.cam.frame) #show

        #waits 30ms for keyboard stroke, pauses video stream
	ST.key = cv2.waitKey(30) & 0xff
        if ST.key == 27:
            exitScript.exitScript(vidSource) #easy to use quit script
