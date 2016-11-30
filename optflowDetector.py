#template.py
#Created 03/22/2016, Username
#Purpose: This is a template for python scripts with standard libraries imported
#Some of the commonly used global variables/objects are listed as well

import cv2
import numpy as np
import sharedTypes as ST #this includes the global variables/objects
import exitScript

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                 qualityLevel = 0.3,
                 minDistance = 7,
                 blockSize = 7 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
color = np.random.randint(0,255,(100,3))

#write functions here
def optflow(p0, old_gray, newImg):
    new_gray = cv2.cvtColor(newImg, cv2.COLOR_BGR2GRAY)

    #calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None, **lk_params)

    #select good new points
    good_new = p1[st==1]
    good_old = p0[st==1]

    #draw the tracks
    a,b = good_new[0].ravel()
    keyPts = [[a,b]]
    #book keeping
    old_gray = new_gray.copy()
    p0 = good_new.reshape(-1,1,2)
    
    return keyPts, p0, old_gray

def shiTomasi(old_gray,mask=None):
    p0 =  cv2.goodFeaturesToTrack(old_gray, mask, **feature_params)
    return p0


#if script is called directly from terminal, useful for debugging methods
if __name__ == "__main__":
    print "This was called from terminal."
    ST.init() #initializes global variables and classes
    ST.cam.vidSource = cv2.VideoCapture('./media/testVid.mp4')
    cv2.namedWindow('frame')
    ret, ST.cam.frame = ST.cam.vidSource.read()

    LM = ST.LandMark(ST.cam.frame,0,0)
    LM.old_gray = cv2.cvtColor(ST.cam.frame, cv2.COLOR_BGR2GRAY)
    LM.p0 = cv2.goodFeaturesToTrack(LM.old_gray, mask=None, **feature_params)
    while(1):
        ret, ST.cam.frame = ST.cam.vidSource.read() #read
        keyPts, LM.p0, LM.old_gray = optflow(LM.p0, LM.old_gray, ST.cam.frame) #process
        mask = np.zeros_like(LM.old_gray)

        [a,b]=keyPts[0]
        ST.cam.frame = cv2.circle(ST.cam.frame, (a,b), 5, (0,0,255), -1)

        cv2.imshow('frame', ST.cam.frame) #show

        #waits 30ms for keyboard stroke, pauses video stream
	ST.key = cv2.waitKey(30) & 0xff
        if ST.key == ord('q'):
            exitScript.exitScript(vidSource) #easy to use quit script
