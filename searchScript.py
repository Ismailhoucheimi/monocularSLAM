#globalLandMarkInit.py
#Created 03/31/2016, kevkchoi
#Purpose: Looks for 4 corners of a black rectangle based on 4 templates
import pdb
import cv2
import numpy as np
import sharedTypes as ST #this includes the global variables/objects
import exitScript
import siftDetector
import templateDetector
import optflowDetector
matchMask = np.zeros(1)


def landMarkSearch(MODE,listOfInterest,seeAllMatches=True):
    global matchMask
    matchMask[:] = 0
    scanned = [] #clear the "scan" list
    

    ### Search through Active Landmark List ###
    for LM in listOfInterest: 
        #use "initialization mask" if not detected
        if LM.detected == False: 
            initMask = cv2.add(ST.window.zone[0], ST.window.zone[5]) #use left side of cam
            initMask[initMask > 0] = 1
           ### We want a search mask that EXCLUDES other landmark search windows ###
            for otherLM in listOfInterest:
                if otherLM.ID != LM.ID: #make sure its not the same LM
                    if otherLM.detected == True:
                        initMask = initMask - otherLM.mask

            query_kp, query_des = siftDetector.siftQuery(ST.cam.frame, initMask)
        #use LM's mask if already detected before
        else:
            initMask = LM.mask #use the search window of the landmark as mask
            if MODE == "SIFT":
                query_kp, query_des = siftDetector.siftQuery(ST.cam.frame, initMask)
            elif MODE == "TEMPLATEMATCH":
                pass
            elif MODE == "OPTFLOW":
                if LM.p0 == None:
                    LM.p0 = optflowDetector.shiTomasi(LM.old_gray,mask = LM.mask)
            else:
                "NO MODE SELECTED"

#        y1 = ST.cam.frame.shape[0]
#        x1 = ST.cam.frame.shape[1]
#        white = 255*np.ones((y1,x1),dtype=np.uint8)
#        showMask = cv2.multiply(initMask,white)
#        cv2.imshow('Masks', showMask)

        #search for matches between landmark and current camera frame
        if MODE == "SIFT":
            keyPts = siftDetector.siftMatch(LM.sift_kp,LM.sift_des,query_kp,query_des, LM.matchThreshold, seeAllMatches)
        elif MODE == "TEMPLATEMATCH":
            keyPts = templateDetector.templateMatch(LM.LMimage,ST.cam.frame, initMask)
        elif MODE == "OPTFLOW":
            keyPts, LM.p0, LM.old_gray == optflowDetector.optflow(LM.p0, LM.old_gray, ST.cam.frame)
        
        ### MATCHES DETECTED ###
        if len(keyPts)>0:
            rectColor = (0,255,0)
            #draw circles over keypoints
            for pt in keyPts:                           
                matchMask = cv2.circle(matchMask,(pt[0], pt[1]), 5, color = (0,0,255), thickness =     1)
            #find centroid if there are multiple keyPts
            if len(keyPts)>1:
                keyPts = findCentroid(keyPts)
            #update landmark position
            LM.seen += 1
            LM.updatePosition(keyPts[0][0], keyPts[0][1]) #updates LM.unseen and LM.detected
            #append landmark to "scan" list
            scanned.append(LM)
            
        ### NO MATCHES ###
        else:
            LM.unseen += 1
            rectColor = (0,0,255)
            #if landmark hasn't been seen for awhile
            if LM.unseen > LM.unseenThres_ResetSeen:
                LM.seen = 0

        #draw tracking window
        if LM.detected == True: 
            matchMask = cv2.add(matchMask, drawTrackingWindow(LM,rectColor))
            
            matchMask = cv2.putText(matchMask,text = str(str(LM.ID)+","+str(LM.seen)),
                        org = (LM.loc2D[0,0],LM.loc2D[1,0]),
                        fontFace = cv2.FONT_HERSHEY_COMPLEX,
                        fontScale = 0.5,
                        color = rectColor, 
                        thickness = 1, 
                        lineType = cv2.LINE_AA)


    ### Paint on the matchMask to camera frame ###
    ST.cam.frame = cv2.add(ST.cam.frame, matchMask)
    return scanned

def findCentroid(listOfPts):#don't use numpy, its slow 
    u = 0
    v = 0
    for point in listOfPts:
        u = u+point[0]
        v = v+point[1]
    u = (u/float(len(listOfPts))).astype(int)
    v = (v/float(len(listOfPts))).astype(int)
    return [[u,v]]

def drawTrackingWindow(LM,rectColor): 
    xa = LM.loc2D[0,0]-int(LM.mask_width/2)
    xb = LM.loc2D[0,0]+int(LM.mask_width/2)
    ya = LM.loc2D[1,0]-int(LM.mask_width/2)
    yb = LM.loc2D[1,0]+int(LM.mask_width/2)
    return cv2.rectangle(np.zeros_like(ST.cam.frame), (xa,ya), (xb,yb), rectColor,2)   

def drawROIMask():
    ROI_mask = np.zeros_like(ST.cam.frame)
    (height, width,temp) = np.shape(ROI_mask)
    x1 = width/0
    x2 = 3*width/4
    y1 = height/4
    y2 = 3*height/4
    ROI_mask[y1:y2,x1:x2] = 1
    rect_mask = cv2.rectangle(np.zeros_like(ST.cam.frame), (x1,y1), (x2,y2), (0,255,0),2)     
    return cv2.add(ST.cam.frame, rect_mask)


#if script is called directly from terminal, useful for debugging methods
if __name__ == "__main__":
    #firstTimeRunning = True
    ST.init()
    run()

