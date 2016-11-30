#globalLandMarkInit.py
#Created 03/31/2016, kevkchoi
#Purpose: Looks for 4 corners of a black rectangle based on 4 templates

import cv2
import numpy as np
import sharedTypes as ST #this includes the global variables/objects
import exitScript
import siftDetector
import searchScript
import media

firstTimeRunning = True

def run():
    global firstTimeRunning
    if firstTimeRunning:
        firstTimeRunning = False
        matchMask = np.zeros_like(ST.cam.frame)
        '''
        template1 = cv2.imread('./media/01.png')#/Tag16h6.png') #Tag16h6_small.png')
        template2 = cv2.imread('./media/04.png')#Tag25h7.png')
        template3 = cv2.imread('./media/02.png')#Tag25h9.png')
       # template1 = cv2.imread('./media/Tag16h6.png') #Tag16h6_small.png')
       # template2 = cv2.imread('./media/Tag25h7.png')
       # template3 = cv2.imread('./media/Tag25h9.png')
        #will be added to active list via initilization
        LM1 = ST.LandMark(template1,0,0)
        LM2 = ST.LandMark(template2,0,0)
        LM3 = ST.LandMark(template3,0,0)
        ST.LM_list.preload.append(LM1)
        print "Loaded query images as landmarks"
        print ST.LM_list.status()
        LM1.sift_kp, LM1.sift_des = siftDetector.siftTemplate(template1)
        LM2.sift_kp, LM2.sift_des = siftDetector.siftTemplate(template2)
        LM3.sift_kp, LM3.sift_des = siftDetector.siftTemplate(template3)
        '''
        listOfTemplateNames = media.imagesList
        for name in listOfTemplateNames:
            image = cv2.imread(name)
            LM = ST.LandMark(image,0,0)
            LM.matchThreshold = 0.5
            LM.sift_kp, LM.sift_des = siftDetector.siftTemplate(image)
            ST.LM_list.preload.append(LM)
        print "Loaded query images as landmarks"
        print ST.LM_list.status()
        
    scanned = searchScript.landMarkSearch("SIFT", ST.LM_list.active)
    for LM in scanned:
        ST.LM_list.scan.append(LM)
    
    if len(ST.LM_list.preload)>0:
        scanned = searchScript.landMarkSearch("SIFT", ST.LM_list.preload)
        for LM in scanned:
            ST.LM_list.preload.remove(LM)
            ST.LM_list.addToActive(LM)
            ST.LM_list.scan.append(LM)
    
    totalFound = 0
    for LM in ST.LM_list.active:
        if LM.seen > 3:
            LM.matchThreshold = 0.7
            totalFound += 1

    if totalFound == len(media.imagesList):
        return True
    else:
        return False

#if script is called directly from terminal, useful for debugging methods
if __name__ == "__main__":
    #firstTimeRunning = True
    ST.init()
    run()


