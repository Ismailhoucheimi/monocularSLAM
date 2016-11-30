#globalLandMarkInit.py
#Created 03/31/2016, kevkchoi
#Purpose: Looks for 4 corners of a black rectangle based on 4 templates

import cv2
import numpy as np
import sharedTypes as ST #this includes the global variables/objects
import exitScript
import siftDetector
import searchScript

def run():
     

    for LM in ST.LM_list.passive:
        ST.LM_list.addToActive(LM)
        LM.mask = ST.window.zone[0]
        LM.detected = False
        LM.seen = 0
        LM.unseen = 1000


    scanned = searchScript.landMarkSearch("SIFT",ST.LM_list.active)
    for LM in scanned:
        ST.LM_list.scan.append(LM)
    
    totalFound = 0
    for LM in ST.LM_list.active:
        if LM.seen > 20:
            LM.matchThreshold = 0.7
            totalFound += 1
    
#    if totalFound == 3:
#        return True
#    else:
#        return False

    for LM in ST.LM_list.active:
        if LM.unseen>10 and LM.detected == True:                                                                         
            LM.matchThreshold = 0.4
            LM.detected = False
            ST.LM_list.removeFromActive(LM)
            print "Removed LM ", LM.ID


    for LM in ST.LM_list.active:
        if LM.detected == False:
            return False #return to this function
        
    for LM in ST.LM_list.passive:
        if LM.detected == False:
            return False #return to this function
     
    return True #if all LM are detected, move to next state

#if script is called directly from terminal, useful for debugging methods
if __name__ == "__main__":
    #firstTimeRunning = True
    ST.init()
    run()


