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

    for LM in ST.LM_list.active:
        LM.matchThreshold = 0.7

    #searchScript.landMarkSearch("TEMPLATEMATCH")
    scanned = searchScript.landMarkSearch("SIFT", ST.LM_list.active,seeAllMatches=True)
    for LM in scanned:
        ST.LM_list.scan.append(LM)
    

    for LM in ST.LM_list.active:
        if LM.unseen>10:
            LM.matchThreshold = 0.4
            ST.LM_list.removeFromActive(LM)
            print "Removed LM ", LM.ID
            return True
    
    return False

#if script is called directly from terminal, useful for debugging methods
if __name__ == "__main__":
    #firstTimeRunning = True
    ST.init()
    run()


