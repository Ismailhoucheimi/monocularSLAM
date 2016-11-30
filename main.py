import numpy as np
import cv2
import matplotlib.pyplot as plt
import sharedTypes as ST
import sys
import exitScript
import globalLandMarkInit
import maintainTracking
import searchForNewLM
import ekfpredict
import ekfupdate
import media
#import graphic

def showSearchMasks():
    (v,u,z) = ST.cam.frame.shape
    searchMasks = np.zeros((v,u), dtype=np.uint8)
    for LM in ST.LM_list.active:
        searchMasks = cv2.add(searchMasks, LM.mask*255)
    return searchMasks

def stateMachineRun():
    #map string to function name
    switcher = {
    "Initialize_first_LM": globalLandMarkInit.run,
    "Maintain_tracking": maintainTracking.run,
    "Search_for_new_LM": searchForNewLM.run,
    }
    func = switcher.get(ST.state) #get name of function
    success = func() #execute function, function returns 0 for fail, 1 for success
    return success




### Init. Global Variables ###
ST.init()

### Start Video Capture ###
#ST.cam.vidSource = cv2.VideoCapture('./media/ForwardMovement/ForwardMovementID.mp4') 
ST.cam.vidSource = cv2.VideoCapture(media.videoName) 

#ST.cam.vidSource = cv2.VideoCapture(0) #webcam

#Grab one frame to continue init.
ret, ST.cam.frame = ST.cam.vidSource.read()
ST.window = ST.Window()

### Camera Matrix ###
ST.cam.loadCalParameters('./calibration/calibration_data_openGL.npz')

### Init. windows ###
cv2.namedWindow('Camera') #name a window
cv2.namedWindow('Masks')

### Time ###
time_old = 0.0

### Init. State Machine ###
ST.state = "Initialize_first_LM"
print "Starting Global Init. (", ST.state, ")"
#graphic.plotInit();

### Start MONOSLAM ###
hisLoc = None;
#while(ST.cam.vidSource.isOpened()):
for AA in range(50):
    #Get next frame
    for i in range(10):
        ret, ST.cam.frame = ST.cam.vidSource.read()
    #Get the time
    time_new = ST.cam.vidSource.get(0) #millisec
    time_delta = time_new - time_old
    time_old = time_new
    #Undistort frame of camera
    ST.cam.undistortFrame()
    ### State Machine ###
    #clear the scan list of landmarks
    ST.LM_list.scan = []
    success = stateMachineRun() #perform the function for current state
    if success == True: #go to next state 
        if ST.state == "Initialize_first_LM":
            ST.state = "Maintain_tracking"
            print "Global Init ---> Search for new LM (",ST.state, ")"
        elif ST.state == "Maintain_tracking":
            ST.state = "Search_for_new_LM"
            print "Maintain tracking LM ---> Search for new LM (", ST.state, ")"
        elif ST.state == "Search_for_new_LM":
            ST.state = "Maintain_tracking"
            print "Found new LM ---> Maintain tracking (",ST.state, ")"

#    detectFeatures()
#
#    match_features()
#
#    initialize_landmarks_partial()
#
#    initialize_landmarks_full()
#
#    data_association()
#
#    EKF_predict()
    if hisLoc == None:
        hisLoc = np.array([ST.mu[0:3]]);
    else:
        hisLoc = np.vstack((hisLoc,ST.mu[0:3]));
        #graphic.plotTrajectory(hisLoc);
        print 'History Location:',hisLoc
    IDs,z = ekfupdate.extract_measurements()
    ekfupdate.check_and_initialize(IDs)
    ekfpredict.ekfpredict(time_delta)
#    EKF_update()

    ekfupdate.ekfupdate(IDs,z)
    #graphic.plotCamera(ST.mu[0:3],ST.mu[4:7]);
#    ST.cam.frame = np.zeros_like(ST.cam.frame)
    #landMark = [[-3.3,6.05,7.25],[-3.8,-4.2,8.2],[5.9,-3.1,7.5],[3.22,1.2,9.98]]
    for i in range(0,(len(ST.mu)-13)/6):
        cor,cov = ekfupdate.inverseDepthConversion(i);
        #graphic.plotParticles(cor);
        #graphic.plotEllipsoid(landMark[i],cov);
        #print 'Landmark:',cor,'Covariance:',cov
        #print 'Original states:',ST.mu[13+6*i:19+6*i]
    #graphic.plotAxis();
    #Display on screen
    cv2.imshow('Camera',ST.cam.frame) # cv2.flip(ST.cam.frame,1))
    cv2.imshow('Masks',showSearchMasks())
    #graphic.plotFlip();
    print time_delta 
    #check for ESC key
    ST.key = cv2.waitKey(10) & 0xff #read keyboard, store last 8 binaries
    if ST.key == 27:
        exitScript.exitScript(ST.cam.vidSource)
        #graphic.plotClose();
        #graphic.plotClose();
while(1):
    ST.key = cv2.waitKey(10) & 0xff
    if ST.key == 27:
        exitScript.exitScript(ST.cam.vidSource)
        #graphic.plotClose();
        #graphic.plotClose();    
    
    #graphic.plotlandmarks([0,5,0],[0,0,1],[0,0,-1000],np.fliplr(ST.cam.frame));
    #graphic.plotFlip();

#exitScript.exitScript(ST.cam.vidSource)
