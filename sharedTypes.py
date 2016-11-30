#SharedTypes.py
#Created 03/22/2016, Kevin Choi
#Purpose: Initiate objects that can be accessed by several files
#Includes variables and classes that can be shared between multiple files
#Variables should be declared as global

import numpy as np
import cv2

x=0 #m
y=0 #m
z=0 #m
q=0 #m
v=0.1 #m/s
w= (np.pi/180.0)*10.0 #rad/s

#Called by main.py during start up
def init():
    global key
    key = 1
    global state
    state = 0

    global v_max
    v_max = 0

    global omega_max
    omega_max = 0

    global cam
    cam = Camera()
    global window

    
    global mu

    mu = np.array([0]*3+[1]+[0]*3+[0]*2+[0.00001]+[0]*2+[0.00001]).T


    global Sigma
    Sigma = np.diag([0]*7 + [0.001]*3 + [0.005]*3)
   
    
    #globalLandMarkInit.py
    global LM_list
    LM_list = LandMarkList()

#Stores frames from video, includes methods for preprocessing
class Camera:
    def __init__(self):
        self.d = np.zeros((3,3)) #distortion coeff
        self.K = np.zeros((1,5)) #distortion coeff

        #Video Source
        self.frame = np.zeros((600,600,1)) #video frame
        self.vidSource = 'blank.mp4'

    def loadCalParameters(self, filename):
        with np.load(filename) as calData:
            self.d = calData['distCoeff.npy']
            self.K = calData['intrinsic_matrix.npy']
    
    def undistortFrame(self):
        # read one image
        h, w = self.frame.shape[:2]
        # undistort
        newcamera, roi = cv2.getOptimalNewCameraMatrix(self.K, self.d, (w,h), alpha = 1 ,centerPrincipalPoint = 0)
        self.frame = cv2.undistort(self.frame,self.K, self.d, None, newcamera)

    def setFrame(self, newFrame):
        self.frame = newFrame

class Window:
    def __init__(self):
        (y,x,z) = cam.frame.shape #(480,640,1)#cam.frame.shape
        print "WINDOW:", y,x,z
        border = 20
        liveZone = np.zeros((y,x), np.uint8)
        liveZone[border:y-border,border:x-border]=1
        exitingZone = np.ones((y,x), np.uint8)
        exitingZone = cv2.subtract(exitingZone, liveZone)
        #split the window into 6 zones
        self.zone = [np.zeros_like(exitingZone) for temp in range(6)]
        xleft = 20
        xsplit = x/2
        xright = x-20
        ytop = 20
        ysplit = y/2
        ybot= y-20

        self.zone[0] = liveZone #sum of zones 1 to 4
        self.zone[1][ytop:ysplit,xleft:xsplit] = 1
        self.zone[2][ysplit:ybot,xsplit:xright] = 1
        self.zone[3][ysplit:ybot,xleft:xsplit] = 1
        self.zone[4][ysplit:ybot,xsplit:xright] = 1
        self.zone[5] = exitingZone #LM likely to disappear if in this zone

class OptFlow: 
    def __init__(self, frame):
        self.currentFrame = frame
        self.oldFrame = frame
        self.keyPts = []


class LandMark: 
    def __init__(self, imageOfLM, u,v):
        self.loc2D = np.array([[u,v]]).T #use this to define the center of the ROI
        self.loc3D = np.array([[0,0,0]]).T #use this to define the landmark in 3D coord
        self.LMimage = imageOfLM
        #SIFT & TEMPLATEMATCH Stuff
        (y,x,z) = cam.frame.shape
        self.mask = np.zeros((y,x), np.uint8)
        self.initState = False
        self.mask_width = 80
        self.mask_height = 80
        self.sift_kp = [] #keypoints for sift
        self.sift_des = [] #descriptors of keypoints
        self.matchThreshold = 0.5
        #OPTFLOW
        self.old_gray=cv2.cvtColor(cam.frame, cv2.COLOR_BGR2GRAY)
        self.p0 = None
        #STATUS
        self.detected = False
        self.fully_init = False
        self.seen = 0
        self.unseen = 0
        self.unseenThres_ResetSeen = 15
        #Identification
        self.ID = None
        
    def updatePosition(self, u,v):
        self.loc2D[0,0] = u
        self.loc2D[1,0] = v
        self.detected = True
        self.unseen = 0
        self.updateMask()

    def updateMask(self): #create rect. ROI mask around landMark location
        self.mask[:,:] = 0
        u1 = self.loc2D[0,0] - int(self.mask_width/2)
        u2 = self.loc2D[0,0] + int(self.mask_width/2) 
        v1 = self.loc2D[1,0] - int(self.mask_height/2)
        v2 = self.loc2D[1,0] + int(self.mask_height/2)
        #Threshold
        if u1 < 0: 
            u1 = 0
        if u2 >= np.size(self.mask,1):
            u2 = np.size(self.mask,1) - 1
        if v1 < 0: 
            v1 = 0
        if v2 >= np.size(self.mask,0):
            v2 = np.size(self.mask,0) - 1
        self.mask[v1:v2, u1:u2] = 1 #create the new mask
    

class LandMarkList:
    def __init__(self):
        self.preload = [] #holds LM objects
        self.active = [] #holds LM objects
        self.passive = [] #holds LM objects
        self.scan = [] #holds LM objects
        self.allLMID = [] #holds list of all LM Id's
        self.allLM = [] #holds LM objects
        self.totalLM = 0

    def addToActive(self, LM):
        if LM.ID == None: #new landmark
            self.active.append(LM)
            self.totalLM += 1
            LM.ID = self.totalLM-1 
            self.allLM.append(LM)
            self.allLMID.append(LM.ID)
            print "new LM -> active, ", LM.ID
        else: #not new 
            added = False
            for Q in self.passive:
                if Q.ID == LM.ID:
                    self.passive.remove(Q)
                    self.active.append(Q)
                    added = True
                    print "passive -> active, ", LM.ID
                    break
            if added == False:
                self.active.append(LM)
                print "LM -> active, ", LM.ID
   
    def removeFromActive(self,LM):
        for Q in self.active:
            if Q.ID == LM.ID:
                self.active.remove(Q)
                self.passive.append(Q)
                print "active -> passive, ", LM.ID
                break

    def addToPreload(self,LM):
        self.preload.append(LM)
    
    def removeFromPreload(self,LM):
         self.preload.remove(LM)
         self.addToActive(LM)

    def status(self):
        print "Active List: ", [x.ID for x in self.active]
        print "Passive List: ", [y.ID for y in self.passive]
        print "Scan List", [z.ID for z in self.scan]
        print "All LM List", [a.ID for a in self.allLM]
        print "All LM Index list", self.allLMID

