#siftDetector.py
#Created 03/31/2016, kevkchoi
#Purpose: return location of template match on template 


import cv2
import numpy as np
import sharedTypes as ST
from matplotlib import pyplot as plt

def siftTemplate(img_template):
    sift = cv2.xfeatures2d.SURF_create()
    img_template  = cv2.cvtColor(img_template,  cv2.COLOR_RGB2GRAY)
    kp_template, des_template = sift.detectAndCompute(img_template,None)
    return kp_template, des_template


def siftQuery(img_query, initMask = None):
    sift = cv2.xfeatures2d.SURF_create()
    img_query  = cv2.cvtColor(img_query,  cv2.COLOR_RGB2GRAY)
    kp_query, des_query = sift.detectAndCompute(img_query,initMask) #image,mask
    return kp_query, des_query


def siftMatch(kp_template, des_template, kp_query, des_query, matchThreshold, seeAllMatches=True): 
    bf = cv2.BFMatcher()
    if len(kp_query) > 0 and len(kp_template) > 0: 
         matches = bf.knnMatch(des_template,trainDescriptors = des_query, k=2)
    else:
        matches = []
    
    # Apply ratio test
    siftKeyPoints = []
    good = []
    maxKeyPts = 10
    if np.shape(matches)[0]>0 and np.shape(matches)[1] == 2:
        for mat in matches:
            if mat[0].distance < matchThreshold*mat[1].distance: #default should be 0.7
                m = mat[0]
                if m.queryIdx < len(kp_query):
                    Query_Idx = m.trainIdx
                    good.append([m])
                    siftKeyPoints.append(np.floor(kp_query[Query_Idx].pt).astype(int))
            if seeAllMatches:
                pass
            else:
                break
            maxKeyPts = maxKeyPts - 1
            if maxKeyPts < 0:
                break
    return siftKeyPoints


if __name__ == '__main__':
    ST.init()
    img_template = cv2.imread('./media/02.png')          # templateImage
#    img_template = cv2.imread('./media/2.jpg')          # templateImage
    img_query = cv2.imread('./media/ForwardMovement.png') # queryImage 
    template_kp, template_des = siftTemplate(img_template)
    query_kp, query_des = siftQuery(img_query)
    good, keyPts = siftMatch(template_kp, template_des, query_kp, query_des, 0.7)
    
    A = np.zeros((1,1)) #dummy matrix
    img3 = cv2.drawMatchesKnn(img_template,template_kp,img_query,query_kp,good,A,flags=2)
    plt.imshow(img3),plt.show()

    print siftKeyPoints




############# DEPRECATED CODE ###################
'''
def siftDetector(img_template, img_query, initMask = None): #returns image with keypoints circled, and coordinate of keypoint
    img_query_org = img_query
    # Convert color to gray
    img_template  = cv2.cvtColor(img_template,  cv2.COLOR_RGB2GRAY)
    img_query  = cv2.cvtColor(img_query,  cv2.COLOR_RGB2GRAY)

    # Initiate SIFT detector
    sift = cv2.xfeatures2d.SURF_create()

    # find the keypoints and descriptors with SIFT
    kp_template, des_template = sift.detectAndCompute(img_template,None)

#    if ST.globalLandMarkInitFlags[0] == False: #first time through, check if 1st LM is init
#        kp_query, des_query = sift.detectAndCompute(img_query,initMask) #image,mask (search entire pic)
#    else: #after intial detection, only search in mask area 
#        kp_query, des_query = sift.detectAndCompute(img_query,ST.landMarkList[0].mask) #image,mask

    kp_query, des_query = sift.detectAndCompute(img_query,initMask) #image,mask



    NNmethod = "BF"
    if NNmethod == "BF":
        # BFMatcher with default params
        bf = cv2.BFMatcher()
        if len(kp_query) > 0 and len(kp_template) > 0: 
             matches = bf.knnMatch(des_template,trainDescriptors = des_query, k=2)
            #matches = bf.knnMatch(des_query,des_template, k=2)
        else:
            print "No Matches"
            matches = []
    else:
        #FLANN parameters !!DON'T USE, THERE IS AN OPENCV PYTHON BINDING ISSUE!!
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary
        flann = cv2.FlannBasedMatcher(index_params,search_params,crossCheck=True)
        matches = flann.knnMatch(des_template,des_query,k=2)


    seeAllMatches = True
    # Apply ratio test
    good = []
    siftKeyPoints = []
    maxKeyPts = 10
    if np.shape(matches)[0]>0 and np.shape(matches)[1] == 2:
        for mat in matches:
            if mat[0].distance < 0.5*mat[1].distance: #default should be 0.7
                m = mat[0]
                if m.queryIdx < len(kp_query):
                    #good.append([m])
                    Query_Idx = m.trainIdx
                    siftKeyPoints.append(np.floor(kp_query[Query_Idx].pt).astype(int))
            if seeAllMatches:
                pass
            else:
                break
            maxKeyPts = maxKeyPts - 1
            if maxKeyPts < 0:
                break
    
    img_test_match = np.zeros_like(img_query_org)

    if len(siftKeyPoints) > 0:
        if seeAllMatches: #choose the mode here
            for pt in siftKeyPoints: #draw circles around each key point
                img_test_match = cv2.circle(img_test_match,(pt[0], pt[1]), 5, color = (0,0,255), thickness = 1)
#            img_test_match = img_query_org
            centerPt = findCentroid(siftKeyPoints)
            siftKeyPoints = [centerPt]

        else: #(2) only pick best match
            firstMatch =  np.floor(kp_query[good[0][0].queryIdx].pt).astype(int)
            firstMatch = (firstMatch[0], firstMatch[1])
#            img_test_match = cv2.circle(img_query_org, firstMatch, 10, color = (0,0,255), thickness = 1)

        # cv2.drawMatchesKnn expects list of lists as matches.
#        A = np.zeros((1,1)) #dummy matrix
#        img3 = cv2.drawMatchesKnn(img_template,kp_template,img_query,kp_query,good,A,flags=2)
#        plt.imshow(img3),plt.show()

#    img_test_match = cv2.bitwise_and(img_test_match,img_test_match, mask=initMask)

    return img_test_match, siftKeyPoints

def findCentroid(listOfPts):
    u = 0
    v = 0
    for point in listOfPts:
        u = u+point[0]
        v = v+point[1]
    u = (u/float(len(listOfPts))).astype(int)
    v = (v/float(len(listOfPts))).astype(int)
    return [u,v]

'''
