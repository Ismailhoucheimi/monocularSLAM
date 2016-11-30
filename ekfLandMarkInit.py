# -*- coding: utf-8 -*-
"""
Created on Fri Apr  8 17:22:44 2016

@author: jeffery
"""

import sharedTypes as ST;
import numpy as np;
import quat;

def ekfNewLandMark(u,v):
    #print 'Pixel:',u,v;
    mean = ST.mu;
    sigma = ST.Sigma;
    #mean = np.array([0]*3 + [1] + [0]*9)
    #sigma = np.zeros((13,13))
    #K = ST.cam.K
    #u0 = K[0,2]
    #v0 = K[1,2]
    #fx = K[0,0]
    #fy = K[1,1]
    #u0 = np.array([160.2232])
    #v0 = np.array([128.8661])
    u0 = np.array([300.0])
    v0 = np.array([300.0])
    fx = np.array([250.0])
    fy = np.array([250.0])
    q = mean[3:7];
    q2r = quat.rot(q);
    #q = np.array([1,-2,3,2])
    #q = q/np.linalg.norm(q)
    #q2r = quat.rot(q)
    th = np.array([-(u0-u)/float(fx),-(v0-v)/float(fy),[1]])
    #th = np.array([(u0-u)/float(fx),(v0-v)/float(fy),[1]])    
    h = np.dot(q2r,th) #might be -(u0-u) !! note 

    #print fx,fy,u0-u,v0-v
    #print [[fx*(u0-u)],[fy*(v0-v)],[1]]
    
    #theta = np.arctan2(h[0],h[2])
    #phi = np.arctan2(-h[1],np.sqrt(h[0]**2+h[2]**2))
    theta = np.arctan(h[0]/h[2])
    phi = np.arctan(-h[1]/np.sqrt(h[0]**2+h[2]**2))    
    rho = 0.1
    

    f = np.array([mean[0],mean[1],mean[2],theta,phi,rho])
    mean = np.append(mean,f,axis = 0);
    
    #print h
    
    expSigma = np.zeros((sigma.shape[0]+3,sigma.shape[0]+3));


    dhDq = np.hstack((quat.dRot_dQr(q).dot(th),quat.dRot_dQi(q).dot(th),
        quat.dRot_dQj(q).dot(th),quat.dRot_dQk(q).dot(th)));
    dThetaDh = np.array([h[2]/(h[0]**2+h[2]**2),[0],-1*h[0]/(h[0]**2+h[2]**2)]);  #dtheta_dgw
    dPhiDh = np.array([(h[0]*h[1])/((h[0]**2+h[1]**2+h[2]**2)*np.sqrt(h[0]**2+h[2]**2)),(-1*np.sqrt(h[0]**2+h[2]**2))/(h[0]**2+h[1]**2+h[2]**2),
        (h[1]*h[2])/((h[0]**2+h[1]**2+h[2]**2)*np.sqrt(h[0]**2+h[2]**2))]); #dphi_dgw
    dThetaDq = np.dot(dThetaDh.T,dhDq); #dtheta_dqwr
    dPhiDq = np.dot(dPhiDh.T,dhDq); #dphi_dqwr
    dyDq = np.concatenate((np.zeros((3,4)),dThetaDq,dPhiDq,np.zeros((1,4))),axis = 0); #dy_dqwr
    dyDr = np.concatenate((np.eye(3),np.zeros((3,3))),axis = 1).T; #dy_drw
    dyDxv = np.concatenate((dyDr,dyDq, np.zeros((6,6))),axis = 1); #dy_dxv
    dyprimaDgw = np.concatenate((np.zeros((3,3)),dThetaDh,dPhiDh),axis = 1).T; #dyprima_dgw
    dgwDgc = q2r; #dgw_dgc
    dgcDhu = np.array([[-1.0/fx[0],0,0],
                       [0,1.0/-fy[0],0]]).T;# dgc_dhu
    dhuDhd = np.eye(2);#dgc_dhu        
    dyprimaDh = np.dot(np.dot(np.dot(dyprimaDgw,dgwDgc),dgcDhu),dhuDhd);#dyptima_dhd
    
    temp1 = np.concatenate((dyprimaDh, np.zeros((5,1))), axis=1)
    temp2 = np.array([[0,0,1]])
    dyDhd = np.concatenate((temp1,temp2),axis=0); #dy_dhd
    Ri = 10*np.eye(2);
    
    temp1 = np.concatenate((Ri,np.zeros((2,1))), axis=1)
    temp2 = np.array([[0,0,0.5**2]])
    pAdd = np.concatenate((temp1,temp2),axis=0);
    
    
    P_xv = sigma[0:13,0:13];
    P_yxv = sigma[13:,0:13];
    P_y = sigma[13:,13:];
    P_xvy = sigma[0:13,13:];
    P31 = np.dot(dyDxv,P_xv);
    P32 = np.dot(dyDxv,P_xvy);
    P33 = np.dot(np.dot(dyDxv,P_xv),dyDxv.T)+np.dot(np.dot(dyDhd,pAdd),dyDhd.T);
    row1 = np.concatenate((P_xv,P_xvy,P31.T),axis=1);
    row2 = np.concatenate((P_yxv,P_y,P32.T),axis=1);
    row3 = np.concatenate((P31,P32,P33),axis=1);
    print dThetaDh
    expSigma = np.concatenate((row1,row2,row3),axis=0);

    ST.Sigma = expSigma
    ST.mu = mean
    
    print "theta, phi, rho", theta, phi, rho
    print "cov",ST.Sigma
    return expSigma,mean
    '''mean = ST.mu;
    sigma = ST.Sigma;
    #mean = np.array([0]*3 + [1] + [0]*9)
    #sigma = np.zeros((13,13))
    #K = ST.cam.K
    #u0 = K[0,2]
    #v0 = K[1,2]
    #fx = K[0,0]
    #fy = K[1,1]
    u0 = np.array([160.2232])
    v0 = np.array([123.8661])

    fx = np.array([194.0625])
    fy = np.array([194.0625])

    #q2r = quat.rot(mean[3:7]);
    q = np.array([1,-2,3,2])
    q = q/np.linalg.norm(q)
    print "q", q
    q2r = quat.rot(q)
    print q2r
    th = np.array([-(u0-u)/float(fx),-(v0-v)/float(fy),[1]])
    h = np.dot(q2r,th) #might be -(u0-u) !! note 

    print "h: ", h
    #print fx,fy,u0-u,v0-v
    #print [[fx*(u0-u)],[fy*(v0-v)],[1]]

    theta = np.arctan(h[0]/h[2])
    phi = np.arctan(-h[1]/np.sqrt(h[0]**2+h[2]**2))
    rho = 0.1
    

    f = np.array([mean[0],mean[1],mean[2],theta,phi,rho])
    mean = np.append(mean,f,axis = 0);
    
    #print h
    
    expSigma = np.zeros((sigma.shape[0]+3,sigma.shape[0]+3));

    J = np.zeros((sigma.shape[0]+6,sigma.shape[0]+3));
    J[:sigma.shape[0],:sigma.shape[0]] = np.eye(sigma.shape[0]);
    J[sigma.shape[0]:sigma.shape[0]+6,0:3] = np.vstack((np.eye(3),np.zeros((3,3))));

    dhDq = np.hstack((quat.dRot_dQr(q).dot(th),quat.dRot_dQi(q).dot(th),
        quat.dRot_dQj(q).dot(th),quat.dRot_dQk(q).dot(th)));
    dThetaDh = np.array([[h[2]/(h[0]**2+h[2]**2)],[0],[-1*h[0]/(h[0]**2+h[2]**2)]]);  #dtheta_dgw
    dPhiDh = np.array([(h[0]*h[1])/((h[0]**2+h[1]**2+h[2]**2)*np.sqrt(h[0]**2+h[2]**2)),(-1*np.sqrt(h[0]**2+h[2]**2))/(h[0]**2+h[1]**2+h[2]**2),
        (h[1]*h[2])/((h[0]**2+h[1]**2+h[2]**2)*np.sqrt(h[0]**2+h[2]**2))]); #dphi_dgw


    print "dThetaDh",dThetaDh
    print "dPhiDh", dPhiDh
    print "dhDq", dhDq
    
    J[sigma.shape[0]:sigma.shape[0]+6,3:7] =np.vstack((np.zeros((3,4)),np.dot(dThetaDh.T,dhDq),np.dot(dPhiDh.T,dhDq),np.zeros((1,4))));#dy_dqwr
    dYDh = np.zeros((6,3));
    dYDh[3,0:3] = dThetaDh[0,0:3];
    dYDh[4,0:3] = dPhiDh[0,0:3];
    dHDuv = np.zeros((3,2));
    dHDuv[0,0] = q2r[0,0] * -1/fx;
    dHDuv[1,0] = q2r[1,0] * -1/fx;
    dHDuv[0,1] = q2r[0,1] * -1/fy;
    dHDuv[1,1] = q2r[1,1] * -1/fy;
    
    J[sigma.shape[0]:sigma.shape[0]+6,sigma.shape[0]:sigma.shape[0]+2] = np.dot(dYDh,dHDuv);
    J[sigma.shape[0]:sigma.shape[0]+6,sigma.shape[0]+2] = [0,0,0,0,0,1];    
    
    Q = 2*np.eye(2);
    expSigma[0:sigma.shape[0],0:sigma.shape[1]] = sigma;
    expSigma[sigma.shape[0]:sigma.shape[0]+2,sigma.shape[1]:sigma.shape[1]+2] = Q;
    expSigma[sigma.shape[0]+2,sigma.shape[1]+2] = 0.25;
    sigma = np.dot(np.dot(J,expSigma),J.T);
    ST.Sigma = sigma
    ST.mu = mean
    
    print "theta, phi, rho", theta, phi, rho
    return sigma,mean'''
if __name__ == "__main__":
    ST.mu=np.array([0,0,0,1,0,0,0,0.5,0.5,0.5,0.5,0.5,0.5,1,1,1,1,1,1]);
    ST.Sigma = np.fliplr(0.5*np.eye(19));
    print ST.Sigma.shape
    sigma = ekfNewLandMark(5,7);