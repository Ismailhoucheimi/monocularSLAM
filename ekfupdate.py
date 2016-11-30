import numpy as np
import sharedTypes as ST
import math
import quat
import ekfLandMarkInit

def ekfupdate(index, z):
    print "EKF UPDATE"
    print "Measurements: ", z
    print "Index", index
    #index nparray
    Sigma = ST.Sigma.copy()
    mu = ST.mu.copy()
    #K = ST.cam.K

    #K = np.eye(3)
    #q = np.array([1,-2,3,2])
    #q = q/np.linalg.norm(q)
    #q = q.tolist()
    #print q
    #mu = np.array([1]*3+q+[1]*6 +[1]*6)
    #Sigma = np.diag([0.5]*19)


    ind = 13+6*np.array(index)
    N = np.shape(index)[0]
    #measurement function
    r = mu[0:3]
    q = mu[3:7].copy()
    V = mu[7:10]
    omega = mu[10:13]

    #q = np.array([1,0,0,0])

    qconj = quat.conj(q).copy()
    Rcw = np.array(quat.rot(qconj))
    #print Rcw

    hp = []

    H = np.zeros((2*N,len(mu)))

    dq2r_dqr = quat.dRot_dQr(qconj)
    dq2r_dqi = quat.dRot_dQi(qconj)
    dq2r_dqj = quat.dRot_dQj(qconj)
    dq2r_dqk = quat.dRot_dQk(qconj)

    dqconj_dq = np.diag([1,-1,-1,-1])   

    #Loop start
    k = 0
    for i in ind:

        #hi_inverse_depth checks whether point is in front of camera and if the projection is on the image

        Xm = mu[i:i+3]
        theta = mu[i+3]
        phi = mu[i+4]
        rho = mu[i+5]
        
        h = Rcw.dot(rho*(Xm - r) + m(theta,phi))
        print theta,phi,m(theta,phi)

        #u0 = K[0,2]
        #v0 = K[1,2]
        #fx = K[0,0]
        #fy = K[1,1]

        u0 = 300
        v0 = 300
        fx = 250
        fy = 250

        hxi = h[0]
        hyi = h[1]
        hzi = h[2]

        #u = u0 - fx*hxi/hzi #Reprojection. + sign in Matlab
        #v = v0 - fy*hyi/hzi

        u = u0 + fx*hxi/hzi #Reprojection. + sign in Matlab. note
        v = v0 + fy*hyi/hzi

        #u = 100
        #v = 150

        X = np.array([u,v])
        hp = np.concatenate((hp,X.T),0)
        #print "zhat", hp
        dh_dhi = np.zeros((2,3))
        dh_dhi[0,0] = +fx/hzi
        dh_dhi[0,2] = -fx*hxi/(hzi**2)
        dh_dhi[1,1] = +fy/hzi
        dh_dhi[1,2] = fy*hyi/(hzi**2)  #Checked
        dhi_dr = -rho*Rcw

        di = rho*(Xm - r) + m(theta,phi)
        print Rcw
        dhi_dqconj = np.zeros((3,4))
        dhi_dqconj[:,0] = dq2r_dqr.dot(di)
        dhi_dqconj[:,1] = dq2r_dqi.dot(di)
        dhi_dqconj[:,2] = dq2r_dqj.dot(di)
        dhi_dqconj[:,3] = dq2r_dqk.dot(di)

        dhi_dq = np.dot(dhi_dqconj,dqconj_dq)

        dhi_dX = rho*Rcw 
        dhi_dtheta = Rcw.dot([math.cos(theta)*math.cos(phi), 0 , -math.sin(theta)*
            math.cos(phi)])
        dhi_dphi = Rcw.dot(np.array([-math.sin(theta)*math.sin(phi), 
            -math.cos(phi), -math.cos(theta)*math.sin(phi)]).T)
        dhi_drho = Rcw.dot(Xm - r)

        dhi_df = np.zeros((3,6))
        dhi_df[:,0:3] = dhi_dX
        dhi_df[:,3] = dhi_dtheta
        dhi_df[:,4] = dhi_dphi
        dhi_df[:,5] = dhi_drho

        dh_dr = np.dot(dh_dhi,dhi_dr) #dh_dhrl * dhrl_drw
        dh_dq = np.dot(dh_dhi,dhi_dq) 
        dh_df = np.dot(dh_dhi,dhi_df)


        dh_dx = np.zeros((2,13))
        dh_dx[:,0:3] = dh_dr
        dh_dx[:,3:7] = dh_dq
        
        H[k:k+2,0:13] = dh_dx 
        H[k:k+2,i:i+6] = dh_df
        k = k+1
        #print H

    Q = 10*np.eye(2*N)

    S = ((H.dot(Sigma.dot(H.T)) + Q))
    Sinv = np.linalg.inv(S)
    Kt = Sigma.dot(H.T).dot(Sinv)
    
    mu = mu.copy() + Kt.dot(z-hp)
    KH = Kt.dot(H) 
    I = np.eye(len(KH))
    Sigma = (I - KH).dot(Sigma.copy())
    #Sigma = Sigma - Kt.dot(S.dot(Kt.T))
    Sigma = 0.5*Sigma.copy()+0.5*Sigma.T.copy()

    q = mu[3:7]
    qr = q[0]
    qi = q[1]
    qj = q[2]
    qk = q[3]


    #print qr,qi,qj,qk
    #print [-qr*qk, -qi*qk, -qj*qk, qr**2+qi**2+qj**2]
    Qr = np.array([[qi**2+qj**2+qk**2,-qr*qi, -qr*qj,-qr*qk],
        [-qr*qi,qr**2+qj**2+qk**2,-qi*qj,-qi*qk],
        [-qr*qj,-qi*qj,qr**2+qi**2+qk**2,-qj*qk],
        [-qr*qk, -qi*qk, -qj*qk, qr**2+qi**2+qj**2]])

    dnormq_dq = (qr**2 + qi**2 + qj**2 + qk**2)**(-1.5)*Qr
    Sigma_q = dnormq_dq.dot(Sigma[3:7,3:7].dot(dnormq_dq.T))

    

    #Sigma[0:3,0:3] = Sigma[0:3,0:3]
    Sigma[0:3,3:7] = np.dot(Sigma[0:3,3:7].copy(),dnormq_dq.T)
    Sigma[3:7,0:3] = np.dot(dnormq_dq,Sigma[3:7,0:3].copy())
    Sigma[3:7,3:7] = Sigma_q.copy()
    Sigma[3:7,7:] = np.dot(dnormq_dq,Sigma[3:7,7:].copy())
    Sigma[7:,3:7] = np.dot(Sigma[7:,3:7].copy(),dnormq_dq.T)

    mu[3:7] = q/np.linalg.norm(q)

    ST.Sigma = Sigma
    ST.mu = mu

    V = mu[7:10]
    omega = mu[10:13]
    v_max = np.max(V)
    #v_max = 0.01
    
    omega_max = np.max(omega)
    #omega_max = 0.01
    print "Updated mu:", mu

    if v_max > ST.v_max:
        ST.v_max = v_max
    if omega_max > ST.omega_max:
        ST.omega_max = omega_max

    #print "V, omega", mu[7:10], mu[10:13]


    hf = []
    r = mu[0:3]
    #REMOVE
    for i in ind:
        Xm = mu[i:i+3]
        theta = mu[i+3]
        phi = mu[i+4]
        rho = mu[i+5]
        
        h = Rcw.dot(rho*(Xm - r) + m(theta,phi))
        print theta,phi,m(theta,phi)

        #u0 = K[0,2]
        #v0 = K[1,2]
        #fx = K[0,0]
        #fy = K[1,1]

        u0 = 300
        v0 = 300
        fx = 250
        fy = 250

        hxi = h[0]
        hyi = h[1]
        hzi = h[2]

        #u = u0 - fx*hxi/hzi #Reprojection. + sign in Matlab
        #v = v0 - fy*hyi/hzi

        u = u0 + fx*hxi/hzi #Reprojection. + sign in Matlab. note
        v = v0 + fy*hyi/hzi
        X = np.array([u,v])
        hf = np.concatenate((hf,X.T),0)
        
    print 'z,hp:', z,hp;
    print "zhatupdated", hf

def m(theta,phi):
    m = np.array([math.sin(theta)*math.cos(phi), -math.sin(phi),
        math.cos(theta)*math.cos(phi)])
    m = m.transpose()
    return m

def extract_measurements():
    LMlist = ST.LM_list.scan
    index = []
    z = np.array([])
    for LM in LMlist:
        index.append(LM.ID)
        u = LM.loc2D[0,0]
        v = LM.loc2D[1,0]
        z = np.concatenate((z,[u,v]),0)
    return index, z

def check_and_initialize(IDs):
    print "ID: ", IDs
    
    for i in IDs:
        if 14 + 6*i > len(ST.mu):
            LM = ST.LM_list.allLM[i]
            u = LM.loc2D[0]
            v = LM.loc2D[1]
            #print u,v
            ekfLandMarkInit.ekfNewLandMark(u,v)


def inverseDepthConversion(ID):

    Sigma = ST.Sigma[13+6*ID:19+6*ID,13+6*ID:19+6*ID];
    x = ST.mu[13+6*ID:19+6*ID];
    #x = np.array([1,1,1,0.5,0.5,0.5])
    cor = np.zeros(3);
    cor[0:3] = x[0:3];
    cor[0:3] += (1/x[5])*np.array([np.sin(x[3])*np.cos(x[4]),-1*np.sin(x[4]),np.cos(x[3])*np.cos(x[4])]);
    #print cor
    J = np.zeros((3,6));
    J[0:3,0:3] = np.eye(3);
    J[0:3,3:6] = [[(1/x[5])*np.cos(x[3])*np.cos(x[4]),(-1/x[5])*np.sin(x[3])*np.sin(x[4]),(-1/(x[5]**2))*np.sin(x[3])*np.cos(x[4])],
                  [0,(-1/x[5])*np.cos(x[4]),(1/(x[5]**2))*np.sin(x[4])],
                  [(-1/x[5])*np.sin(x[3])*np.cos(x[4]),(-1/x[5])*np.cos(x[3])*np.sin(x[4]),(-1/(x[5]**2))*np.cos(x[3])*np.cos(x[4])]];       
    cov = np.dot(np.dot(J,Sigma),J.T);  
    return cor,cov;

            