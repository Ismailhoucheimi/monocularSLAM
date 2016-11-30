import ekfupdate
import ekfpredict
import numpy as np
import quat
import sharedTypes
import cv2
import ekfLandMarkInit

#vid = cv2.VideoCapture('./media/ForwardMovementID.mp4') #identify video source, 0 = webcam
#ST.cam.vidSource = cv2.VideoCapture(0) #webcam
#print vid.isOpened()
#ekfLandMarkInit.ekfNewLandMark(5,7)
#ekfupdate.ekfupdate(np.array([0]),np.array([1,2]))

#q = np.array([1,-2,3,2])
#q = q/np.linalg.norm(q)
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

print dnormq_dq

Sigma = np.ones((19,19))

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

print Sigma
#q = np.array([1,-2,3,2])
#q = q/np.linalg.norm(q)
#print quat.dRot_dQr(q)
#print quat.dRot_dQi(q)
#print quat.dRot_dQj(q)
#print quat.dRot_dQk(q)
#ekfpredict.ekfpredict(0.1)
#ekfupdate.ekfupdate(np.array([0]),np.array([10,10]).T)