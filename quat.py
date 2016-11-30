import numpy as np
import math

def conj(q):
	conj = q
	conj[0] = q[0]
	conj[1:] = [-x for x in q[1:]]
	return np.array(conj) 

def inv(q):
    r =q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2

    inv = conj(q)/((r)*1.0)
    print r
    return np.array(inv)


def mult(q,p):
	res = [None]*4
	res[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3]
	res[1] = p[0]*q[1] + p[1]*q[0] - p[2]*q[3] + p[3]*q[2]
	res[2] = p[0]*q[2] + p[1]*q[3] + p[2]*q[0] - p[3]*q[1]
	res[3] = p[0]*q[3] - p[1]*q[2] + p[2]*q[1] + p[3]*q[0]
	return np.array(res)

def rot(q):
	q = q/float(np.linalg.norm(q))
	q0 = q[0]
	qx = q[1]
	qy = q[2]
	qz = q[3]

	R =[[q0**2+qx**2-qy**2-qz**2, 2.0*(qx*qy-q0*qz), 2.0*(qx*qz+q0*qy)], \
	[2.0*(qy*qx+q0*qz), (q0**2-qx**2+qy**2-qz**2), 2.0*(qy*qz-q0*qx)], \
	[2.0*(qz*qx-q0*qy), 2.0*(qz*qy+q0*qx), q0**2-qx**2-qy**2+qz**2]]
	return np.array(R)

def dRot_dQr(q):
    return np.multiply(2.0,[[q[0],-1*q[3],q[2]],[q[3],q[0],-1*q[1]],[-1*q[2],q[1],q[0]]]);
def dRot_dQi(q):
    return np.multiply(2.0,[[q[1],q[2],q[3]],[q[2],-1*q[1],-1*q[0]],[q[3],q[0],-1*q[1]]]);
def dRot_dQj(q):
    return np.multiply(2.0,[[-1*q[2],q[1],q[0]],[q[1],q[2],q[3]],[-1*q[0],q[3],-1*q[2]]]);
def dRot_dQk(q):
    return np.multiply(2.0,[[-1*q[3],-1*q[0],q[1]],[q[0],-1*q[3],q[2]],[q[1],q[2],q[3]]]);
def aa2quat(angle, axis):
    """Return quaternion for rotation about axis.

    >>> q = quaternion_about_axis(0.123, [1, 0, 0])
    >>> numpy.allclose(q, [0.99810947, 0.06146124, 0, 0])
    True

    """
    q = np.array([0.0, axis[0], axis[1], axis[2]])
    qlen = np.linalg.norm(q)
    if qlen > _EPS:
        q *= math.sin(angle/2.0) / qlen
    q[0] = math.cos(angle/2.0)
    return q

_EPS = np.finfo(float).eps * 4.0