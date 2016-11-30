import numpy as np
import quat
import math
import sharedTypes as ST
import pdb 
def ekfpredict(deltaT):
	print "EKF PREDICT"
	print "Time: ", deltaT
	deltaT = deltaT/1000.0;
	mu = ST.mu
	Sigma = ST.Sigma
	#deltaT = 0.5
	#mu = 
	#mu = np.array([0]*3+[1]+[0]*3+[0.5]*6+[1]*6)
	#Sigma = np.diag([0.5]*19)
	#Sigma = np.flipud(Sigma)
	
	mu_pred = mu.copy()
	Sigma_pred = Sigma.copy()
	#mu 13x1 state vector mu = (r,q,v,omega)
	r_prev = mu[0:3]
	q_prev = mu[3:7]
	v_prev = mu[7:10]
	omega_prev = mu[10:13]
	
	omega = omega_prev

	#q_prev = np.array([1,0,0,0])

	print "Mu prev", mu[0:13]
	#omega = np.array(omega)
	angle = np.linalg.norm(deltaT*omega);
	axis = omega*deltaT/angle 
	#omega = omega.tolist()
	ap = quat.aa2quat(angle,axis) #quat representing rotation


	r = r_prev + deltaT*v_prev
	q = quat.mult(q_prev,ap)
	v = v_prev
	#print q_prev
	#print "AXIS" ,q[1:4]/np.linalg.norm(q[1:4])
	#q = np.array([1,0,0,0])
	#print "q predicted", q

	mu_pred[0:3] = r
	mu_pred[3:7] = q
	mu_pred[7:10] = v
	mu_pred[10:13] = omega


	dim = len(mu)
	G = np.zeros((dim,dim))
	F = np.zeros((13,13))


	drt_drtp = np.eye(3)
	dvt_dvtp = np.eye(3)
	domega_domegap = np.eye(3)

	drt_dvtp = deltaT*np.eye(3)

	ar = ap[0]
	ai = ap[1]
	aj = ap[2]
	ak = ap[3]

	dqt_dqtp = np.array([[ar,-ai,-aj,-ak],[ai,ar,ak,-aj],[aj,-ak,ar,ai],[ak,aj,-ai,ar]])

	qpr = q_prev[0]
	qpi = q_prev[1]
	qpj = q_prev[2]
	qpk = q_prev[3]


	dqt_dap = np.array([[qpr,-qpi,-qpj,-qpk],[qpi,qpr,-qpk,qpj],[qpj,qpk,qpr,-qpi],
		[qpk,-qpj,qpi,qpr]])

                     
	N_omegap = np.linalg.norm(omega_prev)

	coeff =  -math.sin(N_omegap*deltaT/2.0)*deltaT/N_omegap/2.0
	dapr_domegax = omega_prev[0]*coeff
	dapr_domegay = omega_prev[1]*coeff
	dapr_domegaz = omega_prev[2]*coeff

	coeff1 = math.cos(N_omegap*deltaT/2.0)*deltaT/2.0/N_omegap**2
	coeff2 = math.sin(N_omegap*deltaT/2.0)/N_omegap
	dapi_domegax = coeff1*omega_prev[0]**2 + coeff2*(1-omega_prev[0]**2/N_omegap**2) 
	dapj_domegay = coeff1*omega_prev[1]**2 + coeff2*(1-omega_prev[1]**2/N_omegap**2) 
	dapk_domegaz = coeff1*omega_prev[2]**2 + coeff2*(1-omega_prev[2]**2/N_omegap**2)

	coeff3 = (math.cos(N_omegap*deltaT/2.0)*deltaT/2.0 -
	 (math.sin(N_omegap*deltaT/2.0)/N_omegap))/N_omegap**2
	dapi_domegay = coeff3 * omega_prev[0]*omega_prev[1]
	dapi_domegaz = coeff3 * omega_prev[0]*omega_prev[2]
	dapj_domegax = coeff3 * omega_prev[0]*omega_prev[1]
	dapj_domegaz = coeff3 * omega_prev[1]*omega_prev[2]
	dapk_domegax = coeff3 * omega_prev[0]*omega_prev[2]
	dapk_domegay = coeff3 * omega_prev[1]*omega_prev[2]

	dap_domegap = np.array([[dapr_domegax,dapr_domegay,dapr_domegaz],
		[dapi_domegax,dapi_domegay,dapi_domegaz],
		[dapj_domegax,dapj_domegay,dapj_domegaz],
		[dapk_domegax,dapk_domegay,dapk_domegaz]])

	dqt_domegap = np.dot(dqt_dap,dap_domegap)

	F[0:3,0:3] 	= drt_drtp
	F[0:3,7:10] = drt_dvtp
	F[3:7,3:7]	= dqt_dqtp
	F[3:7,10:13] = dqt_domegap
	F[7:10,7:10] = dvt_dvtp
	F[10:13,10:13] = domega_domegap

	G[0:13,0:13] = F

	#vmax = (ST.v_max*deltaT)**2
	vmax = 0.1
	omegamax = 0.1
	#omegamax = (ST.omega_max*deltaT)**2

	D = [vmax]*3
	D.extend([omegamax]*3)

	V = np.diag(D)

	F_hat = F[0:13,7:13]

	Sigma_m = Sigma[0:13,0:13]
	R = F_hat.dot(V).dot(F_hat.transpose())
	Sigma_pr = F.dot(Sigma_m).dot(F.transpose()) + R
	Sigma_pred[0:13,0:13] = Sigma_pr

	if len(Sigma) > 13:
		Sigma_pred[0:13,13:] = np.dot(F,Sigma[0:13,13:])
		Sigma_pred[13:,0:13] = np.dot(Sigma[13:,0:13],F.T) 

	ST.Sigma = Sigma_pred
	ST.mu = mu_pred
	print "Predicted mu", mu_pred[0:13]
	#print "R", R
	#print "State: ", mu_pred
	#print "cov: ", np.array(Sigma_pred)[0:7,8:]
	
	#print "Sigma", Sigma