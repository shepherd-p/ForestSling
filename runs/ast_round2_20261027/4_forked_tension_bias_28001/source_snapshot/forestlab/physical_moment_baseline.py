"""Support-free physically consistent least-squares baseline (4x4 moment PSD)."""
import numpy as np
def fit_moment_psd(A,b):
 import clarabel
 from scipy import sparse
 A=np.asarray(A);b=np.asarray(b)
 # Scaled upper-triangle vectorization of [[.5 tr(J) I-J, h], [h.T, m]].
 S=np.zeros((10,10));s2=np.sqrt(2.)
 S[0,4:7]=[-.5,.5,.5];S[1,7]=-s2;S[2,4:7]=[.5,-.5,.5]
 S[3,8]=-s2;S[4,9]=-s2;S[5,4:7]=[.5,.5,-.5]
 S[6,1]=s2;S[7,2]=s2;S[8,3]=s2;S[9,0]=1
 C=np.zeros((2,10));C[0,0]=1;C[1,0]=-1
 eps=np.zeros(10);eps[[0,2,5,9]]=1e-9
 settings=clarabel.DefaultSettings();settings.verbose=False
 settings.tol_feas=1e-9;settings.tol_gap_abs=1e-10;settings.tol_gap_rel=1e-9;settings.max_iter=200
 sol=clarabel.DefaultSolver(sparse.csc_matrix(A.T@A+1e-8*np.eye(10)),-A.T@b,
       sparse.csc_matrix(np.vstack([-S,C])),np.r_[-eps,1.5,-.7],
       [clarabel.PSDTriangleConeT(4),clarabel.NonnegativeConeT(2)],settings).solve()
 if str(sol.status) not in ('Solved','AlmostSolved'):raise RuntimeError('Moment PSD fit '+str(sol.status))
 return np.asarray(sol.x),dict(status=str(sol.status),iterations=sol.iterations,
       physical_constraint='Positive 4x4 pseudo-inertia, no spatial support constraint',mass_interval_kg=[.7,1.5])
