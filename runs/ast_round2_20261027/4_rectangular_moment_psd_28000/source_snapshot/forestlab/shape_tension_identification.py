"""Nonnegative shape-support inertial fit using the measured integral system.

The convex hull of an observed surface is a conservative support envelope for
nonconvex objects. It is not a reconstruction of their hidden density.
"""
from __future__ import annotations
import numpy as np
from .inertial_identification import fit,decode


def support_basis(surface,spacing=.08,include_vertices=True):
    from scipy.spatial import ConvexHull
    p=np.asarray(surface,float);hull=ConvexHull(p)
    axes=[np.arange(p[:,j].min()+spacing/2,p[:,j].max(),spacing) for j in range(3)]
    candidates=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
    eq=hull.equations
    keep=(candidates@eq[:,:3].T+eq[:,3]<=1e-6).all(axis=1)
    inside=candidates[keep]
    vertices=p[hull.vertices]
    if len(vertices)>80:vertices=vertices[np.linspace(0,len(vertices)-1,80,dtype=int)]
    points=np.vstack([inside,vertices]) if include_vertices else inside
    B=np.zeros((10,len(points)));B[0]=1.;B[1:4]=points.T
    square=np.sum(points**2,axis=1)
    B[4:7]=(square[:,None]-points**2+.00005).T
    B[7]=-points[:,0]*points[:,1];B[8]=-points[:,0]*points[:,2];B[9]=-points[:,1]*points[:,2]
    return B,points


def uniform_shape_parameters(surface,mass=1.05):
    B,points=support_basis(surface,include_vertices=False)
    if len(points)<4:raise ValueError('Too few volumetric scan support points')
    return B@np.full(len(points),mass/len(points))


def fit_shape_supported(rows,anchors,surface,window_sec=.4,tension_floor=.01):
    import clarabel
    from scipy import sparse
    result=fit(rows,anchors,window_sec=window_sec,tension_floor=tension_floor)
    B,points=support_basis(surface)
    A=np.asarray(result['system']['A']);b=np.asarray(result['system']['b'])
    design=A@B;n=B.shape[1]
    # A tiny distribution regularizer resolves indistinguishable moment
    # representations, without supplying the unknown density to the fit.
    P=design.T@design+1e-8*np.eye(n);q=-design.T@b
    constraints=np.vstack([-np.eye(n),np.ones(n),-np.ones(n)])
    rhs=np.r_[np.zeros(n),1.5,-.7]
    settings=clarabel.DefaultSettings();settings.verbose=False
    settings.tol_feas=1e-9;settings.tol_gap_abs=1e-10;settings.tol_gap_rel=1e-9
    solution=clarabel.DefaultSolver(sparse.csc_matrix(P),q,sparse.csc_matrix(constraints),rhs,
        [clarabel.NonnegativeConeT(n+2)],settings).solve()
    result['unconstrained_theta']=result['theta'].copy()
    result['unconstrained_physical']=bool(result.get('decoded') and result['decoded']['physical'])
    result['support_solver_status']=str(solution.status)
    if str(solution.status) not in ('Solved','AlmostSolved'):
        result['activated']=False;result['activation_refusal']='shape_fit_solver';return result
    weights=np.asarray(solution.x);theta=B@weights;decoded=decode(theta)
    result.update(theta=theta,decoded=decoded,distribution_weights=weights,support_points=points,
        support_description='Convex hull of reconstructed surface; conservative envelope for nonconvex objects',
        support_residual_rmse=float(np.sqrt(np.mean((A@theta-b)**2))),
        activated=bool(result['rank']==10 and result['excitation_passed'] and result['tension_gate_passed'] and decoded['physical']))
    return result
