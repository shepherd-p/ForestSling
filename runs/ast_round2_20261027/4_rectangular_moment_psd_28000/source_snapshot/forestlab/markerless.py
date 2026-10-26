"""Untrained geometric RGB-D baseline; never imports asset or payload truth.

Known fixed pickup workspace supplies coarse segmentation, not forest object
detection. Reference geometry is fused only from sensor scan point clouds.
"""
from pathlib import Path
import itertools
import sys
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'.deps/markerless'))
from scipy.spatial import cKDTree


def quat_rotation(q):
    q=np.asarray(q,float);q=q/np.linalg.norm(q);w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def depth_points(raw, metadata, intrinsic, crop_center, crop_halfsize, stride=2):
    """ProjectAirSim perspective/range depth in uint16 mm, NOT planar Z."""
    if raw.dtype!=np.uint16:
        raise ValueError('Expected uint16 perspective-range depth in millimetres')
    height,width=raw.shape;yy,xx=np.mgrid[:height:stride,:width:stride]
    rays=np.stack(((xx-intrinsic[0,2])/intrinsic[0,0],
                   (yy-intrinsic[1,2])/intrinsic[1,1],np.ones_like(xx)),axis=-1)
    rays/=np.linalg.norm(rays,axis=-1,keepdims=True)
    ranges=raw[::stride,::stride].astype(float)/1000
    optical=rays*ranges[...,None]
    optical_to_frd=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
    r=quat_rotation([metadata['rot_'+k] for k in 'wxyz'])@optical_to_frd
    p=np.array([metadata['pos_'+k] for k in 'xyz'])
    points=(optical@r.T+p).reshape(-1,3)
    valid=((ranges>.1)&(ranges<20.)).ravel()
    valid &= (np.abs(points-np.asarray(crop_center))<np.asarray(crop_halfsize)).all(axis=1)
    return points[valid]


def voxel_reduce(points, resolution=.012, limit=6000):
    points=np.asarray(points,float)
    if not len(points):return np.empty((0,3))
    _,inverse=np.unique(np.floor(points/resolution).astype(np.int64),axis=0,return_inverse=True)
    sums=np.zeros((inverse.max()+1,3));np.add.at(sums,inverse,points)
    count=np.bincount(inverse);reduced=sums/count[:,None]
    if len(reduced)>limit:reduced=reduced[np.linspace(0,len(reduced)-1,limit,dtype=int)]
    return reduced


def proper_axis_permutations():
    result=[]
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product((-1,1),repeat=3):
            matrix=np.eye(3)[:,permutation]*signs
            if np.linalg.det(matrix)>.5:result.append(matrix)
    return result


def principal_axes(points):
    _,vectors=np.linalg.eigh(np.cov(points.T))
    vectors=vectors[:,::-1]
    if np.linalg.det(vectors)<0:vectors[:,-1]*=-1
    return vectors


def rigid_fit(source,target):
    cs,ct=source.mean(0),target.mean(0)
    u,_,vh=np.linalg.svd((source-cs).T@(target-ct))
    r=vh.T@u.T
    if np.linalg.det(r)<0:vh[-1]*=-1;r=vh.T@u.T
    return r,ct-r@cs


class ScanRegistration:
    def __init__(self, reference_world, voxel=.012, min_points=80):
        reference_world=voxel_reduce(reference_world,voxel)
        if len(reference_world)<min_points:raise ValueError('Insufficient reference scan points')
        # A fixed scan frame; not recomputed via PCA in each observed frame.
        self.origin=(reference_world.max(0)+reference_world.min(0))/2
        self.model=reference_world-self.origin
        self.tree=cKDTree(self.model)
        self.axes=principal_axes(self.model)
        self.min_points=min_points;self.voxel=voxel

    def _icp(self,source,r,t,iterations):
        r,t=r.copy(),t.copy()
        for _ in range(iterations):
            transformed=source@r.T+t
            distances,indices=self.tree.query(transformed,workers=1)
            threshold=min(.22,max(.015,float(np.quantile(distances,.85))))
            mask=distances<=threshold
            if mask.sum()<self.min_points:break
            dr,dt=rigid_fit(transformed[mask],self.model[indices[mask]])
            r,t=dr@r,dr@t+dt
            if np.linalg.norm(dt)<1e-6 and np.linalg.norm(dr-np.eye(3))<1e-5:break
        distances,indices=self.tree.query(source@r.T+t,workers=1)
        # Outlier-trimmed observation-to-scan fit; no unseen-surface labels.
        score=float(np.sqrt(np.mean(np.sort(distances)[:max(self.min_points,int(.85*len(distances)))]**2)))
        return score,r,t,distances

    def estimate(self,observed_world):
        observed=voxel_reduce(observed_world,self.voxel,1800)
        if len(observed)<self.min_points:
            return dict(valid=False,reason='insufficient_depth_points',points=len(observed))
        source=observed-self.origin
        source_axes=principal_axes(source)
        center=source.mean(0)
        candidates=[]
        coarse=source[np.linspace(0,len(source)-1,min(650,len(source)),dtype=int)]
        for axis in proper_axis_permutations():
            r=self.axes@axis@source_axes.T;t=self.model.mean(0)-r@center
            candidates.append(self._icp(coarse,r,t,14))
        candidates.sort(key=lambda v:v[0])
        refined=[self._icp(source,c[1],c[2],60) for c in candidates[:3]]
        refined.sort(key=lambda v:v[0]);score,r,t,distances=refined[0]
        inliers=distances<.025
        fraction=float(inliers.mean())
        if score>.018 or fraction<.65:
            return dict(valid=False,reason='poor_scan_alignment',points=len(source),
                        fit_rmse_m=score,inlier_fraction=fraction)
        rw=r.T;pw=self.origin-r.T@t
        # Avoid reporting a single confident pose when similarly good global
        # modes remain geometrically distinct. This is a heuristic rejection.
        for alternative in refined[1:]:
            alt_score,alt_r,alt_t,_=alternative
            angle=float(np.linalg.norm(cv2.Rodrigues(r@alt_r.T)[0]))
            shift=float(np.linalg.norm((-r.T@t)-(-alt_r.T@alt_t)))
            if alt_score-score<.0015 and (angle>np.radians(15) or shift>.04):
                return dict(valid=False,reason='ambiguous_alignment',points=len(source),
                            fit_rmse_m=score,alternative_fit_m=alt_score)
        return dict(valid=True,position_ned=pw.tolist(),rotation_scan_to_ned=rw.tolist(),
                    fit_rmse_m=score,inlier_fraction=fraction,points=len(source),
                    reference_points=len(self.model),covariance_available=False)
