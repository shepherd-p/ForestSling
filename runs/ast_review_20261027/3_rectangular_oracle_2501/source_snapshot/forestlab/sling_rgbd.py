"""Calibrated virtual RGB-D reconstruction and causal point-cloud tracking.

Only depth pixels, camera intrinsics/extrinsics and a predicted search region
enter reconstruction. Object IDs, true meshes and true body poses are not used.
The initial scan is acquired in a known installation fixture before motion.
"""
from __future__ import annotations
import numpy as np
from .sixdof_sling import mujoco
from .markerless import voxel_reduce, ScanRegistration
from .markerless_tracking import TemporalScanTracker


class SlingRGBD:
    def __init__(self, model, width=256, height=192, seed=1,
                 tracking_stride=2, tracking_correspondence_gate=.08,
                 appearance_filter=False):
        self.model=model; self.width=width; self.height=height
        self.renderer=mujoco.Renderer(model,height,width)
        self.scene_option=mujoco.MjvOption()
        self.rng=np.random.default_rng(seed)
        self.last_rgb=None
        self.tracking_stride=tracking_stride
        self.tracking_correspondence_gate=tracking_correspondence_gate
        self.appearance_filter=appearance_filter
        self.appearance=None

    def close(self):
        self.renderer.close()

    def capture(self,data,center,azimuth=60.,elevation=-20.,distance=2.0,half=(.85,.85,.55),stride=2):
        camera=mujoco.MjvCamera();camera.lookat[:]=center
        camera.distance=distance;camera.azimuth=azimuth;camera.elevation=elevation
        self.renderer.update_scene(data,camera=camera,scene_option=self.scene_option)
        self.renderer.disable_depth_rendering();self.last_rgb=self.renderer.render().copy()
        cams=self.renderer.scene.camera
        position=.5*(np.array(cams[0].pos)+np.array(cams[1].pos))
        forward=np.array(cams[0].forward);up=np.array(cams[0].up)
        right=np.cross(forward,up)
        fy=self.height*cams[0].frustum_near/(cams[0].frustum_top-cams[0].frustum_bottom)
        self.renderer.enable_depth_rendering();depth=self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        yy,xx=np.mgrid[0:self.height:stride,0:self.width:stride]
        d=depth[::stride,::stride].astype(float)
        d+=self.rng.normal(0.,.001,d.shape)
        points=(position+d[...,None]*(forward+((xx+.5-self.width/2)/fy)[...,None]*right
                -((yy+.5-self.height/2)/fy)[...,None]*up)).reshape(-1,3)
        keep=np.isfinite(points).all(axis=1)&(np.abs(points-np.asarray(center))<np.asarray(half)).all(axis=1)
        self.last_point_colors=self.last_rgb[::stride,::stride].reshape(-1,3)[keep].astype(float)
        return points[keep]

    @staticmethod
    def chromaticity(colors):
        return colors/np.maximum(colors.sum(axis=1,keepdims=True),1.)

    def scan(self,data,fixture_origin=(0.,0.,3.)):
        clouds=[];images=[];colors=[]
        # Pre-rigging fixture scan: cables have not been attached in this
        # acquisition phase. Later tracking images include the fitted cables.
        self.scene_option.tendongroup[:]=0
        for azimuth in (15.,75.,135.,195.,255.,315.):
            for elevation in (-30.,20.):
                clouds.append(self.capture(data,fixture_origin,azimuth,elevation,stride=2))
                colors.append(self.last_point_colors.copy())
                images.append(self.last_rgb.copy())
        self.scene_option.tendongroup[:]=1
        if self.appearance_filter:
            # Learn material appearance from the known empty installation
            # fixture's RGB-D crop. No semantic labels or renderer IDs enter.
            chroma=self.chromaticity(np.vstack(colors))
            center=np.median(chroma,axis=0)
            radius=max(.06,float(np.quantile(np.linalg.norm(chroma-center,axis=1),.99))+.025)
            self.appearance=dict(chromaticity_center=center,chromaticity_radius=radius)
        points=np.vstack(clouds)
        if self.appearance is not None:
            distance=np.linalg.norm(self.chromaticity(np.vstack(colors))-self.appearance['chromaticity_center'],axis=1)
            points=points[distance<self.appearance['chromaticity_radius']]
        cloud=voxel_reduce(points,.01,6000)
        from scipy.spatial import cKDTree
        # Thin cables in the search volume are not rigid payload surfaces.
        # Local 3D surface support removes these without an object-ID mask.
        density=cKDTree(cloud).query_ball_point(cloud,.035,return_length=True)
        if not self.appearance_filter:cloud=cloud[np.asarray(density)>=12]
        # In the empty pre-rigging fixture, learned RGB appearance already
        # separates the payload. The cable line filter would erase its thin
        # prongs and cavity boundaries, leaving an incomplete rigid model.
        for _ in range(0 if self.appearance_filter else 2):
            # A cable can have several occupied voxels across its diameter.
            # Reject line-like neighborhoods; retain planar/curved surfaces.
            _,neighbors=cKDTree(cloud).query(cloud,k=min(20,len(cloud)))
            local=cloud[neighbors];local-=local.mean(axis=1,keepdims=True)
            covariance=np.einsum('nki,nkj->nij',local,local)/local.shape[1]
            eigenvalues=np.linalg.eigvalsh(covariance)
            cloud=cloud[eigenvalues[:,1]>.18*np.maximum(eigenvalues[:,2],1e-12)]
        tracker=TemporalScanTracker(cloud)
        # Known static fixture fixes the initial orientation, including the
        # cuboid's geometric symmetry. Subsequent registration is local/causal.
        tracker.last=(float(data.time),tracker.registration.origin.copy(),np.eye(3))
        return cloud,tracker,images

    def track(self,data,tracker,predicted_center,timestamp):
        clouds=[]
        for azimuth in (45.,225.):
            points=self.capture(data,predicted_center,azimuth,-20.,stride=self.tracking_stride)
            if self.appearance is not None:
                distance=np.linalg.norm(self.chromaticity(self.last_point_colors)-self.appearance['chromaticity_center'],axis=1)
                points=points[distance<self.appearance['chromaticity_radius']]
            clouds.append(points)
        points=np.vstack(clouds)
        if tracker.last is not None:
            pp,rr,age=tracker._prediction(timestamp)
            distance,_=tracker.registration.tree.query((points-pp)@rr,workers=1)
            # Partial scans contain newly revealed cavity surfaces and cables.
            # Match only surfaces supported by the acquired reference scan.
            points=points[distance<self.tracking_correspondence_gate]
        return tracker.step(points,timestamp)
