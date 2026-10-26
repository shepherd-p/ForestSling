"""Smooth cable-force/endpoint trajectory and motor-force tracking interface."""
from __future__ import annotations
import numpy as np
from .tension_transport import vee


class EndpointTrajectory:
    def __init__(self,plan,times,forces,anchors,stiffness=180.,damping=5.,rest_length=2.):
        from scipy.interpolate import CubicSpline
        self.plan=plan;self.anchors=np.asarray(anchors);self.duration=plan.duration
        self.force_curve=CubicSpline(times,np.asarray(forces),axis=0,bc_type='clamped')
        self.times=np.linspace(0.,plan.duration,round(plan.duration/.005)+1)
        f=self.force_curve(self.times);ten=np.linalg.norm(f,axis=-1)
        extension=np.empty_like(ten);extension[0]=ten[0]/stiffness
        keep=np.exp(-np.diff(self.times)*stiffness/damping)
        for j,k in enumerate(keep):
            extension[j+1]=k*extension[j]+(1-k)*.5*(ten[j]+ten[j+1])/stiffness
        p=[]
        for j,t in enumerate(self.times):
            position,R,_=plan.pose(t)
            p.append(position+self.anchors@R.T+(rest_length+extension[j,:,None])*f[j]/ten[j,:,None])
        self.endpoint_curve=CubicSpline(self.times,np.asarray(p),axis=0,bc_type='clamped')
        self.maximum_reference_speed=float(np.linalg.norm(self.endpoint_curve(self.times,1),axis=-1).max())
        self.maximum_reference_acceleration=float(np.linalg.norm(self.endpoint_curve(self.times,2),axis=-1).max())
        self.minimum_reference_tension=float(ten.min());self.maximum_reference_tension=float(ten.max())
        thrust=self.endpoint_curve(self.times,2)+[0.,0.,9.81]+f+.2*self.endpoint_curve(self.times,1)
        self.maximum_thrust=float(np.linalg.norm(thrust,axis=-1).max())
        self.maximum_tilt=float(np.degrees(np.arctan2(np.linalg.norm(thrust[:,:,:2],axis=-1),thrust[:,:,2])).max())
        self.pose_correction=np.zeros((len(anchors),3))
        # Wide mounts give greater rotational leverage. Keep the slow image
        # feedback below the support/load swing mode; endpoint feedback still
        # uses 50 Hz aircraft telemetry and the planned load-force feedforward.
        self.payload_pose_gain=.02 if np.ptp(self.anchors[:,0])>1.0 else .2

    def command(self,row,uav_velocity,t,dt=.02,uav_mass=1.):
        u=np.clip(t,0.,self.duration);target=self.plan.at(u)
        p=self.endpoint_curve(u);v=self.endpoint_curve(u,1);a=self.endpoint_curve(u,2)
        if t>=self.duration:v=np.zeros_like(v);a=np.zeros_like(a)
        R=np.asarray(row['rotation'])
        e=.5*vee(target.rotation@R.T-R@target.rotation.T)
        arms=self.anchors@target.rotation.T
        correction=self.payload_pose_gain*((target.position-np.asarray(row['position']))+np.cross(e,arms))
        self.pose_correction+=(1-np.exp(-dt/.2))*(correction-self.pose_correction)
        pu=np.asarray(row['uav_position']);vu=np.asarray(uav_velocity)
        acceleration=a+18.*(p+self.pose_correction-pu)+7.*(v-vu)
        acceleration*=np.minimum(1.,4.5/np.maximum(np.linalg.norm(acceleration,axis=1),1e-12))[:,None]
        force=uav_mass*(acceleration+[0.,0.,9.81])+self.force_curve(u)+.2*vu
        return force,dict(uav_target_neu=p,reference_acceleration=a,reference_velocity=v,
            desired_load_cable_forces_neu_n=self.force_curve(u))
