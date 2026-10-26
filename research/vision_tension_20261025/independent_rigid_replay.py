"""Separate NumPy RK4 rigid-body/motor implementation; no MuJoCo import.

Evaluator inertia enters this post-execution numerical check only. Saved motor
commands are replayed, with no pose resets after the initial condition.
"""
from pathlib import Path
import argparse
import json
import time
import numpy as np


def rotations(q):
    q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
        2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
        2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)


class RigidReplay:
    def __init__(self,asset):
        p=asset['public'];h=asset['evaluator'];self.n=len(p['anchors']);self.b=self.n+1
        self.com=np.asarray(h['com']);self.anchors=np.asarray(p['anchors'])
        self.mass=np.r_[h['mass'],np.ones(self.n)]
        self.inertia=np.tile(np.diag([.009,.009,.016]),(self.b,1,1));self.inertia[0]=h['inertia']
        self.inv_inertia=np.linalg.inv(self.inertia)
        self.mixer=np.array([[1.,1.,1.,1.],[0.,.16,0.,-.16],[-.16,0.,.16,0.],[.018,-.018,.018,-.018]])

    def unpack(self,x):
        a=self.b*3;b=a+self.b*3;c=b+self.b*4;d=c+self.b*3
        return x[:a].reshape(self.b,3),x[a:b].reshape(self.b,3),x[b:c].reshape(self.b,4),x[c:d].reshape(self.b,3),x[d:].reshape(self.n,4)

    def from_native(self,row):
        b=self.b;qp=np.asarray(row[1:1+7*b]).reshape(b,7);qv=np.asarray(row[1+7*b:1+13*b]).reshape(b,6)
        R=rotations(qp[:,3:]);omega=np.einsum('bij,bj->bi',R,qv[:,3:])
        p=qp[:,:3].copy();v=qv[:,:3].copy();arm=R[0]@self.com
        p[0]+=arm;v[0]+=np.cross(omega[0],arm)
        return np.r_[p.ravel(),v.ravel(),qp[:,3:].ravel(),omega.ravel(),row[1+13*b:1+13*b+4*self.n]]

    def rhs(self,x,command):
        p,v,q,w,motors=self.unpack(x);R=rotations(q)
        load_arms=(self.anchors-self.com)@R[0].T
        uav_arms=np.einsum('bij,j->bi',R[1:],[0.,0.,-.015])
        attachments=p[0]+load_arms;endpoints=p[1:]+uav_arms
        delta=endpoints-attachments;length=np.linalg.norm(delta,axis=1);direction=delta/length[:,None]
        rate=np.sum(direction*(v[1:]+np.cross(w[1:],uav_arms)-v[0]-np.cross(w[0],load_arms)),axis=1)
        extension=length-2.;tension=np.where(extension>0.,np.maximum(0.,180.*extension+5.*rate),0.)
        f=direction*tension[:,None]
        force=np.zeros((self.b,3));torque=np.zeros_like(force)
        force[0]=f.sum(axis=0);torque[0]=np.cross(load_arms,f).sum(axis=0)
        thrust=motors.sum(axis=1)[:,None]*R[1:,:,2]
        force[1:]=thrust-f-.2*v[1:]
        body_torque=motors@self.mixer[1:].T
        torque[1:]=np.einsum('bij,bj->bi',R[1:],body_torque)+np.cross(uav_arms,-f)
        acceleration=force/self.mass[:,None]+[0.,0.,-9.81]
        iw=np.einsum('bij,bjk,bkl->bil',R,self.inertia,R.transpose(0,2,1))
        inv=np.einsum('bij,bjk,bkl->bil',R,self.inv_inertia,R.transpose(0,2,1))
        angular=np.einsum('bij,bj->bi',inv,torque-np.cross(w,np.einsum('bij,bj->bi',iw,w)))
        qdot=.5*np.c_[-np.sum(w*q[:,1:],axis=1),q[:,0,None]*w+np.cross(w,q[:,1:])]
        return np.r_[v.ravel(),acceleration.ravel(),qdot.ravel(),angular.ravel(),((command-motors)/.025).ravel()]

    def step(self,x,command,dt):
        k1=self.rhs(x,command);k2=self.rhs(x+.5*dt*k1,command)
        k3=self.rhs(x+.5*dt*k2,command);k4=self.rhs(x+dt*k3,command)
        x=x+dt/6.*(k1+2*k2+2*k3+k4)
        p,v,q,w,motors=self.unpack(x);q/=np.linalg.norm(q,axis=1)[:,None]
        return x


def run(source,out,step=.001,seconds=None):
    out.mkdir(parents=True,exist_ok=False)
    asset=json.loads((source/'ASSET.json').read_text());plant=RigidReplay(asset)
    raw=np.load(source/'trajectory.npz');states=raw['states'];commands=raw['commands']
    if seconds is not None:commands=commands[:round(seconds/.02)];states=states[:len(commands)+1]
    x=plant.from_native(states[0]);records=[];tick=time.perf_counter()
    for j,command in enumerate(commands):
        for _ in range(round(.02/step)):x=plant.step(x,command,step)
        ref=plant.from_native(states[j+1]);p,v,q,w,m=plant.unpack(x);pr,vr,qr,wr,mr=plant.unpack(ref)
        R=rotations(q);Rr=rotations(qr)
        origin=p[0]-R[0]@plant.com;origin_ref=pr[0]-Rr[0]@plant.com
        angle=np.degrees(2*np.arccos(np.clip(abs(np.sum(q[0]*qr[0])),0.,1.)))
        endpoints=p[1:]+np.einsum('bij,j->bi',R[1:],[0.,0.,-.015])
        endpoints_ref=pr[1:]+np.einsum('bij,j->bi',Rr[1:],[0.,0.,-.015])
        records.append([states[j+1,0],np.linalg.norm(origin-origin_ref),angle,
            np.linalg.norm(endpoints-endpoints_ref,axis=1).max(),np.abs(m-mr).max()])
    records=np.asarray(records)
    summary=dict(source=str(source),integrator='Independent NumPy rigid-body RK4, world angular velocity, motor ODE',
        whole_interval_without_resets=True,replayed_seconds=float(records[-1,0]),step_sec=step,
        maximum_payload_origin_difference_m=float(records[:,1].max()),maximum_payload_angle_difference_deg=float(records[:,2].max()),
        maximum_aircraft_endpoint_difference_m=float(records[:,3].max()),maximum_motor_force_difference_n=float(records[:,4].max()),
        wall_sec=time.perf_counter()-tick,scope='Numerical implementation cross-check; not independent object/map statistical validation')
    np.savez_compressed(out/'differences.npz',differences=records)
    (out/'SUMMARY.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(summary,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--out',required=True)
    p.add_argument('--step',type=float,default=.001);p.add_argument('--seconds',type=float)
    a=p.parse_args();run(Path(a.source).resolve(),Path(a.out).resolve(),a.step,a.seconds)
