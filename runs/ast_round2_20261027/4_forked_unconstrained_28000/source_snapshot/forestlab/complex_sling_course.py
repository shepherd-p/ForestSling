"""Known-map sequential openings, with shape and cable-force feasibility."""
from types import SimpleNamespace
import numpy as np
from .sixdof_sling import rpy
from .tension_passage import smooth,box_sdf,segment_box_hits,PassageRefusal
from .tension_transport import radial_seed,vee
from .inertial_identification import skew


def course_geometry(pose_mode='standard'):
    gates=[dict(name='offset_narrow',label='错位窄门',center=np.array([5.,.65,3.]),
                rotation=rpy(np.radians([0,0,15.])),width=2.1 if pose_mode=='large' else 2.7,height=6.),
           dict(name='low_roof',label='低顶区',center=np.array([13.,-.65,2.9]),
                rotation=np.eye(3),width=6.0,height=4.45),
           dict(name='tilted_opening',label='斜向开口',center=np.array([22.,.55,3.]),
                rotation=rpy(np.radians([12.,0,-25.])),width=2.85,height=6.)]
    boxes=[]
    for g in gates:
        c,R,w=g['center'],g['rotation'],g['width']
        if g['name']=='low_roof':
            boxes.append(dict(name='low_roof_panel',gate=g['name'],center=[13.,-.65,4.57],
                half=[1.05,3.0,.12],rotation=np.eye(3),material='portal_wood.png'))
            for sign in (-1,1):boxes.append(dict(name=f'low_roof_post_{sign}',gate=g['name'],
                center=[13.,-.65+sign*3.17,2.225],half=[.22,.17,2.225],rotation=np.eye(3),material='portal_guard_blue.png'))
        else:
            # A rotated portal frame, whose complete geometry is also passed
            # to MuJoCo. Its opening has no invisible collision proxy.
            for sign in (-1,1):
                local=np.array([0.,sign*(w/2+.18),0.])
                boxes.append(dict(name=f'{g["name"]}_post_{sign}',gate=g['name'],center=c+R@local,
                    half=[.22,.18,3.],rotation=R,material='portal_guard_blue.png'))
            boxes.append(dict(name=g['name']+'_lintel',gate=g['name'],center=c+R@np.array([0.,0.,3.10]),
                half=[.22,w/2+.36,.10],rotation=R,material='portal_wood.png'))
    # Staggered short blocks narrow the approach and exit away from the load
    # centerline; they are collision geometry, not decorative trees.
    for name,center,half in [
        ('approach_left',[2.2,-2.45,1.3],[.30,.45,1.3]),
        ('between_right',[9.,2.65,1.65],[.38,.48,1.65]),
        ('exit_left',[26.,-2.5,1.6],[.38,.45,1.6])]:
        boxes.append(dict(name=name,gate='clutter',center=center,half=half,rotation=np.eye(3),material='portal_wood.png'))
    return gates,boxes


def window(x,start,end,width=2.):
    return smooth((x-start)/width)*smooth((end-x)/width)


class ComplexCoursePlan:
    def __init__(self,p0,R0,duration=52.,drop=.45,pose_mode='standard'):
        if pose_mode not in ('standard','large'):raise ValueError('Unknown payload pose mode')
        self.p0=np.array(p0);self.R0=np.array(R0);self.duration=float(duration);self.distance=28.
        self.pose_mode=pose_mode
        self.drop=drop;self.peak=np.array([3.,3.]);self.formation_mode='multi_opening_constrained'
        self.gates,self.obstacles=course_geometry(pose_mode)
        a=np.arccos(np.clip((np.trace(R0)-1)/2,-1,1))
        self.initial=(np.zeros(3) if a<1e-8 else a/(2*np.sin(a))*np.array([R0[2,1]-R0[1,2],R0[0,2]-R0[2,0],R0[1,0]-R0[0,1]]))
        n=self.gates[2]['rotation'][:,1];m1=np.tan(np.radians(15.));m3=-n[0]/n[1]
        knots=np.array([0.,2.4,7.6,10.2,16.1,19.4,24.6,28.])
        ys=[0.,.65+m1*(2.4-5),.65+m1*(7.6-5),-.65,-.65,
            .55+m3*(19.4-22)-n[2]/n[1]*(self.p0[2]-3.),
            .55+m3*(24.6-22)-n[2]/n[1]*(self.p0[2]-3.),0.]
        derivatives=[0.,m1,m1,0.,0.,m3,m3,0.];self.path=[]
        for lo,hi,y0,y1,v0,v1 in zip(knots[:-1],knots[1:],ys[:-1],ys[1:],derivatives[:-1],derivatives[1:]):
            d=hi-lo;a1=d*v0;end=d*v1;delta=y1-y0
            self.path.append((lo,hi,np.array([y0,a1,0.,10*delta-6*a1-4*end,-15*delta+8*a1+7*end,6*delta-3*a1-3*end])))

    def values(self,t):
        s=smooth(t/self.duration);x=self.p0[0]+self.distance*s
        first=window(x,.1,9.4,2.2);last=window(x,17.0,28.,2.3)
        low=window(x,7.7,18.1,2.2)
        # Follow each portal's normal through its extruded neighborhood, so
        # skewed frames constrain the complete moving rig consistently.
        z=self.p0[2]-self.drop*low
        coordinate=np.clip(x-self.p0[0],0.,self.distance)
        for lo,hi,coefficients in self.path:
            if coordinate<=hi:
                u=np.clip((coordinate-lo)/(hi-lo),0.,1.);y=np.array([u**i for i in range(6)])@coefficients;break
        p=np.array([x,y+(1-s)*self.p0[1],z])
        angles=np.radians([90.*first-85.*last,0.,15.*first+90.*low-25.*last] if self.pose_mode=='large'
                          else [8.*low+18.*last,-5.*low+4.*last,15.*first-25.*last])
        v=(1-smooth(t/6.) if self.pose_mode=='large' else 1-s)*self.initial;K=skew(v);a=np.linalg.norm(v)
        Ri=np.eye(3)+(np.sin(a)/a if a>1e-8 else 1.)*K+((1-np.cos(a))/a**2 if a>1e-8 else .5)*K@K
        return p,rpy(angles)@Ri,first,low,last

    def pose(self,t):
        p,R,*_=self.values(t);return p,R,np.array([1.25,1.15])

    def outward(self,t,R,anchors):
        p,R,first,low,last=self.values(t)
        if self.pose_mode=='large':
            # Preserve each aircraft's horizontal sector when the attachment
            # plane becomes vertical. Projecting that plane alone collapses
            # upper/lower mounts onto the same endpoint direction.
            forward=R[:,0].copy();forward[2]=0.;forward/=np.linalg.norm(forward)
            side=np.cross([0,0,1.],forward)
            local=radial_seed(anchors,np.eye(3),1.);local[:,2]=0.
            direction=local[:,0,None]*forward+local[:,1,None]*side
            return direction/np.linalg.norm(direction,axis=1)[:,None]
        base=radial_seed(anchors,R,1.);base[:,2]=0.;base/=np.maximum(np.linalg.norm(base,axis=1)[:,None],1e-12)
        for weight,gate in ((first,self.gates[0]),(last,self.gates[2])):
            direction=gate['rotation'][:,0].copy();direction[2]=0.
            arms=(anchors-anchors.mean(0))@R.T
            tangent=np.array([-direction[1],direction[0],0.])
            along=(np.sign(arms@direction)[:,None]*direction
                   +.45*np.sign(arms@tangent)[:,None]*tangent)
            along/=np.maximum(np.linalg.norm(along,axis=1)[:,None],1e-12)
            base=(1-weight)*base+weight*along
        base[:,2]=0.;return base/np.maximum(np.linalg.norm(base,axis=1)[:,None],1e-12)

    def force_seed(self,t,R,anchors):
        return 1.7*self.outward(t,R,anchors)

    def at(self,t):
        h=.002;p,R,first,low,last=self.values(t)
        # Release the last opening once the complete rig is past the frame.
        # Pose recovery and aperture occupancy have different exit distances;
        # retaining the aperture through pose recovery crowds corner mounts.
        corner_mounts=hasattr(self,'current_anchors') and np.ptp(self.current_anchors[:,0])>1.0
        last_aperture=last*smooth((26.-p[0])/1.2) if corner_mounts else last
        pm,Rm,*_=self.values(t-h);pp,Rp,*_=self.values(t+h)
        def w(u):
            rr=self.values(u)[1]
            return vee((self.values(u+h)[1]-self.values(u-h)[1])/(2*h)@rr.T)
        target=SimpleNamespace(position=p,velocity=(pp-pm)/(2*h),acceleration=(pp-2*p+pm)/h**2,
            rotation=R,omega_world=vee((Rp-Rm)/(2*h)@R.T),alpha_world=(w(t+h)-w(t-h))/(2*h),
            pressure=np.array([1.25,1.15]),maximum_endpoint_z=(self.p0[2]+2.4)*(1-low)+4.10*low,
            endpoint_halfspaces=[],force_halfspaces=[])
        lateral=(np.cross([0,0,1.],R[:,0]) if self.pose_mode=='large' else R[:,1].copy())
        lateral[2]=0.;lateral/=np.linalg.norm(lateral)
        arms=(self.current_anchors-self.current_anchors.mean(0))@R.T if hasattr(self,'current_anchors') else None
        if arms is not None:
            lateral_floor=(.30*(1-max(first,last_aperture)**2) if corner_mounts else .30)
            # A vertical load needs a force couple across the upper/lower
            # mounts. Do not require every line to pull to its original side.
            if self.pose_mode=='large':
                pose_weight=max(first,last)
                lateral_floor=.30*(1-pose_weight**2)-1.2*pose_weight**3
            signs=np.sign(self.current_anchors[:,1]-self.current_anchors[:,1].mean()) if self.pose_mode=='large' else np.sign(arms@lateral)
            for i,sign in enumerate(signs):
                target.force_halfspaces.append((i,-sign*lateral,-lateral_floor*smooth(t/3.)))
            out=self.outward(t,R,self.current_anchors)
            target.normalization_directions=.6*out+np.array([0.,0.,.8])
        for weight,gate in ((first,self.gates[0]),(last_aperture,self.gates[2])):
            # Widen constraints smoothly outside the portal neighborhood. The
            # dense geometric check below still evaluates every physical box.
            if weight<.05:continue
            normal=gate['rotation'][:,1]
            allowance=gate['width']/2-.30+3.0*(1-weight)
            middle=float(normal@gate['center'])
            target.endpoint_halfspaces.extend([(normal,middle+allowance),(-normal,-middle+allowance)])
        return target


def plan_complex_course(p0,R0,theta,anchors,shape_points,obstacles,duration=52.,initial_force=None,pose_mode='standard'):
    from .smooth_sling_forces import smooth_forces
    from .sling_trajectory import EndpointTrajectory
    points=np.asarray(shape_points)
    # Use the complete reconstructed point cloud at dense validation times.
    # Chunking bounds memory without discarding thin protrusions.
    attempts=[]
    for factor in (1.,1.2,1.45,1.7):
        for drop in (.45,.65,.85):
            plan=ComplexCoursePlan(p0,R0,duration*factor,drop,pose_mode)
            plan.current_anchors=np.asarray(anchors).copy()
            diagnostic=dict(duration=plan.duration,drop=drop,feasible=False)
            attempts.append(diagnostic)
            try:
                times=np.linspace(0,plan.duration,121)
                f,solver=smooth_forces(plan,theta,anchors,times,initial_force)
                if pose_mode=='large':
                    for _ in range(3):f,solver=smooth_forces(plan,theta,anchors,times,initial_force,separation_reference=f)
                profile=EndpointTrajectory(plan,times,f,anchors)
            except RuntimeError as error:
                diagnostic['reason']=str(error);continue
            diagnostic.update(solver=solver,maximum_reference_speed=profile.maximum_reference_speed,
                maximum_reference_acceleration=profile.maximum_reference_acceleration,
                minimum_tension=profile.minimum_reference_tension,maximum_tension=profile.maximum_reference_tension,
                maximum_thrust=profile.maximum_thrust,maximum_tilt=profile.maximum_tilt)
            if not (profile.maximum_reference_speed<=2.5 and profile.maximum_reference_acceleration<=2.75
                and profile.minimum_reference_tension>1.0 and profile.maximum_reference_tension<8.25
                and profile.maximum_thrust<28. and profile.maximum_tilt<37.):
                diagnostic['reason']='reference_actuation_budget';continue
            gap=np.inf;rope_hit=False;separation=np.inf
            for ts in np.array_split(np.linspace(0,plan.duration,round(plan.duration/.02)+1),24):
                poses=[plan.pose(t) for t in ts];p=np.array([r[0] for r in poses]);R=np.array([r[1] for r in poses])
                aa=p[:,None,:]+np.einsum('nj,tkj->tnk',anchors,R);pu=profile.endpoint_curve(ts)
                load=p[:,None,:]+np.einsum('qj,tkj->tqk',points,R)
                separation=min(separation,min(np.linalg.norm(pu[:,i]-pu[:,j],axis=1).min() for i in range(len(anchors)) for j in range(i)))
                for box in obstacles:
                    # The 3.5 m center envelope contains every endpoint, rope
                    # and reconstructed point; distant boxes cannot collide.
                    nearby=box_sdf(p,box,3.5)<=0.
                    if not nearby.any():continue
                    gap=min(gap,float(box_sdf(pu[nearby],box,.28).min()),float(box_sdf(load[nearby],box,.045).min()))
                    rope_hit=rope_hit or bool(segment_box_hits(aa[nearby],pu[nearby],box,.035).any())
            diagnostic.update(minimum_inflated_geometry_gap_m=gap,rope_hit=rope_hit,minimum_aircraft_separation=separation)
            if gap>.015 and not rope_hit and separation>.52:
                diagnostic['feasible']=True;plan.profile=profile;return plan,attempts
            diagnostic['reason']='dense_system_geometry'
    raise PassageRefusal('No force/shape/actuation-feasible complex-course candidate',attempts)
