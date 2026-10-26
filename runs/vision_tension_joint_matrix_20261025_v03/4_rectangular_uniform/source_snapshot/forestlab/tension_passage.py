"""Low-dimensional load/formation/altitude passage feasibility optimization.

Geometry comes from reconstructed points and installed anchors. Wrench limits
use identified inertia. The map argument is explicit, allowing known-map
isolation tests before a local RGB-D obstacle map is connected.
"""
from __future__ import annotations
from types import SimpleNamespace
import numpy as np
from .inertial_identification import wrench_matrix,decode
from .tension_transport import wrench_map,radial_seed
from .sixdof_sling import rpy


class PassageRefusal(RuntimeError):
    def __init__(self,message,diagnostics):
        super().__init__(message);self.diagnostics=diagnostics

    def __reduce__(self):
        # ProcessPoolExecutor must preserve the refusal and its diagnostics.
        # RuntimeError's default pickle only carries the message argument.
        return type(self),(str(self),self.diagnostics)


def smooth(x):
    x=np.clip(x,0.,1.);return 126*x**5-420*x**6+540*x**7-315*x**8+70*x**9


def blend(progress):
    # The first/last aircraft reach the obstacle before/after the payload.
    return smooth(progress/.15)*smooth((1-progress)/.15)


def box_sdf(points,box,inflate=0.):
    q=np.abs(np.asarray(points)-np.asarray(box['center']))-np.asarray(box['half'])-inflate
    return np.linalg.norm(np.maximum(q,0.),axis=-1)+np.minimum(np.max(q,axis=-1),0.)


def segment_box_hits(start,end,box,radius=.006):
    start,end=np.asarray(start),np.asarray(end)
    center=np.asarray(box['center']);half=np.asarray(box['half'])+radius
    d=end-start
    parallel=np.abs(d)<1e-12
    outside=(parallel&(np.abs(start-center)>half)).any(axis=-1)
    denominator=np.where(parallel,1.,d)
    lower=(center-half-start)/denominator;upper=(center+half-start)/denominator
    lo=np.where(parallel,-np.inf,np.minimum(lower,upper)).max(axis=-1)
    hi=np.where(parallel,np.inf,np.maximum(lower,upper)).min(axis=-1)
    return (~outside)&(np.maximum(lo,0.)<=np.minimum(hi,1.))


class PassagePlan:
    def __init__(self,p0,R0,duration=14.,distance=8.,peak=(3.5,3.5),drop=.1,angles=(25.,10.,30.),formation_mode='preload'):
        self.p0=np.asarray(p0);self.R0=np.asarray(R0);self.duration=duration;self.distance=distance
        self.peak=np.asarray(peak);self.drop=drop;self.angles=np.asarray(angles)
        self.formation_mode=formation_mode

    def pose(self,t):
        s=smooth(t/self.duration);b=blend(s)
        p=self.p0+np.array([self.distance*s,0.,-self.drop*b])
        # Small initial fixture tilt is blended with a continuous Euler goal.
        # Rotation rates below are obtained from this exact pose function.
        from .sixdof_sling import rotation
        from .inertial_identification import skew
        a=np.arccos(np.clip((np.trace(self.R0)-1)/2,-1,1))
        if a<1e-8: initial=np.zeros(3)
        else:
            initial=a/(2*np.sin(a))*np.array([self.R0[2,1]-self.R0[1,2],self.R0[0,2]-self.R0[2,0],self.R0[1,0]-self.R0[0,1]])
        v=(1-s)*initial;K=skew(v);a=np.linalg.norm(v)
        initial_R=np.eye(3)+(np.sin(a)/a if a>1e-8 else 1)*K+((1-np.cos(a))/a**2 if a>1e-8 else .5)*K@K
        attitude_progress=(np.full(3,s) if self.formation_mode=='height_cone' else
            np.array([smooth((s-.86)/.14),smooth((s-.86)/.14),s]))
        R=rpy(np.radians(self.angles)*attitude_progress)@initial_R
        pressure=np.array([1.25,1.15])+b*(self.peak-[1.25,1.15])
        return p,R,pressure

    def at(self,t):
        from .tension_transport import vee
        h=.002;p,R,pressure=self.pose(t)
        pm,Rm,_=self.pose(t-h);pp,Rp,_=self.pose(t+h)
        omega=vee((Rp-Rm)/(2*h)@R.T)
        def w(u):
            r=self.pose(u)[1]
            return vee((self.pose(u+h)[1]-self.pose(u-h)[1])/(2*h)@r.T)
        result=SimpleNamespace(position=p,velocity=(pp-pm)/(2*h),acceleration=(pp-2*p+pm)/h**2,
            rotation=R,omega_world=omega,alpha_world=(w(t+h)-w(t-h))/(2*h),pressure=pressure)
        if self.formation_mode=='height_cone':
            b=blend(smooth(t/self.duration))
            result.maximum_endpoint_z=(self.p0[2]+2.3)*(1-b)+4.20*b
        return result


def plan_beam_conic(p0,R0,theta,anchors,shape_points,obstacles,duration=16.,initial_force=None):
    """Per-cable inclination budget, with inertia-dependent force balance."""
    from .tension_transport import WrenchAllocator
    diagnostics=[];points=np.asarray(shape_points)
    if len(points)>180:points=points[np.linspace(0,len(points)-1,180,dtype=int)]
    for drop in (0.,.2,.4,.6,.7):
        plan=PassagePlan(p0,R0,duration=duration,peak=(1.25,1.15),drop=drop,formation_mode='height_cone')
        allocator=WrenchAllocator(anchors,uav_mass=1.,pressure=[1.25,1.15],maximum_tension=8.2)
        if initial_force is not None:allocator.previous=np.asarray(initial_force).copy()
        gap=np.inf;min_t=np.inf;max_t=0.;max_residual=0.;positions=[];refused=None;forces=[]
        times=np.linspace(0.,duration,81)
        for t in times:
            target=plan.at(t);R=target.rotation
            span=target.maximum_endpoint_z-target.position[2]-(anchors@R.T)[:,2]
            allocator.vertical_span_limit=span
            # Outward prestress maintains extension without requiring an
            # impossible redistribution of vertical support on three anchors.
            allocator.minimum_outward=.85+.55*smooth(t/2.)
            wrench=wrench_matrix(R,target.omega_world,target.acceleration,target.alpha_world)@theta
            try:f,info=allocator.allocate(R,wrench)
            except RuntimeError as error:refused=str(error);break
            ten=np.linalg.norm(f,axis=1);a=target.position+anchors@R.T
            pu=a+(2.+ten[:,None]/180.)*f/ten[:,None];positions.append(pu)
            forces.append(f)
            min_t=min(min_t,float(ten.min()));max_t=max(max_t,float(ten.max()));max_residual=max(max_residual,info['residual'])
            ropes=a[:,None,:]+np.linspace(0,1,41)[None,:,None]*(pu-a)[:,None,:]
            load=target.position+points@R.T
            for box in obstacles:
                gap=min(gap,float(box_sdf(pu,box,.27).min()),float(box_sdf(ropes,box,.04).min()),float(box_sdf(load,box,.04).min()))
        diagnostic=dict(drop=drop,mode='height_cone',feasible=bool(refused is None and gap>.01 and min_t>.5 and max_t<9.),
            gap=float(gap) if np.isfinite(gap) else None,min_tension=float(min_t) if np.isfinite(min_t) else None,
            max_tension=max_t,max_residual=max_residual,refusal=refused)
        diagnostics.append(diagnostic)
        if diagnostic['feasible']:
            from .sling_trajectory import EndpointTrajectory
            from .smooth_sling_forces import smooth_forces
            try:
                forces,solver=smooth_forces(plan,theta,anchors,times,initial_force)
                diagnostic['smooth_force_solver']=solver
            except RuntimeError as error:
                diagnostic.update(feasible=False,smoothing_refusal=str(error));continue
            profile=EndpointTrajectory(plan,times,forces,anchors)
            diagnostic.update(maximum_reference_speed_m_s=profile.maximum_reference_speed,
                maximum_reference_acceleration_m_s2=profile.maximum_reference_acceleration,
                dense_minimum_tension=profile.minimum_reference_tension,dense_maximum_tension=profile.maximum_reference_tension,
                maximum_predicted_thrust_n=profile.maximum_thrust,maximum_predicted_tilt_deg=profile.maximum_tilt)
            # Half of the 4.5 m/s^2 command budget remains for feedback and
            # motor/attitude tracking during formation transients.
            diagnostic['feasible']=bool(profile.maximum_reference_speed<=2.5 and profile.maximum_reference_acceleration<=2.25
                and profile.minimum_reference_tension>1.0 and profile.maximum_reference_tension<8.25
                and profile.maximum_thrust<28. and profile.maximum_tilt<37.)
            if diagnostic['feasible']:
                dense_times=np.linspace(0.,duration,round(duration/.02)+1)
                poses=[plan.pose(t) for t in dense_times]
                positions=np.asarray([p[0] for p in poses]);rotations=np.asarray([p[1] for p in poses])
                aa=positions[:,None,:]+np.einsum('nj,tkj->tnk',anchors,rotations)
                pp=profile.endpoint_curve(dense_times)
                loads=positions[:,None,:]+np.einsum('qj,tkj->tqk',points,rotations)
                dense_gap=np.inf;rope_hit=False
                for box in obstacles:
                    dense_gap=min(dense_gap,float(box_sdf(pp,box,.27).min()),float(box_sdf(loads,box,.04).min()))
                    rope_hit=rope_hit or bool(segment_box_hits(aa,pp,box,.035).any())
                diagnostic.update(dense_body_gap=float(dense_gap),dense_rope_hit=rope_hit)
                diagnostic['feasible']=bool(dense_gap>.005 and not rope_hit)
            if diagnostic['feasible']:
                plan.profile=profile
                return plan,diagnostics
    raise PassageRefusal('Height-constrained force allocation found no feasible passage candidate',diagnostics)


def plan_beam_joint(p0,R0,theta,anchors,shape_points,obstacles,duration=20.,initial_force=None):
    attempts=[]
    for factor in (1.,1.1,1.25,1.5):
        try:
            plan,diagnostic=plan_beam_conic(p0,R0,theta,anchors,shape_points,obstacles,duration*factor,initial_force)
            attempts.append(dict(duration=duration*factor,accepted=True,candidates=diagnostic))
            return plan,attempts
        except PassageRefusal as error:
            attempts.append(dict(duration=duration*factor,accepted=False,candidates=error.diagnostics))
    raise PassageRefusal('No force/geometry/actuation-feasible timing candidate',attempts)


def evaluate(plan,theta,anchors,shape_points,obstacles,rest_length=2.,stiffness=180.,nodes=61):
    shape_points=np.asarray(shape_points)
    if len(shape_points)>180:shape_points=shape_points[np.linspace(0,len(shape_points)-1,180,dtype=int)]
    minimum_gap=np.inf;max_tension=0.;min_tension=np.inf;min_sep=np.inf;maximum_height=0.
    for t in np.linspace(0,plan.duration,nodes):
        target=plan.at(t);R=target.rotation
        G=wrench_map(anchors,R);pinv=np.linalg.pinv(G)
        wrench=wrench_matrix(R,target.omega_world,target.acceleration,target.alpha_world)@theta
        f=(pinv@wrench+(np.eye(G.shape[1])-pinv@G)@radial_seed(anchors,R,target.pressure).ravel()).reshape(-1,3)
        ten=np.linalg.norm(f,axis=1)
        if f[:,2].min()<.55:return dict(feasible=False,gap=-1.,max_tension=float(ten.max()),reason='vertical_force')
        attach=target.position+anchors@R.T
        pu=attach+(rest_length+ten[:,None]/stiffness)*f/ten[:,None]
        max_tension=max(max_tension,float(ten.max()));min_tension=min(min_tension,float(ten.min()))
        maximum_height=max(maximum_height,float(pu[:,2].max()-target.position[2]))
        min_sep=min(min_sep,min(float(np.linalg.norm(pu[i]-pu[j])) for i in range(len(pu)) for j in range(i)))
        ropes=attach[:,None,:]+np.linspace(0,1,41)[None,:,None]*(pu-attach)[:,None,:]
        load=target.position+shape_points@R.T
        for box in obstacles:
            # Inflated sampling covers the maximum inter-node movement and
            # rope sample spacing in this development speed/length range.
            minimum_gap=min(minimum_gap,float(box_sdf(load,box,.15).min()),
                float(box_sdf(pu,box,.23+.15).min()),float(box_sdf(ropes,box,.04+.15).min()))
    return dict(feasible=bool(minimum_gap>.01 and min_tension>.55 and max_tension<8.8 and min_sep>.46),
        gap=float(minimum_gap),max_tension=max_tension,min_tension=min_tension,min_separation=min_sep,
        max_relative_aircraft_height_m=maximum_height)


def plan_beam(p0,R0,theta,anchors,shape_points,obstacles,duration=14.):
    # Deterministic finite candidate optimization; no global-optimality claim.
    # Full online receding-horizon map integration is a separate execution step.
    grid=[(drop,pressure) for drop in (0.,.2,.4,.6,.7) for pressure in (2.,2.5,3.,3.5,4.,4.5)]
    drops=np.asarray([d for d,p in grid]);peaks=np.asarray([[p,.92*p] for d,p in grid])
    zero=PassagePlan(p0,R0,duration=duration,drop=0.)
    unit=PassagePlan(p0,R0,duration=duration,drop=1.)
    positions=[];rotations=[];particular=[];drop_force=[];nx=[];ny=[];envelope=[]
    for t in np.linspace(0.,duration,61):
        a=zero.at(t);b=unit.at(t);R=a.rotation
        G=wrench_map(anchors,R);pinv=np.linalg.pinv(G);P=np.eye(G.shape[1])-pinv@G
        w0=wrench_matrix(R,a.omega_world,a.acceleration,a.alpha_world)@theta
        w1=wrench_matrix(R,b.omega_world,b.acceleration,b.alpha_world)@theta
        positions.append(a.position);rotations.append(R);particular.append(pinv@w0);drop_force.append(pinv@(w1-w0))
        nx.append(P@radial_seed(anchors,R,[1.,0.]).ravel());ny.append(P@radial_seed(anchors,R,[0.,1.]).ravel())
        envelope.append(blend(smooth(t/duration)))
    positions,rotations,particular,drop_force,nx,ny,envelope=map(np.asarray,
        (positions,rotations,particular,drop_force,nx,ny,envelope))
    pressure=np.array([1.25,1.15])[None,None,:]+envelope[None,:,None]*(peaks[:,None,:]-[1.25,1.15])
    n=len(anchors);c=len(grid)
    f=(particular[None]+drops[:,None,None]*drop_force[None]+pressure[:,:,0,None]*nx[None]
        +pressure[:,:,1,None]*ny[None]).reshape(c,61,n,3)
    ten=np.linalg.norm(f,axis=-1)
    center=np.tile(positions[None],(c,1,1));center[:,:,2]-=drops[:,None]*envelope[None]
    arms=np.einsum('nj,tkj->tnk',anchors,rotations)
    attach=center[:,:,None,:]+arms[None]
    endpoints=attach+(2.+ten[:,:,:,None]/180.)*f/ten[:,:,:,None]
    points=np.asarray(shape_points)
    if len(points)>180:points=points[np.linspace(0,len(points)-1,180,dtype=int)]
    loads=center[:,:,None,:]+np.einsum('qj,tkj->tqk',points,rotations)[None]
    ropes=attach[:,:,:,None,:]+np.linspace(0,1,41)[None,None,None,:,None]*(endpoints-attach)[:,:,:,None,:]
    gap=np.full(c,np.inf)
    for box in obstacles:
        gap=np.minimum(gap,box_sdf(loads,box,.15).min(axis=(1,2)))
        gap=np.minimum(gap,box_sdf(endpoints,box,.38).min(axis=(1,2)))
        gap=np.minimum(gap,box_sdf(ropes,box,.19).min(axis=(1,2,3)))
    sep=np.stack([np.linalg.norm(endpoints[:,:,i]-endpoints[:,:,j],axis=-1)
                  for i in range(n) for j in range(i)],axis=-1).min(axis=(1,2))
    min_t,max_t=ten.min(axis=(1,2)),ten.max(axis=(1,2));min_z=f[:,:,:,2].min(axis=(1,2))
    height=(endpoints[:,:,:,2]-center[:,:,None,2]).max(axis=(1,2))
    candidates=[]
    for i,(drop,p) in enumerate(grid):
        diag=dict(feasible=bool(gap[i]>.01 and min_t[i]>.55 and max_t[i]<8.8 and sep[i]>.46 and min_z[i]>=.55),
            gap=float(gap[i]),max_tension=float(max_t[i]),min_tension=float(min_t[i]),
            min_separation=float(sep[i]),minimum_vertical_force=float(min_z[i]),max_relative_aircraft_height_m=float(height[i]))
        candidates.append(dict(pressure=p,drop=drop,diagnostic=diag))
    feasible=[c for c in candidates if c['diagnostic']['feasible']]
    if not feasible:raise RuntimeError('No geometrically/dynamically feasible beam candidate')
    best=min(feasible,key=lambda c:c['pressure']**2+3*c['drop'])
    return PassagePlan(p0,R0,duration=duration,peak=(best['pressure'],.92*best['pressure']),drop=best['drop']),candidates
