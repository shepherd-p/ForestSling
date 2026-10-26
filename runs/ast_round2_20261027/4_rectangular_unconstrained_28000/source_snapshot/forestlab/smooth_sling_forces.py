"""Sparse whole-interval cable-force optimization with derivative bounds."""
from __future__ import annotations
import numpy as np
from .inertial_identification import wrench_matrix
from .tension_transport import wrench_map,radial_seed


def smooth_forces(plan,theta,anchors,times,initial_force=None,minimum_vertical=.55,
                  separation_reference=None,minimum_separation=.72):
    import clarabel
    from scipy import sparse as sp
    n=len(anchors);dof=3*n;k=len(times);nv=k*dof;dt=times[1]-times[0]
    matrices=[];rhs=[];cones=[];G=[];w=[];preferred=[];height_rows=[];height_rhs=[]
    for j,t in enumerate(times):
        target=plan.at(t);R=target.rotation;g=wrench_map(anchors,R)
        G.append(sp.csc_matrix(g));w.append(wrench_matrix(R,target.omega_world,target.acceleration,target.alpha_world)@theta)
        seed=plan.force_seed(t,R,anchors) if hasattr(plan,'force_seed') else radial_seed(anchors,R,[1.25,1.15])
        preferred.append(seed.ravel())
        span=target.maximum_endpoint_z-target.position[2]-(anchors@R.T)[:,2]
        outward=plan.outward(t,R,anchors) if hasattr(plan,'outward') else radial_seed(anchors,R,1.)
        outward[:,2]=0.
        outward/=np.maximum(np.linalg.norm(outward,axis=1)[:,None],1e-12)
        from .tension_passage import smooth
        minimum_outward=.85+.55*smooth(t/2.)
        for i in range(n):
            row=sp.lil_matrix((1,nv));row[0,j*dof+3*i:j*dof+3*i+3]=-outward[i]
            height_rows.append(row.tocsc());height_rhs.append(-minimum_outward)
        for i,h in enumerate(span):
            if h>=2.05:continue
            if h<.1:raise RuntimeError('Invalid smooth-force span budget')
            row=sp.lil_matrix((1,nv));v=-outward[i].copy();v[2]=np.sqrt((2.05/h)**2-1.)
            row[0,j*dof+3*i:j*dof+3*i+3]=v
            height_rows.append(row.tocsc())
            height_rhs.append(0.)
        if hasattr(target,'endpoint_halfspaces'):
            for normal,limit in target.endpoint_halfspaces:
                normal=np.asarray(normal);positions=target.position+anchors@R.T
                for i,anchor in enumerate(positions):
                    clearance=float(limit-normal@anchor)
                    if clearance<=.03:raise RuntimeError('Payload attachment lies outside planned opening')
                    # For any unit d, d.f <= ||f||. This conservative linear
                    # constraint therefore bounds the actual elastic endpoint.
                    normalization=getattr(target,'normalization_directions',np.tile([0.,0.,1.],(n,1)))[i]
                    coefficient=normal-clearance/2.05*np.asarray(normalization)
                    row=sp.lil_matrix((1,nv));row[0,j*dof+3*i:j*dof+3*i+3]=coefficient
                    height_rows.append(row.tocsc());height_rhs.append(0.)
        for i,coefficient,bound in getattr(target,'force_halfspaces',[]):
            row=sp.lil_matrix((1,nv));row[0,j*dof+3*i:j*dof+3*i+3]=coefficient
            height_rows.append(row.tocsc());height_rhs.append(bound)
        if separation_reference is not None:
            # Sequential convex endpoint separation: linearize the elastic
            # cable endpoint map at the preceding force solution, then check
            # the complete nonlinear trajectory before accepting a plan.
            f0=np.asarray(separation_reference[j]);ten=np.linalg.norm(f0,axis=1)
            unit=f0/ten[:,None]
            endpoints=target.position+anchors@R.T+(2./ten[:,None]+1./180.)*f0
            jac=np.array([2./a*(np.eye(3)-np.outer(u,u))+np.eye(3)/180. for a,u in zip(ten,unit)])
            for i in range(n):
                for other in range(i):
                    normal=endpoints[i]-endpoints[other];distance=np.linalg.norm(normal);normal/=distance
                    ci=-normal@jac[i];co=normal@jac[other]
                    row=sp.lil_matrix((1,nv));row[0,j*dof+3*i:j*dof+3*i+3]=ci
                    row[0,j*dof+3*other:j*dof+3*other+3]=co
                    height_rows.append(row.tocsc());height_rhs.append(distance-minimum_separation+ci@f0[i]+co@f0[other])
    matrices.append(sp.block_diag(G,format='csc'));rhs.append(np.concatenate(w));cones.append(clarabel.ZeroConeT(6*k))
    if initial_force is not None:
        g=G[0].toarray();pinv=np.linalg.pinv(g)
        f0=pinv@w[0]+(np.eye(dof)-pinv@g)@np.asarray(initial_force).ravel()
        eq=sp.hstack([sp.eye(dof),sp.csc_matrix((dof,nv-dof))],format='csc')
        matrices.append(eq);rhs.append(f0);cones.append(clarabel.ZeroConeT(dof))
    vertical=sp.coo_matrix((-np.ones(k*n),(np.arange(k*n),3*np.arange(k*n)+2)),shape=(k*n,nv)).tocsc()
    matrices.append(vertical);rhs.append(np.full(k*n,-minimum_vertical));cones.append(clarabel.NonnegativeConeT(k*n))
    if height_rows:
        matrices.append(sp.vstack(height_rows,format='csc'));rhs.append(np.asarray(height_rhs))
        cones.append(clarabel.NonnegativeConeT(len(height_rows)))
    for j in range(k*n):
        selector=sp.coo_matrix((-np.ones(3),(np.arange(3),3*j+np.arange(3))),shape=(3,nv)).tocsc()
        block=sp.vstack([sp.csc_matrix((1,nv)),selector],format='csc')
        matrices.extend([block,block]);rhs.extend([np.array([8.2,0.,0.,0.]),np.array([29.5,0.,0.,9.81])])
        cones.extend([clarabel.SecondOrderConeT(4),clarabel.SecondOrderConeT(4)])
    D1=sp.diags([-np.ones(k-1),np.ones(k-1)],[0,1],shape=(k-1,k),format='csc')/dt
    D2=sp.diags([np.ones(k-2),-2*np.ones(k-2),np.ones(k-2)],[0,1,2],shape=(k-2,k),format='csc')/dt**2
    D3=sp.diags([-np.ones(k-3),3*np.ones(k-3),-3*np.ones(k-3),np.ones(k-3)],[0,1,2,3],shape=(k-3,k),format='csc')/dt**3
    D1,D2,D3=[sp.kron(d,sp.eye(dof),format='csc') for d in (D1,D2,D3)]
    for operator,bound in ((D1,8.0),(D2,25.0)):
        matrices.extend([operator,-operator]);rhs.extend([np.full(operator.shape[0],bound)]*2)
        cones.extend([clarabel.NonnegativeConeT(operator.shape[0])]*2)
    edge=sp.vstack([D1[:dof],D1[-dof:]],format='csc')
    matrices.extend([edge,-edge]);rhs.extend([np.full(2*dof,.06)]*2)
    cones.extend([clarabel.NonnegativeConeT(2*dof)]*2)
    P=.02*sp.eye(nv,format='csc')+.04*(D1.T@D1)+.2*(D2.T@D2)+.04*(D3.T@D3)
    q=-.02*np.concatenate(preferred)
    settings=clarabel.DefaultSettings();settings.verbose=False
    settings.tol_feas=2e-8;settings.tol_gap_abs=1e-7;settings.tol_gap_rel=1e-7;settings.max_iter=100
    solution=clarabel.DefaultSolver(P.tocsc(),q,sp.vstack(matrices,format='csc'),np.concatenate(rhs),cones,settings).solve()
    if str(solution.status) not in ('Solved','AlmostSolved'):
        raise RuntimeError('Whole-interval force optimization: '+str(solution.status))
    forces=np.asarray(solution.x).reshape(k,n,3)
    return forces,dict(status=str(solution.status),variables=nv,iterations=solution.iterations,
        force_rate_bound_n_s=8.0,force_second_difference_bound_n_s2=25.,minimum_vertical_force_n=minimum_vertical,
        minimum_outward_prestress_n=[.85,1.4],
        linearized_endpoint_separation_m=minimum_separation if separation_reference is not None else None)
