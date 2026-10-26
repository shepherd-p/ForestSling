"""Team-size independent cable wrench servo driven by measured motion/tension.

No plant, true mass, true CoM, or evaluator state is accepted by this module.
The linear inertial regressor uses a fixed visible material origin throughout.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from .inertial_identification import decode, skew, validate, validate_anchors, wrench_matrix


def vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


@dataclass(frozen=True)
class TeamLayout:
    count: int

    def __post_init__(self):
        if self.count < 3:
            raise ValueError('At least three aircraft required')

    @property
    def size(self):
        return 13+9*self.count

    @property
    def position(self):
        return slice(13, 13+3*self.count)

    @property
    def velocity(self):
        return slice(13+3*self.count, 13+6*self.count)

    @property
    def force(self):
        return slice(13+6*self.count, self.size)


def wrench_map(anchors, rotation):
    arms = validate_anchors(anchors)@np.asarray(rotation).T
    return np.vstack([np.tile(np.eye(3), (1, len(arms))), np.hstack([skew(a) for a in arms])])


def radial_seed(anchors, rotation, pressure=1.8):
    a = np.array(anchors, float, copy=True)
    a -= a.mean(axis=0)
    a[:, 2] = 0.
    a /= np.maximum(np.max(np.abs(a), axis=0), 1e-12)
    pressure=np.asarray(pressure)
    xy=np.array([float(pressure),.92*float(pressure)]) if pressure.ndim==0 else pressure
    if xy.shape!=(2,): raise ValueError('Scalar or two horizontal preload values required')
    return (a*np.r_[xy,0.])@np.asarray(rotation).T


class WrenchAllocator:
    """Smooth full-vector allocation, with optional convex force limits.

    A projected outward preload avoids the scalar allocator's restricted
    nullspace. The nonlinear geometry check belongs to the trajectory planner.
    """
    def __init__(self, anchors, maximum_tension=8.9, minimum_vertical=.55,
                 uav_mass=1.5, maximum_thrust=29.9, pressure=1.8):
        self.anchors = validate_anchors(anchors).copy()
        self.n = len(self.anchors)
        self.maximum_tension = maximum_tension
        self.minimum_vertical = minimum_vertical
        self.uav_mass, self.maximum_thrust = uav_mass, maximum_thrust
        self.pressure = pressure
        self.previous = None
        self.conic_calls = 0
        self.vertical_span_limit = None
        self.minimum_outward = None

    def allocate(self, rotation, wrench, measured=None):
        G = wrench_map(self.anchors, rotation)
        pinv = np.linalg.pinv(G)
        preload = radial_seed(self.anchors, rotation, self.pressure)
        preferred = preload.ravel()
        if self.previous is not None:
            # Keep a unique smooth force distribution when a wrench changes.
            preferred = .85*self.previous.ravel()+.15*preferred
        f = (pinv@wrench+(np.eye(3*self.n)-pinv@G)@preferred).reshape(self.n, 3)
        norm = np.linalg.norm(f, axis=1)
        cone_rows=[]
        cone_rhs=[]
        if self.vertical_span_limit is not None:
            spans=np.asarray(self.vertical_span_limit)
            if spans.shape!=(self.n,) or spans.min()<.10:raise RuntimeError('Invalid cable vertical span budget')
            outward=radial_seed(self.anchors,rotation,1.)
            outward[:,2]=0.;outward/=np.maximum(np.linalg.norm(outward,axis=1)[:,None],1e-12)
            for i,height in enumerate(spans):
                if height>=2.05:continue
                row=np.zeros(3*self.n)
                row[3*i:3*i+3]=-outward[i]
                row[3*i+2]=np.sqrt(max(0.,(2.05/height)**2-1.))
                cone_rows.append(row)
                cone_rhs.append(0.)
        if self.minimum_outward is not None:
            outward=radial_seed(self.anchors,rotation,1.)
            outward[:,2]=0.;outward/=np.maximum(np.linalg.norm(outward,axis=1)[:,None],1e-12)
            for i in range(self.n):
                row=np.zeros(3*self.n);row[3*i:3*i+3]=-outward[i]
                cone_rows.append(row);cone_rhs.append(-float(self.minimum_outward))
        cone_rows=np.asarray(cone_rows).reshape(-1,3*self.n)
        cone_ok=not len(cone_rows) or np.max(cone_rows@f.ravel()-cone_rhs)<=1e-9
        if (norm.max() <= self.maximum_tension and f[:, 2].min() >= self.minimum_vertical
                and np.linalg.norm(f+[0, 0, self.uav_mass*9.81], axis=1).max() <= self.maximum_thrust and cone_ok):
            status = 'projected'
        else:
            import clarabel
            from scipy import sparse
            n = self.n
            blocks, rhs, cones = [G], [np.asarray(wrench)], [clarabel.ZeroConeT(6)]
            vertical = np.zeros((n, 3*n))
            vertical[np.arange(n), 3*np.arange(n)+2] = -1.
            blocks.append(vertical); rhs.append(np.full(n, -self.minimum_vertical))
            cones.append(clarabel.NonnegativeConeT(n))
            if len(cone_rows):
                blocks.append(cone_rows);rhs.append(np.asarray(cone_rhs))
                cones.append(clarabel.NonnegativeConeT(len(cone_rows)))
            for i in range(n):
                selector = np.zeros((3, 3*n)); selector[:, 3*i:3*i+3] = np.eye(3)
                block = np.vstack([np.zeros((1, 3*n)), -selector])
                blocks.extend([block, block])
                rhs.extend([np.r_[self.maximum_tension, np.zeros(3)],
                            np.r_[self.maximum_thrust, 0., 0., self.uav_mass*9.81]])
                cones.extend([clarabel.SecondOrderConeT(4), clarabel.SecondOrderConeT(4)])
            settings = clarabel.DefaultSettings(); settings.verbose = False
            settings.tol_feas = 1e-9; settings.tol_gap_abs = 1e-9
            result = clarabel.DefaultSolver(sparse.eye(3*n, format='csc'), -preferred,
                sparse.csc_matrix(np.vstack(blocks)), np.concatenate(rhs), cones, settings).solve()
            self.conic_calls += 1
            status = str(result.status)
            if status not in ('Solved', 'AlmostSolved'):
                raise RuntimeError('Wrench allocation refused: '+status)
            f = np.asarray(result.x).reshape(n, 3)
        residual = float(np.linalg.norm(G@f.ravel()-wrench))
        if residual > 1e-6:
            raise RuntimeError('Wrench equality residual exceeds tolerance')
        self.previous = f.copy()
        return f, dict(residual=residual, status=status)


class TensionPoseServo:
    """Material-origin SE(3) feedback and measured cable-reaction inner loop.

    The allocated wrench and geometric endpoint references provide feedback.
    All force and rate caps are configuration inputs.
    """
    def __init__(self, anchors, theta, rest_length=2.5, stiffness=120., damping=4.,
                 uav_mass=1.5, pressure=1.8, acceleration_limit=1.5,
                 speed_limit=.80, attitude_gain=2., angular_gain=1.75,
                 measured_reaction_fraction=0.):
        self.anchors = validate_anchors(anchors).copy(); self.n = len(self.anchors)
        if not decode(theta)['physical']:
            raise ValueError('Physical identified inertia required')
        self.theta = np.asarray(theta).copy()
        self.rest_length, self.stiffness, self.damping = rest_length, stiffness, damping
        self.uav_mass = uav_mass
        self.allocator = WrenchAllocator(anchors, uav_mass=uav_mass, pressure=pressure)
        self.acceleration_limit, self.speed_limit = acceleration_limit, speed_limit
        self.kr, self.kw = attitude_gain, angular_gain
        self.measured_reaction_fraction = float(measured_reaction_fraction)
        self.previous_endpoint = None; self.endpoint_velocity = np.zeros((self.n, 3))
        self.previous_velocity = None
        self.minimum_separation = np.inf
        self.pose_correction = np.zeros(6)

    def command(self, row, uav_velocity, target, dt=.02):
        validate(row, self.n)
        p, v, r, omega, pu, tension = (np.asarray(row[k]) for k in
            ('position', 'velocity', 'rotation', 'omega', 'uav_position', 'tension'))
        vu = np.asarray(uav_velocity)
        e = .5*vee(target.rotation@r.T-r@target.rotation.T)
        wrench = wrench_matrix(target.rotation, target.omega_world,
                              target.acceleration, target.alpha_world)@self.theta
        if hasattr(target,'maximum_endpoint_z'):
            self.allocator.vertical_span_limit=(target.maximum_endpoint_z-target.position[2]
                -(self.anchors@target.rotation.T)[:,2])
        forces, info = self.allocator.allocate(target.rotation, wrench)
        arms = self.anchors@r.T; attachments = p+arms
        desired_tension = np.linalg.norm(forces, axis=1)
        directions = forces/desired_tension[:, None]
        actual_delta = pu-attachments
        actual_direction = actual_delta/np.linalg.norm(actual_delta, axis=1)[:, None]
        relative_v = vu-v-np.cross(omega, arms)
        rate = np.sum(relative_v*actual_direction, axis=1)
        # Preserve passive cable damping with attitude/motor latency. Cancelling
        # measured stretch-rate damping destabilizes the compliant system.
        extension = desired_tension/self.stiffness
        correction=np.r_[.3*(target.position-p),.3*e]
        self.pose_correction+=(1-np.exp(-dt/.15))*(correction-self.pose_correction)
        angle=np.linalg.norm(self.pose_correction[3:]);K=skew(self.pose_correction[3:])
        adjust=np.eye(3)+(np.sin(angle)/angle if angle>1e-8 else 1.)*K+((1-np.cos(angle))/angle**2 if angle>1e-8 else .5)*K@K
        endpoints = target.position+self.pose_correction[:3]+self.anchors@(adjust@target.rotation).T+(self.rest_length+extension[:, None])*directions
        if self.previous_endpoint is None:
            ref_v = v+np.cross(omega, arms)
        else:
            raw = (endpoints-self.previous_endpoint)/dt
            self.endpoint_velocity += (1-np.exp(-dt/.04))*(raw-self.endpoint_velocity)
            ref_v = self.endpoint_velocity.copy()
        self.previous_endpoint = endpoints.copy()
        ref_v *= np.minimum(1., self.speed_limit/np.maximum(np.linalg.norm(ref_v, axis=1), 1e-12))[:, None]
        acc = 20.*(endpoints-pu)+8.*(ref_v-vu)
        if self.previous_velocity is not None:
            # Differentiated reference acceleration is low pass filtered by the
            # endpoint-velocity filter; cap actual requested acceleration too.
            acc += (ref_v-self.previous_velocity)/dt
        self.previous_velocity = ref_v.copy()
        predicted_v = vu+dt*acc
        predicted_v *= np.minimum(1., self.speed_limit/np.maximum(np.linalg.norm(predicted_v, axis=1), 1e-12))[:, None]
        acc = (predicted_v-vu)/dt
        acc *= np.minimum(1., self.acceleration_limit/np.maximum(np.linalg.norm(acc, axis=1), 1e-12))[:, None]
        measured = tension[:, None]*actual_direction
        reaction = self.measured_reaction_fraction*measured+(1-self.measured_reaction_fraction)*forces
        command = self.uav_mass*(acc+[0, 0, 9.81])+reaction+.2*vu
        info.update(uav_target_neu=endpoints, desired_load_cable_forces_neu_n=forces,
                    requested_uav_acceleration_neu=acc, desired_wrench=wrench)
        return command, info
