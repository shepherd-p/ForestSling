"""MuJoCo N-quadrotor rigid load bench with motor/attitude dynamics.

The engine integrates all free bodies. Motor site forces produce torques;
no command changes a body's position or attitude. Slack cable forces are zero.
True inertia is private to the plant; sensors return only the public allowlist.
"""
from __future__ import annotations

from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
sys.path.append(str(Path(__file__).resolve().parents[1]/'.deps/independent_physics'))
import mujoco

from .inertial_identification import parameters
from .tension_transport import wrench_map, radial_seed, vee


def vector(v):
    return ' '.join(format(float(x), '.16g') for x in np.asarray(v).ravel())


def rotation(q):
    out = np.empty(9); mujoco.mju_quat2Mat(out, np.asarray(q, float))
    return out.reshape(3, 3)


def rpy(angles):
    x, y, z = np.asarray(angles)
    cx, sx, cy, sy, cz, sz = np.cos(x), np.sin(x), np.cos(y), np.sin(y), np.cos(z), np.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
        [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]])


def payload_asset(shape='irregular', count=4, ballast='offset_z', seed=1):
    """Declared evaluator asset; positive hidden voxel weights.

    Calibration returned separately includes shape and installed anchors only.
    """
    rng = np.random.default_rng(seed)
    spacing = np.array([.10, .09, .07])
    ijk = np.array(list(np.ndindex((9, 7, 3))))
    if shape == 'irregular':
        # L cutout plus an asymmetric upper step; nonconvex, not a box proxy.
        keep = ~((ijk[:, 0] > 5)&(ijk[:, 1] > 3))
        keep &= ~((ijk[:, 2] == 2)&(ijk[:, 0] < 3)&(ijk[:, 1] < 2))
        ijk = ijk[keep]
    cells = (ijk+.5)*spacing
    cells -= cells.mean(axis=0)
    target_xy = (np.array([[-.33,-.22],[-.33,.22],[.32,-.18],[.32,.22]]) if count == 4
                 else np.array([[-.32,-.22],[-.25,.22],[.32,-.08]]))
    columns = {}
    for i, p in enumerate(cells):
        k = tuple(ijk[i, :2])
        if k not in columns or p[2] > columns[k][2]: columns[k] = p.copy()
    tops = np.array(list(columns.values())); anchors = []
    for xy in target_xy:
        p = tops[np.argmin(np.linalg.norm(tops[:, :2]-xy, axis=1))].copy()
        p[2] += spacing[2]/2+.04; anchors.append(p)
    anchors = np.asarray(anchors)
    mass = 1.05+float(rng.uniform(-.08,.08))
    direction = dict(center=[0.,0.,0.], offset_xy=[1.4,-1.1,0.], offset_z=[.9,-.8,2.])[ballast]
    weights = np.exp((cells/np.ptp(cells, axis=0))@direction)
    weights *= mass/weights.sum(); com = weights@cells/mass
    d = cells-com; covariance = (d.T*weights)@d/mass
    inertia = mass*(np.trace(covariance)*np.eye(3)-covariance)
    inertia += mass/12*np.diag(np.sum(spacing**2)-spacing**2)
    nominal_cov = cells.T@cells/len(cells)
    nominal_inertia = mass*(np.trace(nominal_cov)*np.eye(3)-nominal_cov)+mass/12*np.diag(np.sum(spacing**2)-spacing**2)
    public = dict(shape=shape, count=count, cells=cells, half=spacing/2, anchors=anchors,
        nominal_theta=parameters(1.05, [0.,0.,0.], nominal_inertia*1.05/mass))
    hidden = dict(mass=mass, com=com, inertia=inertia, theta=parameters(mass, com, inertia), weights=weights)
    return public, hidden


class SixDofSling:
    def __init__(self, public, hidden, dt=.002, rest_length=2.0, obstacles=(), seed=0):
        self.public = public; self._hidden = hidden; self.n = len(public['anchors'])
        self.rest_length = float(rest_length); self.stiffness = 180.; self.damping = 5.
        self.uav_mass = 1.0; self.rng = np.random.default_rng(seed)
        self.motor_tau = .025; self.motor_limit = 7.5
        self.mixer = np.array([[1.,1.,1.,1.],[0.,.16,0.,-.16],[-.16,0.,.16,0.],[.018,-.018,.018,-.018]])
        root = ET.Element('mujoco', model=f'sensor_transport_{self.n}')
        ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false')
        ET.SubElement(root, 'option', timestep=str(dt), gravity='0 0 -9.81',
                      integrator='implicitfast', solver='Newton', tolerance='1e-10')
        visual = ET.SubElement(root, 'visual')
        ET.SubElement(visual, 'global', offwidth='960', offheight='720')
        ET.SubElement(visual, 'map', znear='.01', zfar='50')
        world = ET.SubElement(root, 'worldbody')
        ET.SubElement(world, 'light', pos='1 -3 9', dir='0 0 -1', diffuse='.8 .8 .8')
        ET.SubElement(world, 'geom', name='ground', type='plane', size='30 30 .1', rgba='.22 .29 .21 1')
        body = ET.SubElement(world, 'body', name='payload', pos='0 0 3')
        ET.SubElement(body, 'freejoint', name='payload_joint')
        I = hidden['inertia']; full = I[(0,1,2,0,0,1),(0,1,2,1,2,2)]
        ET.SubElement(body, 'inertial', pos=vector(hidden['com']), mass=str(hidden['mass']), fullinertia=vector(full))
        for j, cell in enumerate(public['cells']):
            ET.SubElement(body, 'geom', name=f'load_cell{j}', type='box', pos=vector(cell),
                size=vector(public['half']), mass='0', rgba='.85 .48 .12 1', friction='.6 .01 .001')
        for i, a in enumerate(public['anchors']):
            ET.SubElement(body, 'site', name=f'load_anchor{i}', pos=vector(a), size='.018', rgba='.1 .1 .1 1')
        # A nominal configuration is used for all hidden ballast worlds.
        G = wrench_map(public['anchors'], np.eye(3)); pinv = np.linalg.pinv(G)
        f = (pinv@np.array([0.,0.,1.05*9.81,0.,0.,0.])+
             (np.eye(3*self.n)-pinv@G)@radial_seed(public['anchors'],np.eye(3),1.25).ravel()).reshape(self.n,3)
        self.nominal_hover_forces = f.copy()
        tension = np.linalg.norm(f, axis=1)
        endpoints = np.array([0.,0.,3.])+public['anchors']+(rest_length+tension[:,None]/self.stiffness)*f/tension[:,None]
        motor_sites = []
        colors = ['.2 .6 .9 1','.9 .2 .2 1','.2 .85 .5 1','.8 .4 .9 1']
        for i in range(self.n):
            force = f[i]+[0.,0.,self.uav_mass*9.81]
            z = force/np.linalg.norm(force); y = np.cross(z,[1.,0.,0.]); y /= np.linalg.norm(y)
            R = np.column_stack([np.cross(y,z),y,z]); quat = np.empty(4)
            mujoco.mju_mat2Quat(quat,R.ravel())
            b = ET.SubElement(world, 'body', name=f'uav{i}', pos=vector(endpoints[i]+R@[0.,0.,.015]), quat=vector(quat))
            ET.SubElement(b, 'freejoint', name=f'uav_joint{i}')
            ET.SubElement(b, 'inertial', pos='0 0 0', mass=str(self.uav_mass), diaginertia='.009 .009 .016')
            ET.SubElement(b, 'geom', type='sphere', size='.22', rgba=colors[i%4], mass='0')
            ET.SubElement(b, 'site', name=f'endpoint{i}', pos='0 0 -.015', size='.01')
            for j, xy in enumerate(((.16,0),(0,.16),(-.16,0),(0,-.16))):
                name = f'motor{i}_{j}'; motor_sites.append(name)
                ET.SubElement(b, 'site', name=name, pos=vector([*xy,0.]), size='.055 .002', type='cylinder', rgba='.12 .12 .12 1')
        tendon = ET.SubElement(root, 'tendon')
        for i in range(self.n):
            t = ET.SubElement(tendon, 'spatial', name=f'cable{i}', width='.006', rgba='.15 .15 .15 1')
            ET.SubElement(t, 'site', site=f'load_anchor{i}'); ET.SubElement(t, 'site', site=f'endpoint{i}')
        actuators = ET.SubElement(root, 'actuator')
        for k, site in enumerate(motor_sites):
            ET.SubElement(actuators, 'general', name=site, site=site, gear=f'0 0 1 0 0 {(.018 if k%2 == 0 else -.018)}',
                dyntype='filterexact', dynprm=str(self.motor_tau), ctrllimited='true', ctrlrange='0 7.5',
                forcelimited='true', forcerange='0 7.5')
        for j, o in enumerate(obstacles):
            ET.SubElement(world, 'geom', name=f'obstacle{j}', type='box', pos=vector(o['center']),
                size=vector(o['half']), rgba='.22 .45 .18 1')
        self.xml = ET.tostring(root, encoding='unicode')
        self.model = mujoco.MjModel.from_xml_string(self.xml); self.data = mujoco.MjData(self.model)
        self.uav_ids = [mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,f'uav{i}') for i in range(self.n)]
        self.site_ids = [mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SITE,f'endpoint{i}') for i in range(self.n)]
        self.data.act[:] = ((np.linalg.norm(f+[0,0,9.81],axis=1)/4)[:,None]*np.ones((1,4))).ravel()
        self.data.ctrl[:] = self.data.act
        mujoco.mj_forward(self.model,self.data)
        self.minimum_tension = np.inf; self.maximum_tension = 0.; self.maximum_tilt = 0.
        self.maximum_motor = 0.; self.minimum_extension = np.inf
        self.collision_steps = 0; self.motor_saturation_intervals = 0
        self.previous_target_rotation = [None]*self.n
        self.target_world_rate = np.zeros((self.n,3))
        self.maximum_aircraft_speed = 0.; self.maximum_aircraft_acceleration = 0.

    def cable_state(self):
        d = self.data
        extension = d.ten_length-self.rest_length
        tension = np.where(extension > 0., np.maximum(0., self.stiffness*extension+self.damping*d.ten_velocity), 0.)
        return extension.copy(), tension

    def observation(self, noise=1.):
        d = self.data; R = rotation(d.qpos[3:7]); v = d.qvel[:3]
        w = R@d.qvel[3:6]
        pu = np.asarray([d.site_xpos[i] for i in self.site_ids])
        _, tension = self.cable_state()
        def perturb(value, bound):
            value = np.asarray(value); return value+self.rng.uniform(-bound,bound,value.shape)*noise
        return dict(time=float(d.time), position=perturb(d.qpos[:3],.0005), velocity=perturb(v,.001),
            rotation=R@rpy(self.rng.uniform(-.0003,.0003,3)*noise), omega=perturb(w,.001),
            uav_position=perturb(pu,.0005), tension=np.maximum(0.,perturb(tension,.01)))

    def uav_velocities(self):
        out = []
        for i, sid in enumerate(self.site_ids):
            vel = np.zeros(6); mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_SITE,sid,vel,0)
            out.append(vel[3:])
        return np.asarray(out)

    def cable_telemetry(self, noise=1.):
        pu=np.asarray([self.data.site_xpos[i] for i in self.site_ids])
        _,ten=self.cable_state()
        return dict(time=float(self.data.time),
            uav_position=pu+self.rng.uniform(-.0005,.0005,pu.shape)*noise,
            uav_velocity=self.uav_velocities()+self.rng.uniform(-.001,.001,(self.n,3))*noise,
            tension=np.maximum(0.,ten+self.rng.uniform(-.01,.01,ten.shape)*noise))

    def motor_command(self, desired_world_force):
        commands = []
        for i, force in enumerate(np.asarray(desired_world_force)):
            offset = 7*(i+1); v_off = 6*(i+1)
            R = rotation(self.data.qpos[offset+3:offset+7]); w = self.data.qvel[v_off+3:v_off+6]
            z = force/max(np.linalg.norm(force),1e-12)
            # Fixed common yaw, with continuous attitude error at modest tilt.
            y = np.cross(z,[1.,0.,0.]); y /= max(np.linalg.norm(y),1e-12)
            target = np.column_stack([np.cross(y,z),y,z])
            error = .5*vee(target.T@R-R.T@target)
            if self.previous_target_rotation[i] is not None:
                raw = vee((target-self.previous_target_rotation[i])/.02@target.T)
                self.target_world_rate[i] += (1-np.exp(-.02/.04))*(raw-self.target_world_rate[i])
            self.previous_target_rotation[i] = target.copy()
            desired_rate = R.T@self.target_world_rate[i]
            moment = -.85*error-.17*(w-desired_rate)+np.cross(w,np.array([.009,.009,.016])*w)
            total = max(0.,float(force@R[:,2]))
            motors = np.linalg.solve(self.mixer,np.r_[total,moment])
            if motors.min() < 0. or motors.max() > self.motor_limit: self.motor_saturation_intervals += 1
            commands.append(np.clip(motors,0.,self.motor_limit))
        return np.asarray(commands)

    def advance(self, motors, dt=.02):
        m,d = self.model,self.data
        steps = round(dt/m.opt.timestep)
        if abs(steps*m.opt.timestep-dt)>1e-10: raise ValueError('Whole substeps required')
        d.ctrl[:] = np.asarray(motors).ravel()
        for _ in range(steps):
            mujoco.mj_step1(m,d)
            extension,tension = self.cable_state()
            J = np.asarray(d.ten_J).reshape(self.n,m.nv)
            d.qfrc_applied[:] = -tension@J
            # Linear air drag on aircraft only; material-origin estimator has
            # no unknown environmental wrench during the identification trial.
            for i in range(self.n):
                v_off=6*(i+1); d.qfrc_applied[v_off:v_off+3] -= .2*d.qvel[v_off:v_off+3]
            self.minimum_tension = min(self.minimum_tension,float(tension.min()))
            self.maximum_tension = max(self.maximum_tension,float(tension.max()))
            self.minimum_extension = min(self.minimum_extension,float(extension.min()))
            self.maximum_motor = max(self.maximum_motor,float(d.act.max()))
            for i in range(self.n):
                R = rotation(d.qpos[7*(i+1)+3:7*(i+1)+7])
                self.maximum_tilt = max(self.maximum_tilt,float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))))
            if d.ncon: self.collision_steps += 1
            mujoco.mj_step2(m,d)
            for i in range(self.n):
                offset=6*(i+1)
                self.maximum_aircraft_speed=max(self.maximum_aircraft_speed,float(np.linalg.norm(d.qvel[offset:offset+3])))
                self.maximum_aircraft_acceleration=max(self.maximum_aircraft_acceleration,float(np.linalg.norm(d.qacc[offset:offset+3])))
        mujoco.mj_forward(m,d)

    def record(self):
        return np.r_[self.data.time,self.data.qpos.copy(),self.data.qvel.copy(),self.data.act.copy(),self.cable_state()[1]]

    def excitation_force(self, start, t, mode='fixed', amplitude=1.):
        def path(t):
            u=np.clip(t/2.,0.,1.); ramp=10*u**3-15*u**4+6*u**5
            v=np.clip((t-12.)/2.,0.,1.)
            ramp*=1.-(10*v**3-15*v**4+6*v**5)
            angles=np.array([.055*np.sin(.73*t),.055*np.sin(.97*t),.08*np.sin(1.17*t)])
            if mode == 'roll': angles[0]*=1.4
            R=rpy(amplitude*ramp*angles)
            center=np.array([0.,0.,3.])
            common=np.array([.08*np.sin(.83*t),.07*np.sin(1.07*t),.04*np.sin(1.29*t)])
            return center+(start-center)@R.T+amplitude*ramp*common
        h=.002; p=path(t); vm=(path(t+h)-path(t-h))/(2*h); acc=(path(t+h)-2*p+path(t-h))/h**2
        row=self.cable_telemetry(noise=1.)
        pu=np.asarray(row['uav_position']); vu=self.uav_velocities()
        a=acc+18.*(p-pu)+7.*(vm-vu)
        force=self.nominal_hover_forces
        return self.uav_mass*(a+[0,0,9.81])+force+.2*vu
