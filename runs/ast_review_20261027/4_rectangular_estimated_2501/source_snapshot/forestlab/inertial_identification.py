"""Linear integral rigid-body identification about a visible, fixed body origin.

Inputs are measured scan-frame motion and N cable tensions/directions. No
mass labels, density, CoM, accelerations, or plant object enter this module.
The momentum regressor is standard rigid-body mechanics, not a novelty claim.
"""
from __future__ import annotations

import numpy as np

FIELDS = ('time', 'position', 'velocity', 'rotation', 'omega', 'uav_position', 'tension')
PARAMETERS = ('mass', 'hx', 'hy', 'hz', 'Ixx_O', 'Iyy_O', 'Izz_O', 'Ixy_O', 'Ixz_O', 'Iyz_O')


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def tensor(v):
    xx, yy, zz, xy, xz, yz = v
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])


def parameters(mass, com, inertia_com):
    com, inertia_com = np.asarray(com, float), np.asarray(inertia_com, float)
    io = inertia_com+mass*((com@com)*np.eye(3)-np.outer(com, com))
    return np.r_[mass, mass*com, io[0, 0], io[1, 1], io[2, 2], io[0, 1], io[0, 2], io[1, 2]]


def decode(theta):
    theta = np.asarray(theta, float)
    if theta.shape != (10,) or not np.isfinite(theta).all() or theta[0] <= 0:
        raise ValueError('Ten finite parameters and positive mass required')
    m, com = theta[0], theta[1:4]/theta[0]
    ic = tensor(theta[4:])-m*((com@com)*np.eye(3)-np.outer(com, com))
    # Physical second moment, stronger than merely positive inertia.
    second = .5*np.trace(ic)*np.eye(3)-ic
    return dict(mass=float(m), com=com, inertia_com=ic,
                physical=bool(np.linalg.eigvalsh(second).min() > 0),
                minimum_second_moment=float(np.linalg.eigvalsh(second).min()))


def momentum_matrix(position, velocity, rotation, omega):
    """[linear momentum; angular momentum about a fixed WORLD origin]."""
    p, v, r, w = map(np.asarray, (position, velocity, rotation, omega))
    linear = np.zeros((3, 10)); linear[:, 0] = v
    linear[:, 1:4] = skew(w)@r
    angular = skew(p)@linear
    angular[:, 1:4] -= skew(v)@r
    for j in range(6):
        angular[:, 4+j] = r@tensor(np.eye(6)[j])@r.T@w
    return np.vstack([linear, angular])


def gravity_matrix(position, rotation, gravity=(0., 0., -9.81)):
    p, r, g = np.asarray(position), np.asarray(rotation), np.asarray(gravity)
    result = np.zeros((6, 10)); result[:3, 0] = g
    result[3:, 0] = np.cross(p, g)
    result[3:, 1:4] = -skew(g)@r
    return result


def wrench_matrix(rotation, omega, acceleration, alpha, gravity=(0., 0., -9.81)):
    """Required cable wrench at body origin O, world coordinates, linear in theta."""
    r, w, a, al = map(np.asarray, (rotation, omega, acceleration, alpha))
    a = a-np.asarray(gravity)
    out = np.zeros((6, 10)); out[:3, 0] = a
    out[:3, 1:4] = (skew(al)+skew(w)@skew(w))@r
    out[3:, 1:4] = -skew(a)@r
    for j in range(6):
        iw = r@tensor(np.eye(6)[j])@r.T
        out[3:, j+4] = iw@al+np.cross(w, iw@w)
    return out


def validate(row, count=None):
    if set(row) != set(FIELDS):
        raise ValueError('Observation must contain exactly the sensor allowlist')
    pu = np.asarray(row['uav_position'], float)
    if pu.ndim != 2 or pu.shape[1] != 3 or len(pu) < 3:
        raise ValueError('At least three measured UAV positions required')
    if count is not None and len(pu) != count:
        raise ValueError('Sensor team size does not match calibrated anchors')
    for name, shape in (('position', (3,)), ('velocity', (3,)), ('rotation', (3, 3)),
                        ('omega', (3,)), ('uav_position', (len(pu), 3)), ('tension', (len(pu),))):
        a = np.asarray(row[name], float)
        if a.shape != shape or not np.isfinite(a).all():
            raise ValueError('Invalid sensor field: '+name)
    if not np.isfinite(row['time']) or np.min(row['tension']) < 0:
        raise ValueError('Invalid timestamp or negative tension observation')
    r = np.asarray(row['rotation'])
    if not np.allclose(r.T@r, np.eye(3), atol=1e-8) or np.linalg.det(r) < .999999:
        raise ValueError('Rotation observation must be in SO(3)')


def validate_anchors(anchors):
    a = np.asarray(anchors, float)
    if a.ndim != 2 or a.shape[1] != 3 or len(a) < 3 or not np.isfinite(a).all():
        raise ValueError('Expected N >= 3 finite calibrated anchors')
    if np.linalg.matrix_rank(a-a.mean(axis=0)) < 2:
        raise ValueError('Noncollinear attachment geometry required')
    return a


def cable_wrench(row, anchors, origin):
    r, p, pu = (np.asarray(row[k], float) for k in ('rotation', 'position', 'uav_position'))
    attachments = p+np.asarray(anchors)@r.T
    delta = pu-attachments; lengths = np.linalg.norm(delta, axis=1)
    if lengths.min() <= 1e-6:
        raise ValueError('Degenerate measured cable')
    f = delta/lengths[:, None]*np.asarray(row['tension'])[:, None]
    return np.r_[f.sum(axis=0), np.cross(attachments-origin, f).sum(axis=0)]


def integral_system(rows, anchors, window_sec=.4):
    """Non-overlapping integral equations; final incomplete block is excluded."""
    if len(rows) < 2:
        raise ValueError('At least two sensor samples required')
    anchors = validate_anchors(anchors)
    for row in rows:
        validate(row, len(anchors))
    times = np.array([r['time'] for r in rows])
    if np.any(np.diff(times) <= 0) or not .02 <= window_sec:
        raise ValueError('Increasing timestamps and >=20ms windows required')
    # One fixed numeric world origin, not a moving body origin. Translation
    # removes large, arbitrary angular-momentum lever arms from conditioning.
    origin = np.asarray(rows[0]['position'], float)
    ms, gs, fs = [], [], []
    for row in rows:
        p = np.asarray(row['position'])-origin
        ms.append(momentum_matrix(p, row['velocity'], row['rotation'], row['omega']))
        gs.append(gravity_matrix(p, row['rotation']))
        fs.append(cable_wrench(row, anchors, origin))
    ms, gs, fs = map(np.asarray, (ms, gs, fs))
    dt = np.diff(times)
    ig = np.concatenate([np.zeros((1, 6, 10)), np.cumsum((gs[:-1]+gs[1:])*.5*dt[:, None, None], axis=0)])
    iff = np.concatenate([np.zeros((1, 6)), np.cumsum((fs[:-1]+fs[1:])*.5*dt[:, None], axis=0)])
    blocks, targets, bounds = [], [], []
    start = 0
    while True:
        end = int(np.searchsorted(times, times[start]+window_sec-1e-9))
        if end >= len(rows):
            break
        blocks.append(ms[end]-ms[start]-(ig[end]-ig[start]))
        targets.append(iff[end]-iff[start]); bounds.append([start, end])
        start = end
    if not blocks:
        raise ValueError('No complete integral window')
    omega = np.array([row['omega'] for row in rows])
    return dict(A=np.concatenate(blocks), b=np.concatenate(targets), blocks=np.asarray(blocks),
                targets=np.asarray(targets), intervals=bounds, origin=origin,
                omega_information=omega.T@omega/len(omega))


def fit(rows, anchors, window_sec=.4, rank_rtol=1e-5, tension_floor=.01):
    system = integral_system(rows, anchors, window_sec)
    result = fit_system(system, len(rows), rows[-1]['time'], rank_rtol)
    minimum_tension = min(float(np.min(row['tension'])) for row in rows)
    result['minimum_observed_tension'] = minimum_tension
    result['tension_gate_passed'] = minimum_tension > tension_floor
    if not result['tension_gate_passed']:
        result['activated'] = False
        result['activation_refusal'] = 'tension_at_or_below_sensor_error_bound'
    return result


def fit_system(system, samples, observed_until, rank_rtol=1e-5):
    a, b = system['A'], system['b']
    scale = np.linalg.norm(a, axis=0)
    # Zero columns remain zero; normalizing must not hide absent excitation.
    normalized = a/np.maximum(scale, 1e-12)
    _, singular, vt = np.linalg.svd(normalized, full_matrices=False)
    rank = int(np.sum(singular > rank_rtol*singular[0]))
    theta = np.linalg.lstsq(normalized, b, rcond=rank_rtol)[0]/np.maximum(scale, 1e-12)
    residual = a@theta-b
    full_rank = rank == 10
    decoded = decode(theta) if theta[0] > 0 else None
    condition = float(singular[0]/singular[-1]) if singular[-1] > 0 else None
    omega_floor = float(np.linalg.eigvalsh(system['omega_information']).min())
    excitation_passed = bool(len(system['blocks']) >= 8 and omega_floor >= 1e-4 and condition is not None and condition <= 1000.)
    return dict(theta=theta, rank=rank, identifiable=full_rank,
                activated=bool(full_rank and excitation_passed and decoded and decoded['physical']),
                decoded=decoded if full_rank else None, singular_values=singular,
                normalized_condition=condition, excitation_passed=excitation_passed,
                minimum_omega_information=omega_floor,
                residual_rmse=float(np.sqrt(np.mean(residual**2))),
                nullspace_normalized=vt[rank:], samples=samples, windows=len(system['blocks']),
                observed_until_sec=float(observed_until), system=system,
                uncertainty_claim='No confidence or guaranteed parameter set from least squares')


class OnlineIdentifier:
    """Causal integral accumulation, with explicit rank refusal at every solve."""
    def __init__(self, anchors, window_sec=.4, tension_floor=.01):
        self.anchors = validate_anchors(anchors).copy(); self.window_sec = float(window_sec)
        self.tension_floor = float(tension_floor); self.minimum_observed_tension = np.inf
        self.previous = None; self.origin = None; self.start = None
        self.blocks, self.targets, self.intervals = [], [], []
        self.ig = np.zeros((6, 10)); self.iff = np.zeros(6); self.samples = 0
        self.omega_information = np.zeros((3, 3))

    def observe(self, row):
        validate(row, len(self.anchors))
        self.minimum_observed_tension = min(self.minimum_observed_tension,
                                            float(np.min(row['tension'])))
        w = np.asarray(row['omega']); self.omega_information += np.outer(w, w)
        if self.origin is None:
            self.origin = np.array(row['position'], float, copy=True)
        p = np.asarray(row['position'])-self.origin
        item = dict(time=float(row['time']), M=momentum_matrix(p, row['velocity'], row['rotation'], row['omega']),
                    G=gravity_matrix(p, row['rotation']), F=cable_wrench(row, self.anchors, self.origin))
        new_window = False
        if self.previous is not None:
            dt = item['time']-self.previous['time']
            if dt <= 0:
                raise ValueError('Online samples must be strictly ordered')
            self.ig += .5*dt*(item['G']+self.previous['G'])
            self.iff += .5*dt*(item['F']+self.previous['F'])
            if item['time']-self.start['time'] >= self.window_sec-1e-9:
                self.blocks.append(item['M']-self.start['M']-self.ig)
                self.targets.append(self.iff.copy())
                self.intervals.append([self.start['index'], self.samples])
                self.ig[:] = 0; self.iff[:] = 0
                self.start = dict(time=item['time'], M=item['M'].copy(), index=self.samples)
                new_window = True
        else:
            self.start = dict(time=item['time'], M=item['M'].copy(), index=0)
        self.previous = item; self.samples += 1
        return new_window

    def estimate(self):
        if not self.blocks:
            return dict(activated=False, reason='no_complete_window', rank=0)
        system = dict(A=np.concatenate(self.blocks), b=np.concatenate(self.targets),
                      blocks=np.asarray(self.blocks), targets=np.asarray(self.targets),
                      intervals=self.intervals.copy(), origin=self.origin.copy(),
                      omega_information=self.omega_information/self.samples)
        result = fit_system(system, self.samples, self.previous['time'])
        result['minimum_observed_tension'] = self.minimum_observed_tension
        result['tension_gate_passed'] = self.minimum_observed_tension > self.tension_floor
        if not result['tension_gate_passed']:
            result['activated'] = False
            result['activation_refusal'] = 'tension_at_or_below_sensor_error_bound'
        return result
