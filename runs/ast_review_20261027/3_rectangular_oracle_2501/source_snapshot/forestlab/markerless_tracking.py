"""Timestamped geometric tracking baseline; sensor point clouds only.

Predictions are explicitly not measurements. No calibrated covariance, semantic
segmentation, multi-modal belief, or closed-loop aerial dynamics are claimed.
"""
import cv2
import numpy as np
from .markerless import ScanRegistration, voxel_reduce


class TemporalScanTracker:
    max_prediction_sec = .35
    max_linear_speed = 1.5
    max_angular_speed = 2.5

    def __init__(self, reference_world):
        self.registration = ScanRegistration(reference_world)
        self.last = None
        self.pending = None
        self.last_input_time = None
        self.velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

    def _prediction(self, timestamp):
        previous_time, position, rotation = self.last
        age = timestamp - previous_time
        return (position + self.velocity * age,
                cv2.Rodrigues(self.angular_velocity * age)[0] @ rotation, age)

    def _accept(self, timestamp, position, rotation, local):
        if local:
            previous_time, previous_position, previous_rotation = self.last
            dt = timestamp - previous_time
            v = (position - previous_position) / dt
            w = cv2.Rodrigues(rotation @ previous_rotation.T)[0].ravel() / dt
            for vector, cap in ((v, self.max_linear_speed), (w, self.max_angular_speed)):
                vector *= min(1., cap / max(1e-12, np.linalg.norm(vector)))
            self.velocity = .5 * self.velocity + .5 * v
            self.angular_velocity = .5 * self.angular_velocity + .5 * w
        else:
            self.velocity[:] = 0
            self.angular_velocity[:] = 0
        self.last = (timestamp, position.copy(), rotation.copy())
        self.pending = None
        return dict(valid=True, measurement_updated=True, state='tracking',
                    method='local_icp' if local else 'global_two_frame_confirmation',
                    position_ned=position.tolist(), rotation_scan_to_ned=rotation.tolist(),
                    measurement_age_sec=0., covariance_available=False)

    def _unobserved(self, timestamp, reason):
        if self.last is not None:
            position, rotation, age = self._prediction(timestamp)
            if age <= self.max_prediction_sec:
                return dict(valid=False, measurement_updated=False, state='prediction_only',
                            reason=reason, position_ned=position.tolist(),
                            rotation_scan_to_ned=rotation.tolist(), measurement_age_sec=age,
                            covariance_available=False)
            self.last = None
            self.velocity[:] = 0
            self.angular_velocity[:] = 0
        return dict(valid=False, measurement_updated=False, state='lost', reason=reason)

    def step(self, observed_world, timestamp):
        timestamp = float(timestamp)
        if not np.isfinite(timestamp) or (self.last_input_time is not None and timestamp <= self.last_input_time):
            raise ValueError('Exposure times must be finite and strictly increasing')
        self.last_input_time = timestamp
        model = self.registration
        observed = voxel_reduce(observed_world, model.voxel, 1800)
        if len(observed) < model.min_points:
            self.pending = None
            return self._unobserved(timestamp, 'insufficient_depth_points')
        if self.last is not None and timestamp - self.last[0] <= self.max_prediction_sec:
            predicted_p, predicted_r, age = self._prediction(timestamp)
            source = observed - model.origin
            r = predicted_r.T
            t = r @ (model.origin - predicted_p)
            score, r, t, distances = model._icp(source, r, t, 24)
            position = model.origin - r.T @ t
            rotation = r.T
            fraction = float((distances < .025).mean())
            shift = float(np.linalg.norm(position - predicted_p))
            angle = float(np.linalg.norm(cv2.Rodrigues(rotation @ predicted_r.T)[0]))
            if score <= .018 and fraction >= .65 and shift <= .08 + .5 * age and angle <= .25 + .5 * age:
                result = self._accept(timestamp, position, rotation, True)
                result.update(fit_rmse_m=score, inlier_fraction=fraction,
                              position_innovation_m=shift, angle_innovation_rad=angle)
                return result
            reason = 'temporal_gate' if score <= .018 and fraction >= .65 else 'poor_scan_alignment'
            return self._unobserved(timestamp, reason)

        # After a long gap do not report extrapolation as fresh sensor evidence.
        self.last = None
        estimate = model.estimate(observed)
        if not estimate['valid']:
            self.pending = None
            return dict(valid=False, measurement_updated=False, state='lost', reason=estimate['reason'])
        position = np.array(estimate['position_ned'])
        rotation = np.array(estimate['rotation_scan_to_ned'])
        if self.pending is not None:
            old_time, old_p, old_r = self.pending
            dt = timestamp - old_time
            shift = np.linalg.norm(position - old_p)
            angle = np.linalg.norm(cv2.Rodrigues(rotation @ old_r.T)[0])
            if dt <= .25 and shift <= .03 + self.max_linear_speed * dt and angle <= .1 + self.max_angular_speed * dt:
                result = self._accept(timestamp, position, rotation, False)
                result.update(fit_rmse_m=estimate['fit_rmse_m'], inlier_fraction=estimate['inlier_fraction'])
                return result
        self.pending = (timestamp, position, rotation)
        return dict(valid=False, measurement_updated=False, state='initializing',
                    reason='awaiting_second_global_confirmation')
