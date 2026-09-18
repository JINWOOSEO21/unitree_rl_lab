"""MIT Cheetah 3 III-H inspired proprioceptive odometry (NumPy only).

18 states: base position/velocity and four foot-link origins in gravity world.
Attitude is a separate gyro/gravity complementary filter. Unlike MIT's public
flat-ground implementation, there is no foot-height=0 observation. No IMU
acceleration bias or scale is fitted. Go2 spherical-foot rolling is modelled
using the existing contract radius and world vertical, as in leg_odometry.
"""
from dataclasses import dataclass
import numpy as np
from .kinematics import quat_to_mat
from .leg_odometry import FOOT_LINKS_IL, LegOdomCfg


def multiply(a, b):
    w, v = a[0], a[1:]
    s, u = b[0], b[1:]
    return np.r_[w*s-np.dot(v,u), w*u+s*v+np.cross(v,u)]


def rotation_quaternion(rotvec):
    angle = np.linalg.norm(rotvec)
    return np.r_[np.cos(angle/2), rotvec*(.5 if angle < 1e-10 else np.sin(angle/2)/angle)]


@dataclass
class MitConfig(LegOdomCfg):
    attitude_gain: float = .1
    acceleration_noise: float = .8       # m/s^2; discrete acceleration uncertainty
    foot_process_noise: float = 1e-6    # m^2/s in settled stance
    foot_position_noise: float = .005   # m, standard deviation
    foot_velocity_noise: float = .08    # m/s, standard deviation


class MitOdometry:
    def __init__(self, kin, cfg=None):
        self.kin, self.cfg = kin, cfg or MitConfig()
        self.feet = [kin.link_names.index(n) for n in FOOT_LINKS_IL]
        self.chains = []
        for foot in self.feet:
            chain = []
            while foot >= 0:
                if kin.joint[foot] >= 0:
                    chain.append(foot)
                foot = int(kin.parent[foot])
            self.chains.append(chain)
        self.x = np.zeros(18)
        self.P = np.eye(18)*.01
        self.orientation = None
        self.initial_quaternion = None
        self.previous_t = None
        self.last_support_t = None
        self.force_lp = None
        self.contact_since = np.full(4, np.nan)
        self.was_support = np.zeros(4, dtype=bool)
        self.last_n_both = 0
        self.last_branch = 0
        self.reset_feet = True
        self.max_innovation = 0.
        self.eye = np.eye(18)

    def foot_kinematics(self, q, dq):
        """FK plus analytic translational/angular Jacobian products, IL order."""
        p, rotations = self.kin.link_poses_base(q)
        velocities, angular = np.zeros((4,3)), np.zeros((4,3))
        for i, chain in enumerate(self.chains):
            for link in chain:
                axis = rotations[link] @ self.kin.axis[link]
                w = axis*dq[self.kin.joint[link]]
                velocities[i] += np.cross(w, p[self.feet[i]]-p[link])
                angular[i] += w
        return p[self.feet], velocities, angular

    def resume_after_gap(self):
        # Hold position, discard unknown velocity/contact history. The adapter
        # reports the gap so the bridge clears the terrain map.
        self.x[3:6] = 0.
        self.P = np.eye(18)*.01
        self.previous_t = None
        self.force_lp = None
        self.contact_since[:] = np.nan
        self.was_support[:] = False
        self.last_support_t = None
        self.reset_feet = True
        self.last_branch = 0
        self.last_n_both = 0

    def step(self, t, q, dq, measured_quaternion, gyro, acc, forces):
        dt = 0. if self.previous_t is None else t-self.previous_t
        if dt < 0 or dt > self.cfg.max_dt_s:
            raise ValueError('MIT timestep out of range')
        if self.orientation is None:
            self.orientation = measured_quaternion.copy()
            self.initial_quaternion = measured_quaternion.copy()
        elif self.previous_t is None:
            # Reacquire orientation after a gap; do not integrate a missing gyro.
            self.orientation = measured_quaternion.copy()
        R = quat_to_mat(self.orientation)
        norm = np.linalg.norm(acc)
        correction = np.zeros(3)
        if norm > 1e-6:
            gain = self.cfg.attitude_gain*np.clip(1-abs(norm-9.81)/9.81, 0, 1)
            correction = gain*np.cross(acc/norm, R.T @ np.array([0.,0.,1.]))
        self.orientation = multiply(self.orientation, rotation_quaternion((gyro+correction)*dt))
        self.orientation /= np.linalg.norm(self.orientation)
        R = quat_to_mat(self.orientation)
        feet, dfeet, joint_omega = self.foot_kinematics(q, dq)
        relative = (R @ feet.T).T
        relative_velocity = (R @ (dfeet+np.cross(gyro, feet)).T).T
        foot_omega = (R @ (joint_omega+gyro).T).T
        rolling = self.cfg.foot_radius*np.cross(foot_omega, np.array([0.,0.,1.]))
        if self.force_lp is None:
            self.force_lp = forces.copy()
        else:
            alpha = min(1., dt/self.cfg.force_lp_tau_s) if self.cfg.force_lp_tau_s > 0 else 1.
            self.force_lp += alpha*(forces-self.force_lp)
        above = self.force_lp > self.cfg.contact_force_thr
        self.contact_since = np.where(above, np.where(np.isnan(self.contact_since), t, self.contact_since), np.nan)
        support = above & ((t-self.contact_since) >= self.cfg.contact_settle_s)
        if self.reset_feet:
            self.x[6:] = (self.x[:3]+relative).ravel()
            self.reset_feet = False
        A = self.eye.copy()
        A[:3,3:6] = np.eye(3)*dt
        B = np.zeros((18,3))
        B[:3] = np.eye(3)*(.5*dt*dt)
        B[3:6] = np.eye(3)*dt
        self.x = A @ self.x+B @ (R @ acc+np.array([0.,0.,-9.81]))
        Q = B @ B.T*self.cfg.acceleration_noise**2
        Q[3:6,3:6] += np.eye(3)*1e-4*dt
        Q[6:,6:] += np.eye(12)*self.cfg.foot_process_noise*dt
        self.P = A @ self.P @ A.T+Q
        self.max_innovation = 0.
        for i in range(4):
            sl = slice(6+3*i,9+3*i)
            if not support[i] or not self.was_support[i]:
                # New foothold: inherit base covariance and uncertainty of FK,
                # rather than falsely treating the new world foot as a landmark.
                self.x[sl] = self.x[:3]+relative[i]
                self.P[sl,:] = self.P[:3,:].copy()
                self.P[:,sl] = self.P[:,:3].copy()
                self.P[sl,sl] = self.P[:3,:3]+np.eye(3)*self.cfg.foot_position_noise**2
            else:
                self.x[sl] += rolling[i]*dt
            if not support[i]:
                continue
            observed_velocity = rolling[i]-relative_velocity[i]
            age = t-self.contact_since[i]-self.cfg.contact_settle_s
            trust = min(1., max(.1, age/.02))
            self.max_innovation = max(self.max_innovation,
                                      float(np.linalg.norm(observed_velocity-self.x[3:6])))
            # Diagonal measurement noise permits scalar Kalman updates, exactly
            # equivalent to the 6D position/velocity observation. Avoid tiny
            # LAPACK solves spawning BLAS workers on the Jetson.
            for j in range(6):
                if j < 3:
                    foot_index = 6+3*i+j
                    ph = self.P[:,foot_index]-self.P[:,j]
                    predicted = self.x[foot_index]-self.x[j]
                    variance = self.cfg.foot_position_noise**2/trust
                    s = ph[foot_index]-ph[j]+variance
                    innovation = relative[i,j]-predicted
                else:
                    ph = self.P[:,j].copy()
                    variance = self.cfg.foot_velocity_noise**2/trust
                    s = ph[j]+variance
                    innovation = observed_velocity[j-3]-self.x[j]
                if not np.isfinite(s) or s <= 0:
                    raise RuntimeError('invalid MIT innovation covariance')
                gain = ph/s
                self.x += gain*innovation
                # Expanded scalar Joseph form, O(n^2), with symmetric terms.
                self.P += s*np.outer(gain,gain)-np.outer(gain,ph)-np.outer(ph,gain)
        self.P = (self.P+self.P.T)*.5
        if not np.isfinite(self.x).all() or not np.isfinite(self.P).all():
            raise RuntimeError('nonfinite MIT filter state')
        self.last_n_both = int(support.sum())
        if support.any():
            self.last_support_t = t
            self.last_branch = 1
        elif self.last_support_t is not None and t-self.last_support_t <= self.cfg.max_hold_s:
            self.last_branch = 2
        else:
            self.last_branch = 4
            # Beyond the bounded flight window the map is invalid. Prevent
            # unlimited inertial drift; retain gyro tracking for reacquisition.
            self.x[3:6] = 0.
        self.was_support = support
        self.previous_t = t
        return self.x[:3].copy()
