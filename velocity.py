import numpy as np
from unit import mps2kph, kph2mps 

from scipy.constants import g
from math import sin, sqrt

class VelocityProfile:
    """
    Generate and stores a velocity profile for a given racing line and vehicle
    """
    def __init__(self, vehicle, racing_line_xy_arr, \
                 racing_line_samples, curvature, \
                 tuning_parameter, \
                 length_closed):
        """
        Generate a velocity profile for the given racing line,
        vehicle and other constraints.
        racing_line_samples and curvature should not include the overlapping element 
        for closed racing line
        The length of a closed racing line should be supplied in length_closed
        if the racing line is not closed, length_closed should be None
        """
        self.vehicle = vehicle
        self.racing_line_xy_arr = racing_line_xy_arr # NOC = NON+1 if closed else NON
        self.racing_line_samples = racing_line_samples # NON
        self.curvature = curvature # NON
        self.tuning_parameter = tuning_parameter # NON of track
        self.max_length_of_closed_track = length_closed
        self.closed = length_closed is not None
        self.tuning_parameter.resize_with_sample(self.racing_line_xy_arr, \
                                                 self.closed) # NON
        self.v_max = None
        self.v_acc_limit = None
        self.v_dec_limit = None
        self.limit_local_velocites()
        self.limit_acceleration()
        self.limit_deceleration()
        self.ax = None
        self.ay = None
        self.v  = None
        self.update_profile()
        # print("v len    : ", len(self.v)) #[DEBUG]
        # print("ax len   : ", len(self.ax)) #[DEBUG]
        # print("ay len   : ", len(self.ay)) #[DEBUG]
        # print("mu len   : ", len(self.tuning_parameter.mu)) #[DEBUG]
        # print(self.racing_line_xy_arr[0].size) # [DEBUG]
        # print(self.v_max.size) # [DEBUG]
        # print(self.v_acc_limit.size) # [DEBUG]
        # print(self.v_dec_limit.size) # [DEBUG]
        
    def limit_local_velocites(self):
        # print('before resizing: ', self.tuning_parameter.ax_upper_limit.size) #[DEBUG]
        # print(self.tuning_parameter.ax_upper_limit) #[DEBUG]
        mu = self.tuning_parameter.mu
        k = self.curvature
        cos_slope = np.cos(self.tuning_parameter.road_slope_rad)
        ay_upper_limit = self.tuning_parameter.ay_upper_limit
        ay_lower_limit = self.tuning_parameter.ay_lower_limit
        # print(mu)
        # print(cos_slope)
        # print(k)
        v_k1 = np.sqrt(mu * g * cos_slope / k)
        v_k2 = np.sqrt(ay_upper_limit / k)
        v_k3 = np.sqrt(ay_lower_limit / k)
        # For safety, keep following sequence!
        v = np.maximum(v_k1, v_k3)
        v = np.minimum(v, v_k2)
        # For an aggressive tuning, follow these steps
        # v = np.minimum(v_k1, v_k2)
        # v = np.maximum(v, v_k3)
        self.v_max = v

        # [DEBUG]
        # for v1, v2, v3, v in zip(v_k1, v_k2, v_k3, v):
        #     print("local max: ", v1, "upper_limit: ", v2, "lower_limit: ", v3, "final: ", v)
        #print('size of k: ', k.size)
        #print('after resizing: ', self.tuning_parameter.ax_upper_limit.size) # [DEBUG]
        #print(self.tuning_parameter.ax_upper_limit) # [DEBUG]

    def limit_acceleration(self):
        k = self.curvature
        # Start at slowest point
        shift = -np.argmin(self.v_max)
        s = np.roll(self.racing_line_samples, shift)
        v = np.roll(self.v_max, shift)
        k = np.roll(k, shift)
        mu = np.roll(self.tuning_parameter.mu, shift)
        Cd = np.roll(self.tuning_parameter.Cd, shift) # (Note) driving command coeff != CD
        road_slope_rad = np.roll(self.tuning_parameter.road_slope_rad, shift)
        s_max = self.max_length_of_closed_track
        m = self.vehicle.mass
        CD = self.vehicle.drag_coefficient # (Note) drag coeff != Cd
        k = self.curvature
        # Limit according to acceleration
        for i in range(s.size):
            index = (i - shift) % s.size # calculate original index (useful)
            wrap = i == (shift % s.size) # check starting point of sample points
            if wrap and not self.closed: continue
            if v[i] > v[i-1]:
                # print(v[i], v[i-1]) # [DEBUG]
                # print(mu, road_slope_rad) # [DEBUG]
                traction = self.vehicle.traction(v[i-1], k[i-1], mu[i-1], road_slope_rad[i-1])
                # print(self.vehicle.engine_d2ax(v[i-1], 1)) # [DEBUG]
                # print("v: ", v[i-1]) # [DEBUG]
                F_engine = Cd[i-1] * m * self.vehicle.engine_d2ax(v[i-1], 1)
                D = CD * (v[i-1] ** 2)
                F_slope = m * g * sin(road_slope_rad[i-1])
                Fx = min(traction, F_engine) - D - F_slope
                if Fx < 0 :
                    Fx = 0 # At least, do not decel
                ax = Fx / m
                ds = s_max - s[i-1] if wrap else s[i] -s[i-1]
                vlim = sqrt(v[i-1]**2 + 2 * ax * ds)
                v[i] = min(v[i], vlim)
                # print("Ft: {:.3f}, Fe: {:.3f}, FD: {:.3f}, Fs: {:.3f}, Fx: {: 3f}".format(traction, F_engine, D, F_slope, Fx))
                # print("ax: {:.3f}".format(ax))
        # Reset shift and return
        self.v_acc_limit = np.roll(v, -shift)
        # for v_acc_limit in self.v_acc_limit:
        #     print(v_acc_limit)

    def limit_deceleration(self):
        # Consider change of slowest point
        slowest_is_changed = True
        self.v_dec_limit = self.v_max
        k = self.curvature
        m = self.vehicle.mass
        CD = self.vehicle.drag_coefficient
        s_max = self.max_length_of_closed_track
        while slowest_is_changed :
            # Start at slowest point
            shift = -np.argmin(self.v_dec_limit)
            # print(shift) # [DEBUG]
            s = np.flip(np.roll(self.racing_line_samples, shift), 0)
            k = np.flip(np.roll(k, shift), 0)
            v = np.flip(np.roll(self.v_dec_limit, shift), 0)
            mu = np.flip(np.roll(self.tuning_parameter.mu, shift), 0)
            road_slope_rad = np.flip(np.roll(self.tuning_parameter.road_slope_rad, shift), 0)
            Cb = np.flip(np.roll(self.tuning_parameter.Cb, shift), 0)
            v_slowest = v[0]
            # Limit according to deceleration
            for i in range(s.size):
                index = (-shift-i-1) % s.size # calculate original index (useful)
                wrap = ((-shift-i-1) % s.size == s.size-1) # check last point of sample points
                # if wrap : print("wrap!!!!") # [DEBUG]
                # print(i) # [DEBUG]
                # print((-shift-i) % s.size) # [DEBUG]
                if wrap and not self.closed : continue
                if v[i] > v[i-1] :
                    traction = self.vehicle.traction(v[i-1], k[i-1], mu[i-1], road_slope_rad[i-1])
                    F_engine = m * Cb[i-1] * -self.vehicle.engine_d2ax(v[i-1], -1) # > 0 (abs)
                    D = CD * (v[i-1] ** 2) # > 0 (abs)
                    F_slope = m * g * sin(road_slope_rad[i-1]) # > 0 (abs)
                    Fx = - min(traction, F_engine) - D + F_slope # < 0 (dec)
                    ax = Fx / m # < 0 (dec)
                    ds = s_max - s[i] if wrap else s[i-1] - s[i]
                    # [DEBUG]
                    # print("Ft: {:.3f}, Fe: {:.3f}, FD: {:.3f}, Fs: {:.3f}, Fx: {: 3f}".format(traction, F_engine, D, F_slope, Fx))
                    # print("ax: {:.3f}".format(ax))
                    # print("ds: {:.3f}".format(ds))
                    # print("vi-1 : {:.3f}".format(v[i-1]))
                    vlim = sqrt(v[i-1]**2 - 2 * ax * ds)
                    v[i] = min(v[i], vlim)
            # Check if slowest point is changed or not
            if v_slowest == v[0] :
                slowest_is_changed = False
            # Reset shift and return
            self.v_dec_limit = np.roll(np.flip(v, 0), -shift)

    def update_profile(self) :
        """ update current racing line's v, ax, ay profile """
        self.v = np.minimum(np.minimum(self.v_max, self.v_acc_limit), self.v_dec_limit)
        v = self.v
        s = self.racing_line_samples
        s_max = self.max_length_of_closed_track
        closed = self.closed
        ds = np.diff(s)
        # print("closed? : ", closed) # [DEBUG]
        if closed :
            ds = np.append(ds, s_max - s[-1])
        v_next = np.roll(v, -1)
        if not closed :
            v = v[:-1]
            v_next = v_next[:-1]
        ax = ((v_next ** 2) - (v ** 2)) / 2 * ds
        # print("ax len b4: ", len(ax)) #[DEBUG]
        if not closed :
            # print("YES") # [DEBUG]
            ax = np.append(ax, 0.0)
        self.ax = ax
        self.ay = self.curvature * (self.v ** 2)
        


