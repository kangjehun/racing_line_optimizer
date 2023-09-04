import numpy as np
import math
import time
import copy
import socket

from spline import Line
from velocity import VelocityProfile
from scipy.optimize import Bounds, minimize
from tqdm import tqdm
from utils import divide_alphas, merge_alphas
from optimization import OptimizationResult, optimizer

import matplotlib.pyplot as plt

class RacingLine:
    """
    Stores the track info and vehicle dynamics,
    handling optimization of the racing line
    spline samples are taken every meter
    """

    def __init__(self, track, vehicle, tuning_parameter):
        """ Store track and vehicle, and initialize a racing line as centerline. """
        self.num_of_update = 0
        self.num_of_update_velocity = 0
        self.track = track
        self.vehicle = vehicle
        self.tuning_parameter = tuning_parameter
        self.update(np.full(track.size, 0), np.full(track.size, 0))
        self.velocity = None
        self.update_velocity()

    def update(self, alphas_left, alphas_right):
        """ Update racing line control points and resulting spline samples. """
        self.alphas_left = alphas_left
        self.alphas_right = alphas_right
        self.racing_line_control_points = self.track.generate_racing_line_control_points(alphas_left, alphas_right)
        self.racing_line = Line(self.racing_line_control_points, self.track.closed)
        # Sample every meter
        self.length = self.racing_line.dists[-1]
        self.ns = math.ceil(self.length)
        self.samples = np.append(np.arange(0, self.ns), self.length)
        self.curvature = self.racing_line.curvature(self.samples)
        self.xy_arr = self.racing_line.position(self.samples)
        self.length_closed = self.racing_line.length if self.track.closed else None
        self.num_of_update += 1
        print("* update racing line", self.num_of_update)

    def update_velocity(self):
        """ Generate a new velocity profile for the current path. """
        racing_line_samples = self.samples
        curvature = self.curvature
        if self.track.closed:
            print("track is closed") # [DEBUG]
            racing_line_samples = racing_line_samples[:-1]
            curvature = curvature[:-1]
        tuning_parameter = copy.deepcopy(self.tuning_parameter)
        # print("tunning_parameter size : ", tuning_parameter.mu.size) # [DEBUG]
        self.velocity = VelocityProfile(self.vehicle, self.xy_arr, racing_line_samples, curvature, tuning_parameter, self.length_closed)
        self.num_of_update_velocity += 1
        print("* update velocity profile", self.num_of_update_velocity)

    def lap_time(self):
        """ Calculate lap time from the velocity profile. """
        if not self.track.closed:
            v = self.velocity.v[:-1]
        return np.sum(np.diff(self.samples) / v)
    
    def minimize_laptime(self):
        """ Find initial alphas using GA Algorithm. """
        # lap-time based cost function
        def costfunc(alphas):
            """ cost function for optimization """
            alphas_left, alphas_right = divide_alphas(alphas)
            self.update(alphas_left, alphas_right)
            self.update_velocity()
            return self.lap_time()
        # curvature-based (gamma2) cost function
        # def costfunc(alphas):
        #     """ cost function for optimization """
        #     alphas_left, alphas_right = divide_alphas(alphas)
        #     self.update(alphas_left, alphas_right)
        #     return self.racing_line.gamma2()
        def getxy(alphas):
            """ get current racingline xy coordinates """
            alphas_left, alphas_right = divide_alphas(alphas)
            self.update(alphas_left, alphas_right)
            self.update_velocity()
            return self.xy_arr 
        def getTrack():
            """ get track boundaries and mid line xy coordinates """
            return self.track.mid_xy_coordinates, self.track.left_boundary, self.track.right_boundary, \
                   self.track.left_opt_boundary, self.track.right_opt_boundary
        t0 = time.time()
        corners, straights = self.track.segments(self.tuning_parameter.k_min, \
                                                 self.tuning_parameter.corner_length_min, \
                                                 self.tuning_parameter.straight_length_min)
        # TODO: should be more generatilized
        start = straights[0][0] + 1
        mid1  = corners[0] + 1
        mid2  = corners[1] + 1
        end   = straights[1][1] + 1
        print(start, mid1, mid2, end) # [DEBUG]
        # run optimization
        opt_res = optimizer(
            costfunc = costfunc,
            getxy = getxy,
            getTrack = getTrack,
            x0 = np.full(self.track.size, 0.0),
            method = 'GA',
            bounds = [-1.0, 1.0],
            start = start,
            mid1 = mid1,
            mid2 = mid2,
            end = end
        ).opt_res
        return opt_res


    #  def objfun(alphas):
    #      """cost function for optimization"""
    #   
    #     
    #      self.update_velocity()
    #      cost = self.lap_time()
    #      return cost
    #  t0 = time.time()
    #  res = minimize(
    #      fun=objfun,
    #      x0=np.full(self.track.size, 0.2),
    #      method='L-BFGS-B',
    #      bounds=Bounds(-1.0, 1.0),
    #      options={'gtol': 1e-6}
    #  )
    #  end = 1
    #  alphas_left = np.maximum(-res.x, 0)
    #  alphas_right = np.maximum(res.x, 0)
    #  self.update(alphas_left, alphas_right)
    #  return (time.time() - t0), res, alphas_left, alphas_right, \
    #         iteration[-1], cost[-1]








##################################################################################
    
    # def generate_initial_alphas(self, init_alphas_left=None, init_alphas_right=None):
    #     """ Generate initial racing line b4 starting lap time optimization. """
    #     iteration = 0
    #     def objfun(alphas):
    #         """cost function for optimization"""
    #         nonlocal iteration
    #         iteration = iteration + 1
    #         alphas_left = np.maximum(-alphas, 0)
    #         alphas_right = np.maximum(alphas, 0)
    #         self.update(alphas_left, alphas_right)
    #         # print("cost : ", cost)
    #         # print("iteration : ", iteration)
    #         return self.racing_line.gamma2()
    #     t0 = time.time()
    #     res = minimize(
    #         fun=objfun,
    #         x0=np.full(self.track.size, 0),
    #         method='L-BFGS-B',
    #         bounds=Bounds(-1.0, 1.0),
    #     )
    #     alphas_left = np.maximum(-res.x, 0)
    #     alphas_right = np.maximum(res.x, 0)
    #     self.update(alphas_left, alphas_right)
    #     cost = self.racing_line.gamma2()
    #     return (time.time() - t0), res, alphas_left, alphas_right, iteration, cost
    










    # def minimise_lap_time(self, iteration, cost, end, init_alphas_left=None, init_alphas_right=None):
    #  """Generate a path that directly minimizes lap time."""
    #  iteration = 0
    #  def objfun(alphas):
    #      """cost function for optimization"""
    #      nonlocal iteration, iteration, cost
    #      iteration += 1
    #      alphas_left = np.maximum(-alphas, 0)
    #      alphas_right = np.maximum(alphas, 0)
    #      self.update(alphas_left, alphas_right)
    #      self.update_velocity()
    #      cost = self.lap_time()
    #      return cost
    #  t0 = time.time()
    #  res = minimize(
    #      fun=objfun,
    #      x0=np.full(self.track.size, 0.2),
    #      method='L-BFGS-B',
    #      bounds=Bounds(-1.0, 1.0),
    #      options={'gtol': 1e-6}
    #  )
    #  end = 1
    #  alphas_left = np.maximum(-res.x, 0)
    #  alphas_right = np.maximum(res.x, 0)
    #  self.update(alphas_left, alphas_right)
    #  return (time.time() - t0), res, alphas_left, alphas_right, \
    #         iteration[-1], cost[-1]
            