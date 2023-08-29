import numpy as np
import json

from scipy.constants import g
from engine import Engine
from math import sqrt, cos


###############################################################################

class Vehicle:
  """Vehicle parameters and behaviour."""

  def __init__(self, path_vehicle, path_engine):
    """Load vehicle data from JSON file."""
    vehicle_data = json.load(open(path_vehicle))
    self.name = vehicle_data["name"]
    self.mass = float(vehicle_data["mass"])
    self.drag_coefficient = float(vehicle_data["drag_coefficient"])
    self.engine = Engine(path_engine)
    print("[ {} is ready. ]\n".format(self.name))

  def engine_d2ax(self, velocity, d):
    """Map current velocity and d to ax output by the engine and brake"""
    return self.engine.engine_map_to_ax(velocity, d)
  
  def engine_ax2d(self, velocity, ax):
    """Map current velocity and ax to d output by the engine and brake"""
    """Not Used"""
    pass

  def traction(self, velocity, curvature, friction_coefficient, road_slope_rad):
    """Determine remaining traction when negotiating a corner."""
    v  = velocity
    k  = curvature
    mu = friction_coefficient
    cos_theta = cos(road_slope_rad)
    # print(cos_theta) # [DEBUG]
    
    # Calculate traction
    f = mu * self.mass * g * cos_theta
    f_lat = self.mass * k * (v ** 2)
    if f <= f_lat: return 0.0
    return sqrt(f**2 - f_lat**2)

