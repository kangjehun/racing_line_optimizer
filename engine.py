import json
import math
import sys
from scipy.constants import g

class Engine:
  """Model of vehicle engine."""

  def __init__(self, path_engine, wheel_radius, mass):
      """Load engine model"""
      engine_data = json.load(open(path_engine))
      self.type = engine_data["engine_type"]
      self.C1 = float(engine_data["driving_command_coefficient"])
      self.C2 = float(engine_data["driving_command_coefficient2"])
      self.B = float(engine_data["braking_command_coefficient"])
      self.ax_upper_limit_low_speed = float(engine_data["ax_upper_limit_low_speed"])
      self.D = float(engine_data["ax_upper_limit_high_speed_coefficient"])
      self.r = float(engine_data["ax_upper_limit_high_speed_power"])
      self.wheel_radius = wheel_radius
      self.mass = mass
      # print("engine type :", self.type) # [DEBUG]

  def engine_map_to_ax(self, vx, d):
      """Map current velocity and d to ax output by the engine and brake"""
      if self.type == "realcar":
         if 0 <= d <= 1:
            ax = self.C1 * d
            if vx < 10.8:
               ax = min(ax, self.ax_upper_limit_low_speed)
            else:
               ax_upper_limit_high_speed = self.D * math.pow(vx, self.r)
               #   print(ax_upper_limit_high_speed * 2065.63)
               #   print(ax*2065.63)
               ax = min(ax, ax_upper_limit_high_speed)
         elif -1 <= d < 0:
            ax = self.B * d
         else:
            sys.exit("Invalid value of d. Should be in the range of -1 to 1.")
      elif self.type == "rccar":
         if 0 <= d <= 1:
            Fx = self.C1 * vx + self.C2
            if Fx < 0 :
               Fx = 0 # Fx cannot be a negative value
               sys.exit("Fx from engine map cannot be a negative value")
            ax = Fx / self.mass
         elif -1 <= d < 0:
            ax = self.B * d # In fact, self.B is set to 0
         else:
            sys.exit("Invalid value of d. Should be in the range of -1 to 1.")
      return ax

  def engine_map_to_d(self, vx, ax):
      """Map current velocity and ax to d output by the engine and brake"""
      if self.type == "realcar":
         if 0 <= vx < 10.8:
            if ax > self.ax_upper_limit_low_speed:
               d = 1
            elif 0 <= ax <= self.ax_upper_limit_low_speed:
               d = ax / self.C1
            else:
               d = max((ax / self.B), -1) 
         elif 10.8 <= vx:
            ax_upper_limit_high_speed = self.D * math.pow(vx, self.r)
            if ax > ax_upper_limit_high_speed:
               d = 1
            elif 0 <= ax <= ax_upper_limit_high_speed:
               d = ax / self.C1
            else:
               d = max((ax/self.C1), -1)
         else:
            sys.exit("vx is smaller than 0. \
                              Not enough model data for reverse driving")
      elif self.type == "rccar" :
         sys.exit("Obsolete")
      else :
         sys.exit("Obsolete")
      return d