import json
import math

class Engine:
  """Model of vehicle engine."""

  def __init__(self, path):
    """Load engine model"""
    engine_data = json.load(open(path))
    self.B = float(engine_data["braking_command_coefficient"])
    self.C = float(engine_data["driving_command_coefficient"])
    self.ax_upper_limit_low_speed = float(engine_data["ax_upper_limit_low_speed"])
    self.D = float(engine_data["ax_upper_limit_high_speed_coefficient"])
    self.r = float(engine_data["ax_upper_limit_high_speed_power"])
  
  def engine_map_to_ax(self, vx, d):
      """Map current velocity and d to ax output by the engine and brake"""
      if 0 <= d <= 1:
          ax = self.C * d
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
          raise ValueError("Invalid value of d. Should be in the range of -1 to 1.")
      return ax
  
  def engine_map_to_d(self, vx, ax):
    """Map current velocity and ax to d output by the engine and brake"""
    if 0 <= vx < 10.8:
       if ax > self.ax_upper_limit_low_speed:
          d = 1
       elif 0 <= ax <= self.ax_upper_limit_low_speed:
          d = ax / self.C
       else:
          d = max((ax / self.B), -1) 
    elif 10.8 <= vx:
       ax_upper_limit_high_speed = self.D * math.pow(vx, self.r)
       if ax > ax_upper_limit_high_speed:
          d = 1
       elif 0 <= ax <= ax_upper_limit_high_speed:
          d = ax / self.C
       else:
          d = max((ax/self.C), -1)
    else:
       raise ValueError("vx is smaller than 0. Not enough model data for reverse driving")
    return d