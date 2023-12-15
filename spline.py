import numpy as np

from scipy.interpolate import splev, splprep

class Line:
  """Wrapper for scipy.interpolate.BSpline."""

  def __init__(self, control_points, closed):
    """Construct a spline through the given control points."""
    self.control_points = control_points
    self.closed = closed
    self.dists = cumulative_distances(control_points)
    self.spline, _ = splprep(control_points, u=self.dists, k=3, s=0, per=self.closed)
    self.length = self.dists[-1]

  def position(self, s=None):
    """Returns x-y coordinates of sample points."""
    if s is None: return self.control_points
    x, y = splev(s, self.spline)
    return np.array([x, y])

  def curvature(self, s=None):
    """Returns sample curvatures, Kappa."""
    if s is None: s = self.dists
    dx, dy = splev(s, self.spline, 1)
    ddx, ddy = splev(s, self.spline, 2)
    kappa = np.abs((dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2))
    return kappa

  def gamma2(self, s=None):
    """Returns the sum of the squares of sample curvatures, Gamma^2."""
    if s is None: s = self.dists
    kappa = self.curvature(s)
    return np.sum(kappa ** 2)

def cumulative_distances(points):
  """Returns the cumulative linear distance at each point."""
  d = np.cumsum(np.linalg.norm(np.diff(points, axis=1), axis=0))
  return np.append(0, d)
