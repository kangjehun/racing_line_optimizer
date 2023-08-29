import numpy as np

def is_closed(left, right):
  """
  Compares the first and last cones in each boundary to determine if a track is
  open ended or a closed loop.
  """
  return all(left[:,0]==left[:,-1]) and all(right[:,0]==right[:,-1])

def is_closed(mid):
  """
  Compares the first and last cones in each boundary to determine if a track is
  open ended or a closed loop.
  """
  return all(mid[:,0]==mid[:,-1])
