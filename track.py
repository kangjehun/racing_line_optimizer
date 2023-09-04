import numpy as np
import math
import matplotlib.pyplot as plt


from utils import is_closed
from spline import Line

class Track:
    def __init__(self, mid_xy_coordinates, left_widths, right_widths, \
                 road_slope, friction_coefficient, \
                 alpha_left_max, alpha_right_max):
        self.mid_xy_coordinates = mid_xy_coordinates
        self.left_widths = left_widths
        self.right_widths = right_widths
        self.road_slope = road_slope
        self.friction_coefficient = friction_coefficient
        self.is_closed()
        self.size = self.mid_xy_coordinates[0].size - int(self.closed)
        self.num_of_segments = self.mid_xy_coordinates[0].size - 1
        self.alpha_left_max = alpha_left_max
        self.alpha_right_max = alpha_right_max
        self.generate_boundaries()
        # print(self.mid_xy_coordinates)
        # print(self.closed)
        self.mid_spline = Line(self.mid_xy_coordinates, self.closed)
        self.mid_length = self.mid_spline.dists[-1]
        self.mid_ns = math.ceil(self.mid_length) + 1 # number of samples of spline
        self.mid_s = np.append(np.arange(0, self.mid_ns - 1), self.mid_length)

    def is_closed(self):
        """ 
        Compares the first and last nodes of mid line of the track
        to determine if a track is open ended or a closed loop 
        """      
        self.closed = is_closed(self.mid_xy_coordinates)

    def generate_boundaries(self):
        """Generate xy coordinates of left & right boundaries of track with mid line and widths"""
        # Check if the input sizes match
        num_left_widths = len(self.left_widths) - int(self.closed)
        num_right_widths = len(self.right_widths) - int(self.closed)
        # Compare the num values
        if self.size != num_left_widths or self.size != num_right_widths:
           raise ValueError("Input sizes do not match")
        self.left_boundary = []
        self.left_opt_boundary = []
        self.right_boundary = []
        self.right_opt_boundary = []
        self.mid_line = []

        for i in range(self.num_of_segments):
            x1, y1 = self.mid_xy_coordinates[:, i]
            x2, y2 = self.mid_xy_coordinates[:, i + 1]
            
            dx, dy = x2 - x1, y2 - y1
            segment_length = np.sqrt(dx**2 + dy**2)
            normal_vector = np.array([-dy, dx]) / segment_length

            left_width = self.left_widths[i]
            right_width = self.right_widths[i]

            left_point = np.array([x1, y1]) + left_width * normal_vector
            left_opt_point = np.array([x1, y1]) + left_width * self.alpha_left_max[i] * normal_vector
            mid = np.array([x1, y1])
            right_point = np.array([x1, y1]) - right_width * normal_vector
            right_opt_point = np.array([x1, y1]) - right_width * self.alpha_right_max[i] * normal_vector

            self.left_boundary.append(left_point)
            self.left_opt_boundary.append(left_opt_point)
            self.mid_line.append(mid)
            self.right_opt_boundary.append(right_opt_point)
            self.right_boundary.append(right_point)
        
        if not self.closed : # extend the boundaries if open track
            x1, y1 = self.mid_xy_coordinates[:,-2]
            x2, y2 = self.mid_xy_coordinates[:,-1]
            dx, dy = x2 - x1, y2 - y1
            segment_length = np.sqrt(dx ** 2 + dy ** 2)
            normal_vector = np.array([-dy, dx]) / segment_length
            left_width = self.left_widths[-1]
            right_width = self.right_widths[-1]
            left_point = np.array([x2, y2]) + left_width * normal_vector
            left_opt_point = np.array([x2, y2]) + left_width * self.alpha_left_max[i] * normal_vector
            mid = np.array([x2, y2])
            right_point = np.array([x2, y2]) - right_width * normal_vector
            right_opt_point = np.array([x2, y2]) - right_width * self.alpha_right_max[i] * normal_vector

            self.left_boundary.append(left_point)
            self.left_opt_boundary.append(left_opt_point)
            self.mid_line.append(mid)
            self.right_opt_boundary.append(right_opt_point)
            self.right_boundary.append(right_point)

    def filter_corners(self, is_corner, dists, straight_length_min, corner_length_min):
        """Update corner status according to length limits"""
        # Shift to avoid splitting a straight or corner
        # check if there is transition point
        if self.closed :
           is_corner = is_corner[:-1]
        if not np.argwhere(is_corner != is_corner[0]).size == 0 :
            shift = np.argwhere(is_corner != is_corner[0])[0][0]
            is_corner = np.roll(is_corner, -shift)
            # Remove short straights
            start = 0
            is_updated = True
            for i in range(1, is_corner.size):
                if is_corner[i-1]:
                    if not is_corner[i]:
                        # Corner to straight, record straight start
                        start = i
                        is_updated = False
                elif is_corner[i]:
                    # Straight to corner, measure staright and convert if too short
                    is_corner[start:i] = (dists[i] - dists[start]) < straight_length_min
                    is_updated = True
            if not is_updated :
               is_corner[start:is_corner.size] = (dists[-1] - dists[start] < straight_length_min)
               is_updated = True
            # Remove short corners
            start = 0
            for i in range(1, is_corner.size):
                if is_corner[i-1]:
                    if not is_corner[i]:
                        # Corner to straight, measure corner and convert if too short
                        is_corner[start:i] = (dists[i] - dists[start]) > corner_length_min
                        is_updated = True
                elif is_corner[i]:
                    # Straight to corner, record corner start
                    start = i
                    is_updated = False
            if not is_updated :
               is_corner[start:is_corner.size] = (dists[-1] - dists[start]) > corner_length_min
            is_corner = np.roll(is_corner, shift)
        if self.closed :
           is_corner = np.append(is_corner, is_corner[-1])
        return is_corner
    
    def segments_idxs(self, is_corner):
        """ Determine the samples at which corner sequences start and end. """
        corners = np.array([], dtype=int)
        straights = np.array([], dtype=int)
        # check if there is transition point
        if np.argwhere(is_corner != is_corner[0]).size == 0 :
           if is_corner[0] == True : # corner only
              corners = np.array([0, is_corner.size - 1])
           else : # straight only
              straights = np.array([0, is_corner.size - 1])
        else :
            is_straight = ~is_corner
            if self.closed : # Track is closed
                # Shift to avoid splitting a straight or corner
                shift = np.argwhere(is_corner != is_corner[0])[0][0]
                is_corner = np.roll(is_corner, -shift)
                is_straight = np.roll(is_straight, -shift)
                # Search for corners
                n = is_corner.size # = mid_ns = number of samples
                start = shift
                for j in range(1, n+1):
                  i = j % n
                  if is_corner[i-1]:
                    if not is_corner[i]: # Corner -> straight
                      end = (i + shift) % n
                      if len(corners) > 0: corners = np.vstack((corners, [start, end]))
                      else: corners = np.array([start, end])
                  else:
                    if is_corner[i]: # Straight -> corner
                      start = (i + shift) % n
                # Search for straights
                start = shift
                for j in range(1, n+1):
                  i = j % n
                  if is_straight[i-1]:
                    if not is_straight[i]: # Straight -> Corner
                      end = (i + shift) % n
                      if len(straights) > 0: straights = np.vstack((straights, [start, end]))
                      else: straights = np.array([start, end])
                  else:
                    if is_straight[i]: # Corner -> Straight
                      start = (i + shift) % n
            else : # Track is open
                # Search for corners
                n = is_corner.size # = mid_ns = number of samples
                start = 0
                is_updated = False
                for i in range(1, n):
                  if is_corner[i-1]:
                    if not is_corner[i]: # Corner -> straight
                      end = i
                      if len(corners) > 0: corners = np.vstack((corners, [start, end]))
                      else: corners = np.array([start, end])
                      is_updated = True
                  else:
                    if is_corner[i]: # Straight -> corner
                      start = i
                      is_updated = False
                if is_updated == False :
                    end = n-1
                    if len(corners) > 0: corners = np.vstack((corners, [start, end]))
                    else: corners = np.array([start, end])
                    is_updated = True
                # Search for straights
                start = 0
                for i in range(1, n):
                  if is_straight[i-1]:
                    if not is_straight[i]: # Straight -> Corner
                      end = i
                      if len(straights) > 0: straights = np.vstack((straights, [start, end]))
                      else: straights = np.array([start, end])
                      is_updated = True
                  else:
                    if is_straight[i]: # Corner -> Straight
                      start = i
                      is_updated = False
                if is_updated == False:
                   end = n-1
                   if len(straights) > 0: straights = np.vstack((straights, [start, end]))
                   else: straights = np.array([start, end])
                   is_updated = True
        return corners, straights

    def samples_to_controls(self, s_dist, s_idx, c_dist):
        """Convert sample distances to control point indices"""
        n = s_idx.size
        s_flat = s_idx.ravel()
        c_flat = np.zeros(n, dtype=int)
        for i in range(n):
          j = 0
          while s_dist[s_flat[i]] > c_dist[j]: j += 1
          c_flat[i] = j
        return c_flat.reshape(s_idx.shape)

    def segments(self, k_min, corner_length_min, straight_length_min):
        """
        Analyse the track to find segments (corners and straights).
        
        k_min: defines the minimum curvature for a corner
        proximity: corners within this distance are joined
        length: corners must exceed this length to be accepted
        
        return: an array of control point index pairs 
        defining the track's corners and straights
        """

        is_corner = self.mid_spline.curvature(self.mid_s) > k_min
        # print(is_corner) # [DEBUG]
        is_corner = self.filter_corners(is_corner, self.mid_s, \
                                        straight_length_min, corner_length_min)
        s_corners, s_straights = self.segments_idxs(is_corner)
        corners = self.samples_to_controls(self.mid_s, s_corners, self.mid_spline.dists)
        straights = self.samples_to_controls(self.mid_s, s_straights, self.mid_spline.dists)
        return corners, straights
    
    def generate_racing_line_control_points(self, alphas_left, alphas_right):
        """ Generate racing line control points with given alpha values. """
        # Check if the input sizes match
        num_alphas_left = len(alphas_left)
        num_alphas_right = len(alphas_right)

        # Compare the num values
        if self.size != num_alphas_left or self.size != num_alphas_right:
           raise ValueError("Input sizes do not match")
        
        # Limit alpha values to the specified range defined in the track
        for i in range(self.size):
            alphas_left[i] = np.clip(alphas_left[i], 0, self.alpha_left_max[i])
            alphas_right[i] = np.clip(alphas_right[i], 0, self.alpha_right_max[i])

        # Initialize racing_line_control_points
        racing_line_control_points = np.empty_like(self.mid_xy_coordinates)

        for i in range(self.num_of_segments):
            x1, y1 = self.mid_xy_coordinates[:, i]
            x2, y2 = self.mid_xy_coordinates[:, i + 1]

            dx, dy = x2 - x1, y2 - y1
            segment_length = np.sqrt(dx**2 + dy**2)
            normal_vector = np.array([-dy, dx]) / segment_length

            if alphas_left[i] != 0 and alphas_right[i] != 0:
                print(alphas_left[i], alphas_right[i])
                raise ValueError(f"Both left and right alpha values are not zero at index {i}")
            elif alphas_left[i] == 0 and alphas_right[i] == 0:
                width = 0
            elif alphas_left[i] == 0:
                width = -self.right_widths[i] * alphas_right[i]
            else:
                width = self.left_widths[i] * alphas_left[i]
                
            racing_line_control_point = np.array([x1, y1]) + width * normal_vector
            racing_line_control_points[:, i] = racing_line_control_point

        if self.closed:
           racing_line_control_points[:, -1] = racing_line_control_points[:, 0]
        else:
           x1, y1 = self.mid_xy_coordinates[:, -1]
           racing_line_control_point = np.array([x1, y1]) + width * normal_vector # use previous width and normal vector
           racing_line_control_points[:, -1] = racing_line_control_point
        return racing_line_control_points
