import numpy as np
import json
import sys

from utils import is_closed
from utils import xy2xy

###############################################################################

class TuningParameter:
    """
    Tunning parameters read from real racing data
    size : number of nodes in real racing data
    mu   : friction coefficient, overall speed change
    ax_upper_limit : max acc > 0,   depends on gg-diagram, set maximum lateral acc
    ax_lower_limit : max dec < 0,   depends on gg-diagram, set minimum lateral acc
    ay_upper_limit : max ay > 0,    depends on gg-diagram, set maximum lateral acc
    ay_lower_limit : min ay >= 0,   depends on gg-diagram, set minimum lateral acc
    TODO: (Change) For safety, ay_upper_limit is applied after ay_lower_limit & use clipping
    Cb   : adjust braking intensity, pulling in or extending braking point
    Cd   : adjust acceleration intensity, pulling in or extending exit point
    alpha_left_max  : left boundary constraint
    alpha_right_max : right boundary constraint
    road_slope_rad  : reflect the road slope effect
    """

    def __init__(self, mid_xy_arr, path_param_corner=None, path_param_default=None):
        """Tuning parameters"""
        # Read params
        if path_param_corner :
            param_corner = json.load(open(path_param_corner))
            self.k_min = param_corner["K_MIN"]
            self.straight_length_min = param_corner["STRAIGHT_LENGTH_MIN"]
            self.corner_length_min = param_corner["CORNER_LENGTH_MIN"]
            if self.straight_length_min < 1 :
                print("[Warning] STRAIGHT_LENGTH_MIN is smaller than 1m. That is too small!")
            if self.corner_length_min < 1 :
                print("[Warning] CORNER_LENGTH_MIN is smaller than 1m. That is too small!")
        else :
            print("Warning : There is no path_param_corner, set default values")
            self.k_min = 0.020
            self.straight_length_min = 10
            self.corner_length_min = 20
        if path_param_default :
            param_default = json.load(open(path_param_default))
            self.default_mu = param_default["mu"]
            self.default_ax_upper_limit = param_default["ax_upper_limit"]
            self.default_ax_lower_limit = param_default["ax_lower_limit"]
            self.default_ay_upper_limit = param_default["ay_upper_limit"]
            self.default_ay_lower_limit = param_default["ay_lower_limit"]
            self.default_Cb = param_default["Cb"]
            self.default_Cd = param_default["Cd"]
            self.default_alpha_left_max = param_default["alpha_left_max"]
            self.default_alpha_right_max = param_default["alpha_right_max"]
            self.default_road_slope_rad = param_default["road_slope_rad"]
        else :
            print("Warning : There is no path_param_default, set default values")
            self.default_mu = 1
            self.default_ax_upper_limit = 10
            self.default_ax_lower_limit = -10
            self.default_ay_upper_limit = 10
            self.default_ay_lower_limit = 0
            self.default_Cb = 0.87
            self.default_Cd = 2.0
            self.default_alpha_left_max = 0.9
            self.default_alpha_right_max = 0.9
            self.default_road_slope_rad = 0.0  
        # create tuning params
        self.xy = mid_xy_arr
        self.closed = is_closed(mid_xy_arr)
        self.size = mid_xy_arr[0].size - int(self.closed)
        # tuning parameters
        self.mu = np.full(self.size, -1, dtype=float) #
        self.ax_upper_limit = np.full(self.size, -1, dtype=float) # TODO
        self.ax_lower_limit = np.full(self.size, -1, dtype=float) # TODO
        self.ay_upper_limit = np.full(self.size, -1, dtype=float) #
        self.ay_lower_limit = np.full(self.size, -1, dtype=float) #
        self.Cb = np.full(self.size, -1, dtype=float) #
        self.Cd = np.full(self.size, -1, dtype=float) #
        self.alpha_left_max = np.full(self.size, -1, dtype=float) #
        self.alpha_right_max = np.full(self.size, -1, dtype=float) #
        self.road_slope_rad = np.full(self.size, -1, dtype=float) #

    def update_with_data(self, mu=None, ax_upper_limit=None, ax_lower_limit=None,
               ay_upper_limit=None, ay_lower_limit=None, Cb=None, Cd=None,
               alpha_left_max=None, alpha_right_max=None, road_slope_rad=None):
        """
        Update the parameters with provided values, if value is not None and not equal to -1.
        Clip each value to its specified range.
        """
        if mu is not None:
            if self.closed:
                mu = mu[:-1]
            self.mu[mu != -1] = np.clip(mu[mu != -1], 0, 5)
        if ax_upper_limit is not None:
            if self.closed:
                ax_upper_limit = ax_upper_limit[:-1]
            self.ax_upper_limit[ax_upper_limit != -1] = np.clip(ax_upper_limit[ax_upper_limit != -1], 0, 10)
        if ax_lower_limit is not None:
            if self.closed:
                ax_lower_limit = ax_lower_limit[:-1]
            self.ax_lower_limit[ax_lower_limit != -1] = np.clip(ax_lower_limit[ax_lower_limit != -1], -10, 0)
        if ay_upper_limit is not None:
            if self.closed:
                ay_upper_limit = ay_upper_limit[:-1]
            self.ay_upper_limit[ay_upper_limit != -1] = np.clip(ay_upper_limit[ay_upper_limit != -1], 0, 10)
        if ay_lower_limit is not None:
            if self.closed:
                ay_lower_limit = ay_lower_limit[:-1]
            self.ay_lower_limit[ay_lower_limit != -1] = np.clip(ay_lower_limit[ay_lower_limit != -1], 0, 10)
        if Cb is not None:
            if self.closed:
                Cb = Cb[:-1]
            self.Cb[Cb != -1] = np.clip(Cb[Cb != -1], 0, 2)
        if Cd is not None:
            if self.closed:
                Cd = Cd[:-1]
            self.Cd[Cd != -1] = np.clip(Cd[Cd != -1], 0, 2)
        if alpha_left_max is not None:
            if self.closed:
                alpha_left_max = alpha_left_max[:-1]
            self.alpha_left_max[alpha_left_max != -1] = np.clip(alpha_left_max[alpha_left_max != -1], 0, 1)
        if alpha_right_max is not None:
            if self.closed:
                alpha_right_max = alpha_right_max[:-1]
            self.alpha_right_max[alpha_right_max != -1] = np.clip(alpha_right_max[alpha_right_max != -1], 0, 1)
        if road_slope_rad is not None:
            if self.closed:
                road_slope_rad = road_slope_rad[:-1]
            self.road_slope_rad[road_slope_rad != -1] = np.clip(road_slope_rad[road_slope_rad != -1], -1.5, 1.5)

    def update_default(self):
        """
        Update each array's values with default values if they are -1.
        """
        self.mu[self.mu == -1] = self.default_mu
        self.ax_upper_limit[self.ax_upper_limit == -1] = self.default_ax_upper_limit
        self.ax_lower_limit[self.ax_lower_limit == -1] = self.default_ax_lower_limit
        self.ay_upper_limit[self.ay_upper_limit == -1] = self.default_ay_upper_limit
        self.ay_lower_limit[self.ay_lower_limit == -1] = self.default_ay_lower_limit
        self.Cb[self.Cb == -1] = self.default_Cb
        self.Cd[self.Cd == -1] = self.default_Cd
        self.alpha_left_max[self.alpha_left_max == -1] = self.default_alpha_left_max
        self.alpha_right_max[self.alpha_right_max == -1] = self.default_alpha_right_max
        self.road_slope_rad[self.road_slope_rad == -1] = self.default_road_slope_rad

    def resize_with_sample(self, dest_xy_arr, dest_is_closed):
        "resize parameters from track control point size to spline sample size"
        # print(dest_is_closed) # [DEBUG]
        self.mu = xy2xy(self.mu, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        # print(self.mu) # [DEBUG]
        # sys.exit("DEBUG") # [DEBUG]
        self.ax_upper_limit = xy2xy(self.ax_upper_limit, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        self.ax_lower_limit = xy2xy(self.ax_lower_limit, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        self.ay_upper_limit = xy2xy(self.ay_upper_limit, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        self.ay_lower_limit = xy2xy(self.ay_lower_limit, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        self.Cb = xy2xy(self.Cb, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        self.Cd = xy2xy(self.Cd, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        self.alpha_left_max = xy2xy(self.alpha_left_max, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        self.alpha_right_max = xy2xy(self.alpha_right_max, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        self.road_slope_rad = xy2xy(self.road_slope_rad, self.xy, self.closed, dest_xy_arr, dest_is_closed)
        # print("converted parameter size: ", self.mu.size) #[DEBUG]
