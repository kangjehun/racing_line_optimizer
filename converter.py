import matplotlib
import matplotlib.pyplot as plt
import sys
import math
import numpy as np
import csv
import copy

from utm import conversion
from xml.etree.ElementTree import Element, ElementTree, parse
from tqdm import tqdm

def indent(elem, level=0):
    """ Format the XML data by adding appropriate indentations & Line breaks. """
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def convert2osm(dest, samples, profile, alphas_left, alphas_right, \
                lon_0 = 127.20587, lat_0 = 37.29687):
    """
    Convert path to osm format
    # TODO Q. 127.20587 & 37.29687 where?
    """
    utm_x_origin, utm_y_origin, _, _ = conversion.from_latlon(lat_0, lon_0)
    yaw_bias = (0.0) * math.pi / 180 # To adjust the orientation of the path data
    node_id = -237278
    node_id_list = []
    way_id = -99999999 
    root = Element('osm', version='0.6', upload='false', generator='JOSM') # root element of XML file
    p = samples.T.reshape(-1, 1, 2) # sample points
    segments = np.concatenate([p[:-1], p[1:]], axis = 1)
    # profile
    vxs = profile.v
    axs = profile.ax
    ays = profile.ay
    # tuning parameters
    mus = profile.tuning_parameter.mu
    ax_upper_limits = profile.tuning_parameter.ax_upper_limit
    ax_lower_limits = profile.tuning_parameter.ax_lower_limit
    ay_upper_limits = profile.tuning_parameter.ay_upper_limit
    ay_lower_limits = profile.tuning_parameter.ay_lower_limit
    Cbs = profile.tuning_parameter.Cb
    Cds = profile.tuning_parameter.Cd
    alpha_left_maxs = profile.tuning_parameter.alpha_left_max
    alpha_right_maxs = profile.tuning_parameter.alpha_right_max
    road_slope_rads = profile.tuning_parameter.road_slope_rad
    #alphas
    alphas_left_s = alphas_left
    alphas_right_s = alphas_right

    # print("mu size          : ", len(mus)) # [DEBUG]
    # print("ax size          : ", len(axs)) # [DEBUG]
    # print("alphas_left size : ", len(alphas_left_s)) # [DEBUG]

    for segment, vx, ax, ay, mu, ax_upper_limit, ax_lower_limit, \
        ay_upper_limit, ay_lower_limit, Cb, Cd, alpha_left_max, alpha_right_max, \
        road_slope_rad, alpha_left, alpha_right in zip(segments, vxs, axs, ays, \
                              mus, ax_upper_limits, ax_lower_limits, \
                              ay_upper_limits, ay_lower_limits, \
                              Cbs, Cds, alpha_left_maxs, alpha_right_maxs, \
                              road_slope_rads, alphas_left_s, alphas_right_s):
        
        # TODO : CHECK, why skip -237278?
        # if (node_id == -237278):
        #     node_id -= 1
        #     continue

        # local xy -> LLH
        local_x = float(segment[0][0]) * math.cos(yaw_bias) + float(segment[0][1]) * math.sin(yaw_bias)
        local_y = -float(segment[0][0]) * math.sin(yaw_bias) + float(segment[0][1]) * math.cos(yaw_bias)
        utm_x_global = local_x + utm_x_origin
        utm_y_global = local_y + utm_y_origin
        lat, lon = conversion.to_latlon(utm_x_global, utm_y_global, 52, 'N') # TODO Check : UTM Zone 52N
        lat_buf = float(lat)
        lon_buf = float(lon)
        alt_buf = float(140.0) # # TODO Check : 140?
        # alphas of sample points
        alpha_left_buf = float(alpha_left)
        alpha_right_buf = float(alpha_right)
        # optimized profile of sample points
        vx_buf = float(vx) # km/h (auto)
        ax_buf = float(ax) # m/s2
        ay_buf = float(ay) # m/s2     
        # tuning parameters of sample points
        mu_buf = float(mu)
        ax_upper_limit_buf = float(ax_upper_limit)
        ax_lower_limit_buf = float(ax_lower_limit)
        ay_upper_limit_buf = float(ay_upper_limit)
        ay_lower_limit_buf = float(ay_lower_limit)
        Cb_buf = float(Cb)
        Cd_buf = float(Cd)
        alpha_left_max_buf = float(alpha_left_max)
        alpha_right_max_buf = float(alpha_right_max)
        road_slope_rad_buf = float(road_slope_rad)
        # create node; A point of the racing line with profile and tuning parameters
        lat = str(lat_buf)
        lon = str(lon_buf)
        alt = str(alt_buf)
        vx = str(vx_buf)
        ax = str(ax_buf)
        ay = str(ay_buf)
        mu = str(mu_buf)
        ax_upper_limit = str(ax_upper_limit_buf)
        ax_lower_limit = str(ax_lower_limit_buf)
        ay_upper_limit = str(ay_upper_limit_buf)
        ay_lower_limit = str(ay_lower_limit_buf)
        Cb = str(Cb_buf)
        Cd = str(Cd_buf)
        alpha_left_max = str(alpha_left_max_buf)
        alpha_right_max = str(alpha_right_max_buf)
        road_slope_rad = str(road_slope_rad_buf)
        alpha_left = str(alpha_left_buf)
        alpha_right = str(alpha_right_buf)
        # create node
        node = Element('node', id='%d' % node_id, action = 'modify', \
                        lat=lat, lon=lon, alt=alt, \
                        alpha_left=alpha_left, alpha_right=alpha_right)
        tag_speed = Element('tag', k = 'speed', v = vx)
        tag_ax = Element('tag', k = 'ax', v = ax)
        tag_ay = Element('tag', k = 'ay', v = ay)
        tag_mu = Element('tag', k = 'mu', v = mu)
        tag_ax_upper_limit = Element('tag', k = 'ax_upper_limit', v = ax_upper_limit)
        tag_ax_lower_limit = Element('tag', k = 'ax_lower_limit', v = ax_lower_limit)
        tag_ay_upper_limit = Element('tag', k = 'ay_upper_limit', v = ay_upper_limit)
        tag_ay_lower_limit = Element('tag', k = 'ay_lower_limit', v = ay_lower_limit)
        tag_Cb = Element('tag', k = 'Cb', v = Cb)
        tag_Cd = Element('tag', k = 'Cd', v = Cd)
        tag_alpha_left_max = Element('tag', k = 'alpha_left_max', v = alpha_left_max)
        tag_alpha_right_max = Element('tag', k = 'alpha_right_max', v = alpha_right_max)
        tag_road_slope_rad = Element('tag', k = 'road_slope_rad', v = road_slope_rad)
        node.append(tag_speed)
        node.append(tag_ax)
        node.append(tag_ay)
        node.append(tag_mu)
        node.append(tag_ax_upper_limit)
        node.append(tag_ax_lower_limit)
        node.append(tag_ay_upper_limit)
        node.append(tag_ay_lower_limit)
        node.append(tag_Cb)
        node.append(tag_Cd)
        node.append(tag_alpha_left_max)
        node.append(tag_alpha_right_max)
        node.append(tag_road_slope_rad)
        root.append(node)
        node_id_list.append(node_id)
        node_id -= 1
    if profile.closed :
        # print("node_id:", node_id)
        first_node = root[0]  # Get the first node element
        new_node = copy.deepcopy(first_node)  # Create a deep copy of the first node
        new_node.attrib['id'] = str(node_id)
        root.append(new_node)  # Append the new node after the last node
        first_node_id = node_id_list[0]  # Get the ID of the first node
        node_id_list.append(first_node_id)  # Append the ID to the end of the list
    else :
        segment = segments[-1]
        # local xy -> LLH
        local_x = float(segment[1][0]) * math.cos(yaw_bias) + float(segment[1][1]) * math.sin(yaw_bias)
        local_y = -float(segment[1][0]) * math.sin(yaw_bias) + float(segment[1][1]) * math.cos(yaw_bias)
        utm_x_global = local_x + utm_x_origin
        utm_y_global = local_y + utm_y_origin
        lat, lon = conversion.to_latlon(utm_x_global, utm_y_global, 52, 'N') # TODO Check : UTM Zone 52N
        # llh of sample points
        lat_buf = float(lat)
        lon_buf = float(lon)
        alt_buf = float(140.0) # # TODO Check : 140?
        # alphas of sample points
        alpha_left_buf = float(alphas_left[-1])
        alpha_right_buf = float(alpha_right[-1])
        # optimized profile of sample points
        vx_buf = float(vxs[-1]) # km/h (auto)
        ax_buf = float(axs[-1]) # m/s2
        ay_buf = float(ays[-1]) # m/s2
        # tuning parameters of sample points
        mu_buf = float(mus[-1])
        ax_upper_limit_buf = float(ax_upper_limits[-1])
        ax_lower_limit_buf = float(ax_lower_limits[-1])
        ay_upper_limit_buf = float(ay_upper_limits[-1])
        ay_lower_limit_buf = float(ay_lower_limits[-1])
        Cb_buf = float(Cbs[-1])
        Cd_buf = float(Cds[-1])
        alpha_left_max_buf = float(alpha_left_maxs[-1])
        alpha_right_max_buf = float(alpha_right_maxs[-1])
        road_slope_rad_buf = float(road_slope_rads[-1])
        # create node; A point of the racing line with profile and tuning parameters
        # llh
        lat = str(lat_buf)
        lon = str(lon_buf)
        alt = str(alt_buf)
        # profile
        vx = str(vx_buf)
        ax = str(ax_buf)
        ay = str(ay_buf)
        # tuning parameter
        mu = str(mu_buf)
        ax_upper_limit = str(ax_upper_limit_buf)
        ax_lower_limit = str(ax_lower_limit_buf)
        ay_upper_limit = str(ay_upper_limit_buf)
        ay_lower_limit = str(ay_lower_limit_buf)
        Cb = str(Cb_buf)
        Cd = str(Cd_buf)
        alpha_left_max = str(alpha_left_max_buf)
        alpha_right_max = str(alpha_right_max_buf)
        road_slope_rad = str(road_slope_rad_buf)
        alpha_left = str(alpha_left_buf)
        alpha_right = str(alpha_right_buf)
        node = Element('node', id='%d' % node_id, action = 'modify', \
                        lat=lat, lon=lon, alt=alt, \
                        alpha_left=alpha_left, alpha_right=alpha_right)
        tag_speed = Element('tag', k = 'speed', v = vx)
        tag_ax = Element('tag', k = 'ax', v = ax)
        tag_ay = Element('tag', k = 'ay', v = ay)
        tag_mu = Element('tag', k = 'mu', v = mu)
        tag_ax_upper_limit = Element('tag', k = 'ax_upper_limit', v = ax_upper_limit)
        tag_ax_lower_limit = Element('tag', k = 'ax_lower_limit', v = ax_lower_limit)
        tag_ay_upper_limit = Element('tag', k = 'ay_upper_limit', v = ay_upper_limit)
        tag_ay_lower_limit = Element('tag', k = 'ay_lower_limit', v = ay_lower_limit)
        tag_Cb = Element('tag', k = 'Cb', v = Cb)
        tag_Cd = Element('tag', k = 'Cd', v = Cd)
        tag_alpha_left_max = Element('tag', k = 'alpha_left_max', v = alpha_left_max)
        tag_alpha_right_max = Element('tag', k = 'alpha_right_max', v = alpha_right_max)
        tag_road_slope_rad = Element('tag', k = 'road_slope_rad', v = road_slope_rad)
        node.append(tag_speed)
        node.append(tag_ax)
        node.append(tag_ay)
        node.append(tag_mu)
        node.append(tag_ax_upper_limit)
        node.append(tag_ax_lower_limit)
        node.append(tag_ay_upper_limit)
        node.append(tag_ay_lower_limit)
        node.append(tag_Cb)
        node.append(tag_Cd)
        node.append(tag_alpha_left_max)
        node.append(tag_alpha_right_max)
        node.append(tag_road_slope_rad)
        root.append(node)
        node_id_list.append(node_id)    
    # create way; Sequence of node index representing a path
    way = Element('way', id='%d' % way_id, action='modify')
    for node_id_tmp in node_id_list:
        nd = Element('nd', ref='%d' % node_id_tmp)
        way.append(nd)
    root.append(way)

    # apply indent and write osm file
    indent(root)
    tree = ElementTree(root)
    tree.write(dest)
    print("OSM file is generated")

def read_osm_racingline(input_osm_file_name, lon_0 = 127.20587, lat_0 = 37.29687):
    try:
        input_ref_tree = parse(input_osm_file_name)
    except FileNotFoundError:
        print("File not found:", input_osm_file_name)
        return None
    input_ref_root = input_ref_tree.getroot()
    utm_x_origin, utm_y_origin, _, _ = conversion.from_latlon(lat_0, lon_0)
    input_nodes = input_ref_root.findall("node")
    # read xy coordinates of racing line
    utm_xy_arr = np.array([[]])
    # read alphas of racing line
    alpha_left_arr = np.array([])
    alpha_right_arr = np.array([])
    # read profile of racing line
    v_arr = np.array([])
    ax_arr = np.array([])
    ay_arr = np.array([])
    # read tuning parameters of racing line
    mu_arr = np.array([])
    ax_upper_limit_arr = np.array([])
    ax_lower_limit_arr = np.array([])
    ay_upper_limit_arr = np.array([])
    ay_lower_limit_arr = np.array([])
    Cb_arr = np.array([])
    Cd_arr = np.array([])
    alpha_left_max_arr = np.array([])
    alpha_right_max_arr = np.array([])
    road_slope_rad_arr = np.array([])

    print("[ read osm file :", input_osm_file_name, "]")
    for input_node in tqdm(input_nodes):
        # read nodes
        # xy
        lat = float(input_node.attrib['lat'])
        lon = float(input_node.attrib['lon'])
        # alphas
        v  = float(input_node.find("tag[@k='speed']").attrib['v'])
        ax = float(input_node.find("tag[@k='ax']").attrib['v'])
        ay = float(input_node.find("tag[@k='ay']").attrib['v'])
        mu = float(input_node.find("tag[@k='mu']").attrib['v'])
        ax_upper_limit = float(input_node.find("tag[@k='ax_upper_limit']").attrib['v'])
        ax_lower_limit = float(input_node.find("tag[@k='ax_lower_limit']").attrib['v'])
        ay_upper_limit = float(input_node.find("tag[@k='ay_upper_limit']").attrib['v'])
        ay_lower_limit = float(input_node.find("tag[@k='ay_lower_limit']").attrib['v'])
        Cb = float(input_node.find("tag[@k='Cb']").attrib['v'])
        Cd = float(input_node.find("tag[@k='Cd']").attrib['v'])
        alpha_left_max = float(input_node.find("tag[@k='alpha_left_max']").attrib['v'])
        alpha_right_max = float(input_node.find("tag[@k='alpha_right_max']").attrib['v'])
        road_slope_rad = float(input_node.find("tag[@k='road_slope_rad']").attrib['v'])
        # vel_gain = float(input_node.find("tag[@k='vel_profile_gain']").attrib['v']
        utm_x, utm_y, _, _ = conversion.from_latlon(lat, lon)
        utm_x = utm_x - utm_x_origin
        utm_y = utm_y - utm_y_origin
        utm_xy = np.array([[utm_x, utm_y]])
        # create array
        v = np.array([v])
        ax = np.array([ax])
        ay = np.array([ay])
        mu = np.array([mu])
        ax_upper_limit = np.array([ax_upper_limit])
        ax_lower_limit = np.array([ax_lower_limit])
        ay_upper_limit = np.array([ay_upper_limit])
        ay_lower_limit = np.array([ay_lower_limit])
        Cb = np.array([Cb])
        Cd = np.array([Cd])
        alpha_left_max = np.array([alpha_left_max])
        alpha_right_max = np.array([alpha_right_max])
        road_slope_rad = np.array([road_slope_rad])
        # append
        utm_xy_arr = np.append(utm_xy_arr, utm_xy)
        v_arr = np.append(v_arr, v)
        ax_arr = np.append(ax_arr, ax)
        ay_arr = np.append(ay_arr, ay)
        # read tuning parameters of racing line
        mu_arr = np.append(mu_arr, mu)
        ax_upper_limit_arr = np.append(ax_upper_limit_arr, ax_upper_limit)
        ax_lower_limit_arr = np.append(ay_lower_limit_arr, ay_lower_limit)
        ay_upper_limit_arr = np.append(ay_upper_limit_arr, ay_upper_limit)
        ay_lower_limit_arr = np.append(ay_lower_limit_arr, ay_lower_limit)
        Cb_arr = np.append(Cb_arr, Cb)
        Cd_arr = np.append(Cd_arr, Cd)
        alpha_left_max_arr = np.append(alpha_left_max_arr, alpha_left_max)
        alpha_right_max_arr = np.append(alpha_right_max_arr, alpha_right_max)
        road_slope_rad_arr = np.append(road_slope_rad_arr, road_slope_rad)

    utm_xy_arr = utm_xy_arr.reshape(-1, 2)
    utm_xy_arr = utm_xy_arr.transpose()
    # print(utm_xy_arr) # [DEBUG]
    return utm_xy_arr, v_arr, ax_arr, ay_arr, mu_arr, \
           ax_upper_limit_arr, ax_lower_limit_arr, ay_upper_limit_arr, ay_lower_limit_arr, \
           Cb_arr, Cd_arr, alpha_left_max_arr, alpha_right_max_arr, road_slope_rad_arr

def read_csv_best_alphas(input_csv_best_alphas_file_name):
    """ read csv best alphas data. """
    try:
        alphas_left = []
        alphas_right = []
        with open(input_csv_best_alphas_file_name, 'r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                alphas_left.append(float(row['alphas_left']))
                alphas_right.append(float(row['alphas_right']))
        alphas_left = np.array(alphas_left)
        alphas_right = np.array(alphas_right)
        return alphas_left, alphas_right
    except Exception as e:
        print("Warning: ", e)
        return None, None
def read_csv_track(input_csv_track_file_name):
    """ read csv tack data. """
    try:
        x = []
        y = []
        wl = []
        wr = []
        slope = []
        friction_coefficient = []
        alphas_left = []
        alphas_right = []
        
        with open(input_csv_track_file_name, 'r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                x.append(float(row['x']))
                y.append(float(row['y']))
                wl.append(float(row['wl']))
                wr.append(float(row['wr']))
                if 'slope' in row:
                    slope.append(float(row['slope']))
                else:
                    slope.append(-1)
                if 'friction_coefficient' in row:
                    friction_coefficient.append(float(row['friction_coefficient']))
                else:
                    friction_coefficient.append(-1)
                
        xy_arr = np.array([x, y])
        wl_arr = np.array(wl)
        wr_arr = np.array(wr)
        slope_arr = np.array(slope)
        mu_arr = np.array(friction_coefficient)
        
        return xy_arr, wl_arr, wr_arr, slope_arr, mu_arr
    
    except Exception as e:
        print("Error:", e)
        return None, None, None, None, None

def xy2xy(v_arr, src_xy_arr, src_is_closed, dest_xy_arr, dest_is_closed):
    """ Convert v_arr from src_xy_arr size to dest_xy_arr size"""
    src_size = len(src_xy_arr[0]) - int(src_is_closed)
    dest_size = len(dest_xy_arr[0]) - int(dest_is_closed)
    # print(v_arr) # [DEBUG]
    # print("track :", src_xy_arr) #[DEBUG]
    # print("spline:", dest_xy_arr) #[DEBUG]
    # v_arr_size = len(v_arr) - int(src_is_closed) #[DEBUG]
    # print("v_arr_size:", v_arr.size) #[DEBUG]
    # print("src size:", src_size) #[DEBUG]
    # print("dest_size:", dest_size) #[DEBUG]
    # print("src_is_closed:", src_is_closed) #[DEBUG]
    v_arr_converted = np.empty(dest_size)  

    min_distance = float('inf')
    distance = float('inf')
    closest_src_point = None
    for j in range(src_size):
        distance = np.linalg.norm(dest_xy_arr[:,0] - src_xy_arr[:,j])
        if distance < min_distance:
            min_distance = distance
            closest_src_point = j
            # print(closest_src_point) #[DEBUG]
    v_arr_converted[0] = v_arr[closest_src_point]
    for i in range(1, dest_size):
        min_distance = float('inf')
        is_updated = False
        # print(">i:", i) #[DEBUG]
        for j in range(src_size if src_is_closed else (src_size - closest_src_point)):
            idx_current = (closest_src_point + j) % src_size
            # print("closed loop count : ", src_size) #[DEBUG]
            # print("j: ", j) #[DEBUG]
            # print("idx_current: ", idx_current) #[DEBUG]
            # print("dest_size : ", dest_size) #[DEBUG]
            # print("open loop count", src_size - closest_src_point) #[DEBUG]
            distance = np.linalg.norm(dest_xy_arr[:, i] - src_xy_arr[:, idx_current])
            if distance < min_distance:
                min_distance = distance
                closest_src_point = idx_current
            else:
                # print("closest point: ", closest_src_point) #[DEBUG]
                v_arr_converted[i] = v_arr[closest_src_point]
                is_updated = True
                break
        if not is_updated : v_arr_converted[i] = v_arr[closest_src_point]
    return v_arr_converted


def divide_alphas(alphas):
    """divide alphas [-1, 1] to alphas_left and alphas_right [0, 1]"""
    alphas_left = np.zeros_like(alphas)
    alphas_right = np.zeros_like(alphas)
    for i, alpha in enumerate(alphas):
        if -1 <= alpha <= 0:
            alphas_left[i]  = abs(alpha)
            alphas_right[i] = 0
        elif 0 < alpha <= 1:
            alphas_left[i]  = 0
            alphas_right[i] = alpha
        else:
            raise ValueError("Elements in alphas must be within the range [-1, 1]")
    return alphas_left, alphas_right


def merge_alphas(alphas_left, alphas_right):
    if not all(0 <= alpha_left <= 1 for alpha_left in alphas_left) \
       or not all(0 <= alpha_right <= 1 for alpha_right in alphas_right):
        raise ValueError("All elements in alphas_left and alphas_right must be within the range [0, 1]")
    if alphas_left.shape != alphas_right.shape:
        raise ValueError("alphas_left and alphas_right must have the same size")
    for alpha_left, alpha_right in zip(alphas_left, alphas_right):
        if alpha_left != 0 and alpha_right != 0:
            raise ValueError("Both alphas_left and alphas_right cannot have non-zero values for the same index")
    alphas = alphas_left * -1 + alphas_right
    return alphas

def normalize_alphas(alphas, alpha_min=-1, alpha_max=1):
    return (alphas-alpha_min)/(alpha_max-alpha_min)

def reverse_normalize_alphas(norm_alphas, alpha_min=-1, alpha_max=1):
    return (norm_alphas * (alpha_max - alpha_min)) + alpha_min