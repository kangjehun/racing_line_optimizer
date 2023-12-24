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
    """
    utm_x_origin, utm_y_origin, _, _ = conversion.from_latlon(lat_0, lon_0)
    yaw_bias = (0.0) * math.pi / 180 # To adjust the orientation of the path data
    node_id = -237278
    node_id_list = []
    way_id = -99999999 
    root = Element('osm', version='0.6', upload='false', generator='JOSM')
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
        local_x = float(segment[0][0]) * math.cos(yaw_bias)\
                  + float(segment[0][1]) * math.sin(yaw_bias)
        local_y = -float(segment[0][0]) * math.sin(yaw_bias)\
                  + float(segment[0][1]) * math.cos(yaw_bias)
        # utm_x_global = local_x + utm_x_origin
        # utm_y_global = local_y + utm_y_origin
        # lat, lon = conversion.to_latlon(utm_x_global, utm_y_global, 52, 'N') # TODO Check : 52N
        # TODO : Use local xy instead of LLH
        lat = local_x
        lon = local_y
        lat_buf = float(lat)
        lon_buf = float(lon)
        # alt_buf = float(140.0) # TODO Check : 140?
        alt_buf = float(0.0) # TODO : Set altitude as 0
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
        local_x = float(segment[1][0]) * math.cos(yaw_bias)\
                  + float(segment[1][1]) * math.sin(yaw_bias)
        local_y = -float(segment[1][0]) * math.sin(yaw_bias)\
                  + float(segment[1][1]) * math.cos(yaw_bias)
        # TODO : use local_xy instead of LLH
        # utm_x_global = local_x + utm_x_origin
        # utm_y_global = local_y + utm_y_origin
        # lat, lon = conversion.to_latlon(utm_x_global, utm_y_global, 52, 'N') # TODO Check : 52N
        lat = local_x
        lon = local_y
        # llh of sample points
        lat_buf = float(lat)
        lon_buf = float(lon)
        # alt_buf = float(140.0) # # TODO Check : 140?
        alt_buf = float(0.0) # TODO: set altitude as 0
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
        # TODO : Use local xy instead of LLH
        # utm_x, utm_y, _, _ = conversion.from_latlon(lat, lon)
        # utm_x = utm_x - utm_x_origin
        # utm_y = utm_y - utm_y_origin
        utm_x = lat
        utm_y = lon
        # profiles
        v  = float(input_node.find("tag[@k='speed']").attrib['v'])
        ax = float(input_node.find("tag[@k='ax']").attrib['v'])
        ay = float(input_node.find("tag[@k='ay']").attrib['v'])
        # params
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
        # create array
        utm_xy = np.array([[utm_x, utm_y]])
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
    print("utm_xy_arr : \n", utm_xy_arr) # [DEBUG]
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
        sys.exit(e)
        return None, None, None, None, None
    
def read_osm_tract(input_osm_track_file_name):
    """read osm track data."""
    # TODO
    return None
    
def read_csv_initset(input_csv_initset_file_name):
    """ Read and update initset.csv data. """
    try:
        initset_data = []
        with open(input_csv_initset_file_name, 'r', newline='') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            initset_data = list(csv_reader)
        # print(initset_data) # [DEBUG]
        initset_value = initset_data[0]['initset'].lower()
        # print(initset_value) # [DEBUG]
        if initset_value == 'false':
            initset_data[0]['initset'] = 'True'
            with open(input_csv_initset_file_name, 'w', newline='') as csv_file:
                fieldnames = initset_data[0].keys()
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                csv_writer.writeheader()
                csv_writer.writerows(initset_data)
        elif initset_value == 'true':
            print("Warning : Initial setting has been done before, it will be reset and performed again")
        else:
            sys.exit("Invalid value found for 'initset' in ./initset/initset.csv. It should be boolean value")
        return initset_data
    except Exception as e:
        sys.exit(e)
        return None
    
    
def read_csv_track_segment_indices(input_csv_initset_file_name):
    """ Read track segment indices data. """
    try:
        segment_indices = [[]]
        with open(input_csv_initset_file_name, 'r', newline='') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            initset_data = list(csv_reader)

        initset_value = initset_data[0]['initset'].lower()

        if initset_value == 'false':
            sys.exit("Do init setting first.")
        elif initset_value == 'true':
            num_columns = len(initset_data[0]) - 1
            # Initialize a list to store each column
            column_lists = [[] for _ in range(num_columns)]
            # Iterate through the rows in initset_data
            for row in initset_data:
                # Iterate through the columns (excluding 'initset' column)
                for i in range(1, num_columns + 1):
                    column_name = f'segment{i}'  
                    column_value = int(float(row[column_name]))  
                    column_lists[i - 1].append(column_value)
            return column_lists  
            # Transpose the column_lists to get segment_indices
            #segment_indices = list(map(list, zip(*column_lists)))
            # Remove the last list from segment_indices
            #segment_indices.pop()
            # Duplicate the 2nd list as the 3rd list
            #second_list = segment_indices[1]
            #segment_indices.insert(2, second_list.copy())
        else:
            print("Invalid value found for 'initset' in ./initset/initset.csv. It should be a boolean value.")
            sys.exit("Try init_setting again")
        # TODO : Modify the segment_indices format for optimization
        return segment_indices
    except Exception as e:
        sys.exit(e)
        return None