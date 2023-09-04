import argparse
import os
import sys
import numpy as np
import multiprocessing as mp


from converter import read_csv_track, read_osm_racingline, read_csv_best_alphas
from converter import convert2osm
from utils import xy2xy, remove_duplicate_points
from easydict import EasyDict as edict
from track import Track
from vehicle import Vehicle
from plot import plot_track, plot_segment
from racingline import RacingLine
from tuning import TuningParameter
from interface import UserInput
from interface import get_user_input_init, init_setting
from optimization import OptimizationResult


### Argument Parsing ###
parser = argparse.ArgumentParser(description='read track and vehicle_model')
# data input
parser.add_argument('--track', type=str, default='track', help='track filename')
parser.add_argument('--vehicle', type=str, default='ioniq5.json', help='vehicle model filename')
parser.add_argument('--engine', type=str, default='ioniq5_engine.json', help='engine model filename')
parser.add_argument('--best', type=str, default='best_racing_line.osm', help='best racing line filename')
parser.add_argument('--data', type=str, default='real_racing_line.osm', help='real racing data filename')
parser.add_argument('--initset', type=str, default='initset.csv', help='initset result filename')
parser.add_argument('--init_best', type=str, default='best_racing_line_alphas.csv', help='best racing line alphas filename')
parser.add_argument('--init_new', type=str, default='new_racing_line_alphas.csv', help='new racing line alphas filename')
# data output
parser.add_argument('--new', type=str, default='new_racing_line', help='new racing line filename')
# data type
parser.add_argument('--csv', action='store_true', help='input track data type is csv, default is osm')
# param
parser.add_argument('--default', type=str, default='default.json', help='default parameters filename')
parser.add_argument('--corner', type=str, default='corner.json', help='corner parameters filename')
# debug
parser.add_argument('--debug_stop', action='store_true', help='enable debug mode : stop program')
parser.add_argument('--debug_path', action='store_true', help='enable debug mode : path info')
parser.add_argument('--debug_track_raw', action='store_true', help='enable debug mode : track raw data')
parser.add_argument('--debug_tuning', action='store_true', help='enable debug mode : tuning parameter info')
parser.add_argument('--debug_track', action='store_true', help='enable debug mode : track info')
parser.add_argument('--debug_segment', action='store_true', help='enable debug mode : segment info')
parser.add_argument('--debug_init', action='store_true', help='enable debug mode : init alphas info')
# interface
parser.add_argument('--re', action='store_true', help='re-optimize with additional data')
parser.add_argument('--noopt', action='store_true', help='no optimization, visulaization only')
parser.add_argument('--init_setting', action='store_true', help='Automatic initial setting for racingline optimization')
# parsing arguments
configs = edict(vars(parser.parse_args()))

### Beginning of Code ###
print("[ Beginning of code ]\n")

### Check the # of CPU Cores ###
print("Check # of CPU cores for parallel computation")
print(mp.cpu_count(), mp.current_process().name)
print()

### Generate Path ###
print("Creating path...")
# input path
if configs.csv :
    path_track = configs.track + '.csv'
else :
    # TODO : use osm
    path_track = configs.track + '.csv'
    # path_track = configs.track + '.osm'
path_init_track = os.path.join('./initset/example/', path_track)
path_init_param_corner = os.path.join('./initset/example/', configs.corner)
path_init_param_default = os.path.join('./initset/example/', configs.default)
path_track = os.path.join('./track/', path_track)
path_vehicle = os.path.join('./model/vehicle/', configs.vehicle)
path_engine = os.path.join('./model/engine/', configs.engine)
path_best_racingline = os.path.join('./best/', configs.best)
path_initset = os.path.join('./initset/', configs.initset)
path_init_best= os.path.join('./best/', configs.init_best)
path_init_new= os.path.join('./result/', configs.init_new)
path_data_racingline = os.path.join('./data/', configs.data)
path_param_default = os.path.join('./param/', configs.default)
path_param_corner = os.path.join('./param/', configs.corner)
# output path
path_new_racingline  = os.path.join('./result/', configs.new + '.osm')
path_new_racingline_plot = os.path.join('./result/', configs.new + '_plot.png')
path_new_racingline_result = os.path.join('./result/', configs.new + '_result.txt')
path_new_racingline_alphas = os.path.join('./result/', configs.new + '_alphas.csv')
print("Done!\n")
if configs.debug_path:
    print("[ Path ]")
    print("1. INPUT PATH")
    print("init track   :", path_init_track)
    print("init corner  :", path_init_param_corner)
    print("init default :", path_init_param_default)
    print("track        :", path_track)
    print("vehicle      :", path_vehicle)
    print("engine       :", path_engine)
    print("best line    :", path_best_racingline)
    print("initset      :", path_initset)
    print("init best    :", path_init_best)
    print("init new     :", path_init_new)
    print("data         :", path_data_racingline)
    print("corner       :", path_param_corner)
    print("default      :", path_param_default)
    print("2. OUTPUT PATH")
    print("new line             : ", path_new_racingline)
    print("new plot             : ", path_new_racingline_plot)
    print("new result           : ", path_new_racingline_result)
    print("new alphas           : ", path_new_racingline_alphas)

### Open file to store the optimization results ###
if not configs.init_setting :
    new_line = "[ New Optimization Result ]\n"
    with open(path_new_racingline_result, 'w') as file:
        file.write(new_line)

### Read track data ###
print("Read track data...")
if not configs.init_setting :
    if configs.csv :
        mid_xy_arr, wl_arr, wr_arr, slope_arr, mu_arr = read_csv_track(path_track)
    else :
        # TODO : use osm (Not implemented yet)
        mid_xy_arr, wl_arr, wr_arr, slope_arr, mu_arr = read_csv_track(path_track)
else :
    if configs.csv :
        mid_xy_arr, wl_arr, wr_arr, slope_arr, mu_arr = read_csv_track(path_init_track)
    else :
        # TODO : use osm (Not implemented yet)
        mid_xy_arr, wl_arr, wr_arr, slope_arr, mu_arr = read_csv_track(path_init_track)
filtered_mid_xy_arr, filtered_arrays = remove_duplicate_points(mid_xy_arr, [wl_arr, wr_arr, slope_arr, mu_arr])
wl_arr, wr_arr, slope_arr, mu_arr = filtered_arrays
mid_xy_arr = filtered_mid_xy_arr
print("Done!\n")
if configs.debug_track_raw:
    print("[ Track Raw Data ]")
    print("track mid xy arr         \n", mid_xy_arr)
    print("left width arr           \n", wl_arr)
    print("right width  arr         \n", wr_arr)
    print("road slope arr           \n", slope_arr)
    print("mu arr                   \n", mu_arr)

### Read references ###
if not configs.init_setting :
    print("Reading reference racing lines...")
    # racing lines
    if configs.re or configs.noopt :
        best_racingline = read_osm_racingline(path_best_racingline)
    if configs.noopt :
        new_racingline = read_osm_racingline(path_new_racingline)
    if configs.re :
        data_racingline = read_osm_racingline(path_data_racingline)
    # alpha values for initial racing line
    if configs.re :
        init_best_alphas_left, init_best_alphas_right = read_csv_best_alphas(path_init_best)
        init_new_alphas_left, init_new_alphas_right = read_csv_best_alphas(path_init_new)
    print("Done!\n")
    if configs.re and configs.debug_init:
        print("best alphas (left)")
        print(init_best_alphas_left)
        print("best alphas (right)")
        print(init_best_alphas_right)
        print("new alphas (left)")
        print(init_new_alphas_left)
        print("new alphas (right)")
        print(init_new_alphas_right)

### Create tuning parameters ###
print("Creating tuning parameters...")
if configs.init_setting :
    tuning_parameter = TuningParameter(mid_xy_arr, path_init_param_corner, path_init_param_default)
else :
    tuning_parameter = TuningParameter(mid_xy_arr, path_param_corner, path_param_default)
# update track data (measured slope and friction coefficient data)
tuning_parameter.update_with_data(road_slope_rad=slope_arr, mu=mu_arr)
tuning_parameter.update_default()
print("Done!\n")

### Create track ###
print("Creating track...")
track = Track(mid_xy_arr, wl_arr, wr_arr, slope_arr, mu_arr, \
              tuning_parameter.alpha_left_max, tuning_parameter.alpha_right_max)
print("Done!\n")
if configs.debug_track:
    print("[ Track Info ]")
    print("track is closed   : ", track.closed)
    print("track size        : ", track.size)
    print("track samples     : ", track.mid_s)
if configs.debug_tuning:
    print("[ Tuning Parameter ]")
    print("1. DEFAULT")
    print("mu                   :", tuning_parameter.default_mu)
    print("ax_upper_limit       :", tuning_parameter.default_ax_upper_limit)
    print("ax_lower_limit       :", tuning_parameter.default_ax_lower_limit)
    print("ay_upper_limit       :", tuning_parameter.default_ay_upper_limit)
    print("ay_lower_limit       :", tuning_parameter.default_ay_lower_limit)
    print("Cb                   :", tuning_parameter.default_Cb)
    print("Cd                   :", tuning_parameter.default_Cd)
    print("alpha_left_max       :", tuning_parameter.default_alpha_left_max)
    print("alpha_right_max      :", tuning_parameter.default_alpha_right_max)
    print("road_slope_rad       :", tuning_parameter.default_road_slope_rad)
    print("2. CORNER")
    print("k_min                :", tuning_parameter.k_min)
    print("straight_length_min  :", tuning_parameter.straight_length_min)
    print("corner_length_min    :", tuning_parameter.corner_length_min)
    print("3. PARAMS")
    print("mu                   :", tuning_parameter.mu)
    # print("ax_upper_limit       :", tuning_parameter.ax_upper_limit)
    # print("ax_lower_limit       :", tuning_parameter.ax_lower_limit)
    # print("ay_upper_limit       :", tuning_parameter.ax_upper_limit)
    # print("ay_lower_limit       :", tuning_parameter.ay_lower_limit)
    # print("Cb                   :", tuning_parameter.Cb)
    # print("Cd                   :", tuning_parameter.Cd)
    # print("alpha_left_max       :", tuning_parameter.alpha_left_max)
    # print("alpha_right_max      :", tuning_parameter.alpha_right_max)
    # print("road_slope_rad       :", tuning_parameter.road_slope_rad)   

### Analyze the segments of track and summarize ###
# [param] corner detection parameters
print("Analyzing the segments of track...")
corners, straights = track.segments(tuning_parameter.k_min, \
                                    tuning_parameter.corner_length_min, \
                                    tuning_parameter.straight_length_min)
print("[ Track Segments Info ]")
if corners.ndim == 1:
    if corners.size == 0:
        print("Corners : Not exists ")
        num_of_corner = 0
    else : 
        print("Corners : 1")
        num_of_corner = 1
else : 
    print("Corners : 1 ~", corners.shape[0])
    num_of_corner = corners.shape[0]

if straights.ndim == 1:
    if straights.size == 0:
        print("Straights : Not exists ")
        num_of_straight = 0
    else : 
        print("Straights : 1")
        num_of_straight = 1
else : 
    print("Straights : 1 ~", straights.shape[0])
    num_of_straight = straights.shape[0]
print("Done!\n")
if configs.debug_segment:
    print("Corners      \n", corners)
    print("Straights    \n", straights)
    print()

### Initial Setting ###
if configs.init_setting :
    init_setting(corners, straights, track, path_initset)
    pass

### Read vehicle data from json file ###
print("Reading vehicle data...")
vehicle = Vehicle(path_vehicle, path_engine)
print("Done!")

### Upate tuning parameters with real racing data if exists ###
if configs.re :
    print("Update tuning parameters with real racing data...")
    if data_racingline is not None :
        # parse the real racing line data
        data_src_xy_arr, data_v_arr, data_ax_arr, data_ay_arr, \
        data_mu_arr, \
        data_ax_upper_limit_arr, data_ax_lower_limit_arr, \
        data_ay_upper_limit_arr, data_ay_lower_limit_arr, \
        data_Cb_arr, data_Cd_arr, \
        data_alpha_left_max_arr, data_alpha_right_max_arr, \
        data_road_slope_rad_arr = data_racingline
        data_src_is_closed = track.closed
        data_dest_xy_arr = track.mid_xy_coordinates[:-1] if track.closed else track.mid_xy_coordinates
        data_dest_is_closed = track.closed
        # conver the data to track size
        data_mu_arr = xy2xy(data_mu_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_ax_upper_limit_arr = xy2xy(data_ax_upper_limit_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_ax_lower_limit_arr = xy2xy(data_ax_lower_limit_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_ay_upper_limit_arr = xy2xy(data_ay_upper_limit_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_ay_lower_limit_arr = xy2xy(data_ay_lower_limit_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_Cb_arr = xy2xy(data_Cb_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_Cd_arr = xy2xy(data_Cd_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_alpha_left_max_arr = xy2xy(data_alpha_left_max_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_alpha_right_max_arr = xy2xy(data_alpha_right_max_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        data_road_slope_rad_arr = xy2xy(data_road_slope_rad_arr, data_src_xy_arr, data_src_is_closed, data_dest_xy_arr, data_dest_is_closed)
        # update tuning parameters with converted data
        tuning_parameter.update_with_data(mu=data_mu_arr,\
                                          ax_upper_limit=data_ax_upper_limit_arr,\
                                          ax_lower_limit=data_ax_lower_limit_arr,\
                                          ay_upper_limit=data_ax_upper_limit_arr,\
                                          ay_lower_limit=data_ay_lower_limit_arr,\
                                          Cb=data_Cb_arr, Cd=data_Cd_arr,\
                                          alpha_left_max=data_alpha_left_max_arr,\
                                          alpha_right_max=data_alpha_right_max_arr,\
                                          road_slope_rad=data_road_slope_rad_arr)
    else :
        print("Warning : No real racing line data. update tuning parameter with default values")
    tuning_parameter.update_default()
    print("Done!\n")
if configs.re and configs.debug_tuning:
    print("4. UPDATED PARAMS")
    print("mu                   :", tuning_parameter.mu)
    # print("ax_upper_limit       :", tuning_parameter.ax_upper_limit)
    # print("ax_lower_limit       :", tuning_parameter.ax_lower_limit)
    # print("ay_upper_limit       :", tuning_parameter.ax_upper_limit)
    # print("ay_lower_limit       :", tuning_parameter.ay_lower_limit)
    # print("Cb                   :", tuning_parameter.Cb)
    # print("Cd                   :", tuning_parameter.Cd)
    # print("alpha_left_max       :", tuning_parameter.alpha_left_max)
    # print("alpha_right_max      :", tuning_parameter.alpha_right_max)
    # print("road_slope_rad       :", tuning_parameter.road_slope_rad)

### Optimization & Visualization ###
save = False
if configs.noopt:
    """ Visualization only """
    if new_racingline is None :
        print("Warning: new racingline is not exists, please check result folder or file name")  
    if best_racingline is None :
        print("Warning: best racingline is not exists, please check best folder or file name")  
    plot_track(track, corners, straights, new=new_racingline, best=best_racingline)
else:
    """ Optimize track """
    # create racing line
    racingline = RacingLine(track, vehicle, tuning_parameter)
    # take user input of initial racing line
    if configs.re :
        user_input_init = get_user_input_init()
    else : 
        user_input_init = None
    print("User Input : ", user_input_init) # [DEBUG]
    # do optimization
    if user_input_init == None :
        # TODO : First opt
        sys.exit("First opt is not implemented yet")
    elif user_input_init == UserInput.QUIT : 
        sys.exit("Quit by user")
    elif user_input_init == UserInput.RE: # First 
        print("\n[ Re-opt : completely re-optimization without use of initial racing line ]")
        # run optimization
        opt_res = racingline.minimize_laptime()
        # print optimization result
        print("[ Racing Initialization Result ]")
        if opt_res.success:
            print("Optimization successful.")
        else:
            if opt_res.max_iteration_is_reached:
                print("Optimization did not converge and stopped due to reaching iteration limit.")
            else:
                print("Optimization did not converge and did not reach iteration limit.")
        print("terminal condition   = ", opt_res.message)
        print("Final iteration      = ", opt_res.iteration)
        print("Final cost           = ", opt_res.cost)
        print("Run time             = {:.3f}".format(opt_res.run_time))
        print("Lap time             = {:.3f}".format(racingline.lap_time()))
        
        # save results
        # plot (png)
        save = True
        plot_track(track, corners, straights, racingline=racingline, save=save, path_plot=path_new_racingline_plot)
        # opt results (txt)
        with open(path_new_racingline_result, 'a') as result_file:
            if opt_res.success:
                result_file.write("Optimization successful.\n")
            else:
                if opt_res.max_iteration_is_reached :
                    result_file.write("Optimization did not converge and stopped due to reaching iteration limit.\n")
                else:
                    result_file.write("Optimization did not converge and did not reach iteration limit.\n")
            result_file.write(f"terminal condition   : {opt_res.message}\n")
            result_file.write("run time             : {:.3f} s\n".format(opt_res.run_time))
            result_file.write("iteration            : {} times\n".format(opt_res.iteration))
            result_file.write("cost                 : {:.3f}\n".format(opt_res.cost))
        # alphas (csv)
        combined_alphas = np.column_stack((opt_res.alphas_left, opt_res.alphas_right))
        header = "alphas_left,alphas_right"
        np.savetxt(path_new_racingline_alphas, combined_alphas, delimiter=",", header=header, comments="")
        # osm
        src_xy_arr = track.mid_xy_coordinates[:-1] if track.closed else track.mid_xy_coordinates
        src_is_closed = track.closed
        dest_xy_arr = racingline.xy_arr
        dest_is_closed = False if racingline.length_closed is None else True
        # print(racingline.xy_arr) # [DEBUG]
        racingline.alphas_left = xy2xy(racingline.alphas_left, src_xy_arr, src_is_closed, dest_xy_arr, dest_is_closed)
        # print(racingline.xy_arr) # [DEBUG]
        racingline.alphas_right = xy2xy(racingline.alphas_right, src_xy_arr, src_is_closed, dest_xy_arr, dest_is_closed)
        convert2osm(path_new_racingline, \
                    racingline.xy_arr, \
                    racingline.velocity, \
                    racingline.alphas_left, racingline.alphas_right)
    elif user_input_init == UserInput.BEST :
        init_alphas_left = init_best_alphas_left
        init_alphas_right = init_best_alphas_right
    else : # user_input_init == UserInput.New
        init_alphas_left = init_new_alphas_left
        init_alphas_right = init_new_alphas_right

# [DEBUG]
if configs.debug_stop:
    sys.exit("Stop for debugging")

print("[ End of Code ]")

##################################################################################
        
        # print("Lap time = {:.3f}".format(racingline.lap_time()))
        # Create a thread that runs the optimize_track function
        
        # plot_lock = threading.Lock()
        # plot_thread  = threading.Thread(target=plot_optimization_realtime, args=(plot_lock, iterations, costs))
        # plot_thread.daemon = True
        # plot_thread.start()
        # initialization_time = racingline.generate_initial_alphas(iterations, costs)
        # plot_thread.join()
        # initialization_time = racingline.generate_initial_alphas(iterations, costs)
        # initialization_time = racingline.minimise_lap_time()
        # print the result of initialization
        # print("Lap time = {:.3f}".format(racingline.lap_time()))
        # print("Run time = {:.3f}".format(initialization_time))
        # print("[ Optimizing Racing Line ]")        





# # Select the segment to make further optimization
# if not configs.nosegmentplot:
#     print("[ Select segment to further optimize ]")
#     print("  To plot a corner i, enter ci (e.g., c2 for corner 2)")
#     print("  To plot a straight j, enter sj (e.g., s4 for straight 4)")

#     while True:
#         user_input = input("Enter the segment you want to plot (e.g., c2 or s4): ")
#         if user_input.startswith("c") and user_input[1:].isdigit():
#             corner_index = int(user_input[1:])
#             if 1 <= corner_index <= corners.shape[0]:
#                 # Plot the specified corner
#                 plot_segment(track, corners[corner_index - 1], user_input)
#                 break
#             else:
#                 print("Invalid corner index. Please enter a valid index.\n")
#         elif user_input.startswith("s") and user_input[1:].isdigit():
#             straight_index = int(user_input[1:])
#             if 1 <= straight_index <= straights.shape[0]:
#                 # Plot the specified straight
#                 plot_segment(track, straights[straight_index - 1], user_input)
#                 break
#             else:
#                 print("Invalid straight index. Please enter a valid index.\n")
#         else:
#             print("Invalid input format. Please enter a valid input.\n")
#     print("End of Code")
