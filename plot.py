import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time
import threading

from track import Track
from racingline import RacingLine
from unit import mps2kph


###############################################################################

def plot_track(track, corners, straights, racingline=None, best=None, \
               new=None, save=False , path_plot=None):
    if not isinstance(track, Track):
        raise ValueError("Input must be an instance of the Track class")
    track_size = len(track.mid_line)

    # Set color
    straight_R = 123
    straight_G = 142
    straight_B = 169
    corner_R = 248
    corner_G = 155
    corner_B = 108 

    color_straight = (straight_R/255, straight_B/255, straight_B/255)
    color_corner = (corner_R/255, corner_G/255, corner_B/255)
    hex_color_straight = "#{:02X}{:02X}{:02X}".format(straight_R, straight_G, straight_B)
    hex_color_corner = "#{:02X}{:02X}{:02X}".format(corner_R, corner_G, corner_B)

    # Plot the graph
    fig = plt.figure(figsize=(12, 8))
    fig.set_facecolor('white') ## 캔버스 색상 설정
    subplot = fig.add_subplot() ## 프레임(그림 뼈대) 생성
    subplot.plot([point[0] for point in track.mid_line], [point[1] for point in track.mid_line], \
             'k.', markersize=0.1, marker='x', label='Midline', zorder=2)
    
    # Plot segment information

    if corners.ndim == 1:
        if corners.size == 0:
            # print("No corners")
            pass
        else:
            start = corners[0]
            end = corners[1]
            if end < start:
                    end = end + track_size
            for j in range(start, end):
                current_idx = j % track_size
                next_idx = (j + 1) % track_size
                subplot.plot([track.left_boundary[current_idx][0], track.left_boundary[next_idx][0]], \
                         [track.left_boundary[current_idx][1], track.left_boundary[next_idx][1]], \
                         color=color_corner, linewidth=2.5)
                subplot.plot([track.right_boundary[current_idx][0], track.right_boundary[next_idx][0]], \
                         [track.right_boundary[current_idx][1], track.right_boundary[next_idx][1]], \
                         color=color_corner, linewidth=2.5)  
            subplot.scatter(track.left_boundary[start][0], track.left_boundary[start][1], s=50, c='red', marker='o', label=f'c{1}')
            subplot.scatter(track.right_boundary[start][0], track.right_boundary[start][1], s=50, c='red', marker='o')
            subplot.annotate(f'c{1}', (track.left_boundary[start][0], track.left_boundary[start][1]), \
                         textcoords="offset points", xytext=(5, 10), ha='center', fontsize=10, color='red')
    elif corners.ndim == 2: 
        for i, (start, end) in enumerate(corners):
            if end < start:
                end = end + track_size
            for j in range(start, end):
                current_idx = j % track_size
                next_idx = (j + 1) % track_size
                subplot.plot([track.left_boundary[current_idx][0], track.left_boundary[next_idx][0]], \
                         [track.left_boundary[current_idx][1], track.left_boundary[next_idx][1]], \
                         color=color_corner, linewidth=2.5)
                subplot.plot([track.right_boundary[current_idx][0], track.right_boundary[next_idx][0]], \
                         [track.right_boundary[current_idx][1], track.right_boundary[next_idx][1]], \
                         color=color_corner, linewidth=2.5)  
            subplot.scatter(track.left_boundary[start][0], track.left_boundary[start][1], s=50, c='red', marker='o', label=f'c{i + 1}')
            subplot.scatter(track.right_boundary[start][0], track.right_boundary[start][1], s=50, c='red', marker='o')
            subplot.annotate(f'c{i + 1}', (track.left_boundary[start][0], track.left_boundary[start][1]), \
                         textcoords="offset points", xytext=(5, 10), ha='center', fontsize=10, color='red')
    else:
        raise ValueError(" Wrong dimension of corners ")

    if straights.ndim == 1:
        if straights.size == 0 :
            # print("No straights")
            pass
        else:
            start = straights[0]
            end   = straights[1]
            if end < start:
                    end = end + track_size
            for j in range(start, end):
                current_idx = j % track_size
                next_idx = (j + 1) % track_size
                subplot.plot([track.left_boundary[current_idx][0], track.left_boundary[next_idx][0]], \
                         [track.left_boundary[current_idx][1], track.left_boundary[next_idx][1]], \
                         color=color_straight, linewidth=2.5)  # Change the line color for straights to blue
                subplot.plot([track.right_boundary[current_idx][0], track.right_boundary[next_idx][0]], \
                         [track.right_boundary[current_idx][1], track.right_boundary[next_idx][1]], \
                         color=color_straight, linewidth=2.5)  # Change the line color for straights to blue
            subplot.scatter(track.left_boundary[start][0], track.left_boundary[start][1], s=50, c='blue', marker='s', label=f's{1}')
            subplot.scatter(track.right_boundary[start][0], track.right_boundary[start][1], s=50, c='blue', marker='s')
            subplot.annotate(f's{1}', (track.left_boundary[start][0], track.left_boundary[start][1]), \
                         textcoords="offset points", xytext=(5, 10), ha='center', fontsize=10, color='blue')
    elif straights.ndim == 2:
        for i, (start, end) in enumerate(straights):
            if end < start:
                end = end + track_size
            for j in range(start, end):
                current_idx = j % track_size
                next_idx = (j + 1) % track_size
                subplot.plot([track.left_boundary[current_idx][0], track.left_boundary[next_idx][0]], \
                         [track.left_boundary[current_idx][1], track.left_boundary[next_idx][1]], \
                         color=color_straight, linewidth=2.5)  # Change the line color for straights to blue
                subplot.plot([track.right_boundary[current_idx][0], track.right_boundary[next_idx][0]], \
                         [track.right_boundary[current_idx][1], track.right_boundary[next_idx][1]], \
                         color=color_straight, linewidth=2.5)  # Change the line color for straights to blue
            subplot.scatter(track.left_boundary[start][0], track.left_boundary[start][1], s=50, c='blue', marker='s', label=f's{i + 1}')
            subplot.scatter(track.right_boundary[start][0], track.right_boundary[start][1], s=50, c='blue', marker='s')
            subplot.annotate(f's{i + 1}', (track.left_boundary[start][0], track.left_boundary[start][1]), \
                         textcoords="offset points", xytext=(5, 10), ha='center', fontsize=10, color='blue')
    else:
        raise ValueError(" Wrong dimension of straights ")
    
    if racingline:
        if not isinstance(racingline, RacingLine):
            raise ValueError("racingline parameter must be an instance of RacingLine")
        
        # subplot.plot(racingline.xy_arr[0], racingline.xy_arr[1], 'b.', markersize=1, label='Racing line')
        subplot.plot(racingline.xy_arr[0], racingline.xy_arr[1], color='blue', linewidth=1.5, label='Racing line', zorder=5)

        # Plot velocity information
        size = len(racingline.xy_arr[0]) - int(racingline.track.closed)
        # print("# of control points of racing line : ",size) # [DEBUG]
        # print("# of K : ",len(racingline.curvature)) # [DEBUG]
        # print("# of V : ",len(racingline.velocity.v)) # [DEBUG]
        # print("# of ax : ",len(racingline.velocity.ax)) # [DEBUG]
        # print("# of ay : ",len(racingline.velocity.ay)) # [DEBUG]
        for i in range(size):
            x = racingline.xy_arr[0][i]
            y = racingline.xy_arr[1][i]
            k = 1/racingline.curvature[i]
            v_max = mps2kph(racingline.velocity.v_max[i])
            v_acc_limit = mps2kph(racingline.velocity.v_acc_limit[i])
            v_dec_limit = mps2kph(racingline.velocity.v_dec_limit[i])
            v = mps2kph(racingline.velocity.v[i])
            ax = racingline.velocity.ax[i]
            ay = racingline.velocity.ay[i]
            # Plot velocity markers [for debugging]
            # subplot.text(x, y, f'{v:.1f}', color='green', fontsize=2, ha='left', va='bottom')
            # subplot.text(x, y, f'{ax:.2f}', color='orange', fontsize=10, ha='right', va='bottom')
            # subplot.text(x, y, f'{ay:.1f}', color='red', fontsize=10, ha='left', va='top')
            # subplot.text(x, y, f'{k:.1f}', color='blue', fontsize=2, ha='right', va='top')
        
    # Plot boundaries for optimization
    subplot.plot([point[0] for point in track.left_opt_boundary], [point[1] for point in track.left_opt_boundary], \
              color='gray', linestyle='--', linewidth=0.5, label='Left Boundary')
    subplot.plot([point[0] for point in track.right_opt_boundary], [point[1] for point in track.right_opt_boundary], \
              color='gray', linestyle='--', linewidth=0.5, label='Right Boundary')
    
    # Plot best and new racing line
    if best :
        best_xy_arr = best[0]
        subplot.plot(best_xy_arr[0], best_xy_arr[1], color='red', linewidth=1.5, label='Best racingline', zorder=0)
    if new :
        new_xy_arr = new[0]
        subplot.plot(new_xy_arr[0], new_xy_arr[1], color='blue', linestyle='-', linewidth=1.5, label='New racingline', zorder=1)
    
    # Legend handling
    prop = dict(
    family='DejaVu Sans', 
    style='italic',
    size=14 
    )
    config_legend = dict(
    loc = 'upper left',
    prop=prop,
    edgecolor='k' #
    )
    handles, labels = subplot.get_legend_handles_labels()
    dict_labels_handles = dict(zip(labels, handles))
    if best and new :
        labels = ['Best racingline', 'New racingline']
        
    elif best :
        labels = ['Best racingline', 'Racing line']
    else :
        labels = ['Racing line']
    handles = [dict_labels_handles[l] for l in labels]
    subplot.legend(handles, labels, **config_legend)

    # Plot
    plt.title('Track Info')
    plt.axis('equal')
    plt.xticks(rotation=40)
    plt.yticks(rotation=40)
    if save:
        plt.show(block=False)
        if path_plot == None :
            raise ValueError("No path to save the plot")
        plt.savefig(path_plot, dpi=600)
    else: 
        # print(" show the plot ", save) #[DEBUG]
        plt.show() 

def plot_segment(track, segment_idx, user_input):
    if not isinstance(track, Track):
        raise ValueError("Input must be an instance of the Track class")
    
    # Ensure segment_idx is within a valid range
    if segment_idx[0] < 0 or segment_idx[1] >= len(track.left_boundary):
        raise ValueError("Invalid segment_idx")
    
    start_idx, end_idx = segment_idx
    
    if start_idx <= end_idx:
        segment_points_left = track.left_boundary[start_idx:end_idx + 1]
        segment_points_mid = track.mid_line[start_idx:end_idx + 1]
        segment_points_right = track.right_boundary[start_idx:end_idx + 1]
    else:
        segment_points_left = np.concatenate((track.left_boundary[start_idx:], track.left_boundary[:end_idx + 1]))
        segment_points_mid = np.concatenate((track.mid_line[start_idx:], track.mid_line[:end_idx + 1]))
        segment_points_right = np.concatenate((track.right_boundary[start_idx:], track.right_boundary[:end_idx + 1]))
    
    # Plot the segment
    plt.figure(figsize=(15, 10))
    plt.plot([point[0] for point in segment_points_left], [point[1] for point in segment_points_left], \
              color='black', linewidth=0.5, label='Left Boundary')
    plt.plot([point[0] for point in segment_points_mid], [point[1] for point in segment_points_mid], 'k.', markersize=0.3, label='Midline')
    plt.plot([point[0] for point in segment_points_right], [point[1] for point in segment_points_right], \
              color='black', linewidth=0.5, label='Right Boundary')
    plt.title(user_input)
    plt.axis('equal')
    plt.xticks([])
    plt.yticks([])
    plt.show()

# def plot_optimization_realtime(plot_lock, iterations, costs, xy_arr=None, track=None) :
#     with plot_lock:
#         prev_iterations_size = len(iterations)
#         prev_costs_size = len(costs)
#         is_updated = True
    
#     fig, ax = plt.subplots()
#     ax.set_xlabel('Iteration')
#     ax.set_ylabel('Cost')
#     line, = ax.plot(iterations, costs, label='Cost')
#     ax.legend()
#     plt.ion()  # Turn on interactive mode for real-time updating
   
#     while is_updated:
#         with plot_lock:
#             print(len(iterations))
#             # print(len(costs))
#             if len(iterations) == 15100 :
#                 plt.close()
#                 return
#             if len(iterations) == prev_iterations_size and len(costs) == prev_costs_size:
#                 time.sleep(0.005)  # 5ms
#                 continue
#             elif len(iterations) != prev_iterations_size or len(costs) != prev_costs_size:
#                 prev_iterations_size = len(iterations)
#                 prev_costs_size = len(costs)
#                 line.set_data(iterations, costs)
#                 ax.relim()
#                 ax.autoscale_view()
#                 plt.pause(0.1)  # Pause to allow the plot to update
#             else:
#                 raise ValueError(" Inconsistence of iterations and costs. ")

# def plot_realtime(iteration, cost, end, num_of_corner, lock, \
#                   xy_arr=None, track=None) :
#     plt.ion()
#     iterations = []
#     costs = []
#     prev_iteration = 0
#     prev_cost = 0
#     fig1, ax1 = plt.subplots()
#     ax1.set_xlabel('Iteration')
#     ax1.set_ylabel('Cost')
#     line, = ax1.plot(iterations, costs, label='Cost')
#     ax1.legend()
#     # print("Start") #[DEBUG]
#     while not end == 0 :
#         with lock:
#             # print("iteration        : ", iteration.value) #[DEBUG]
#             # print("prev iteration   : ", prev_iteration) #[DEBUG]
#             if prev_iteration == iteration.value :
#                 time.sleep(0.05) # 5ms
#                 continue
#             else:
#                 prev_iteration = iteration.value
#                 iterations.append(iteration.value)
#                 costs.append(cost.value)
#                 if iteration.value > 1000 :
#                     iterations = iterations[:-1000]
#                     costs = costs[:-1000]
#                 line.set_data(iterations, costs)
#                 ax1.relim()
#                 ax1.autoscale_view()
#                 plt.pause(0.01)
#     # print("Done") #[DEBUG]



        

